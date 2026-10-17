#!/usr/bin/env python3
"""Offline numerical probe of the dance assets and 1 kHz / 50 Hz ZOH contract.

Requires numpy, torch, mujoco and PyYAML (or ruamel.yaml). This is a Python
simulation, not a build/test of the Ubuntu ROS 2 executable or DCU hardware.
It uses the training MJCF, identified armature, effort saturation and the
dedicated deployment YAML. No ROS messages, motors or viewers are opened.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import struct
from pathlib import Path

import mujoco
import numpy as np
import torch


def load_yaml(path):
    try:
        import yaml
        return yaml.safe_load(path.read_text())
    except ImportError:
        from ruamel.yaml import YAML
        return YAML(typ="safe").load(path)


def run(training_root: Path, stand_hold_s: float, yaw: float, ready_timeout_s: float,
        candidate: str = "configured") -> dict:
    control = Path(__file__).resolve().parents[1]
    cfg = load_yaml(control / "cfg/rl_x1_dance.yaml")
    conf = cfg["controllers"]["dance_x1"]
    assert cfg["control_frequecy"] == 1000 and conf["decimation"] == 20
    assert conf["prepare_ticks"] == 0 and not conf["joint_sign"]
    metadata = json.loads((control / "policy/dance_x1_v11_4999.json").read_text())
    for name, digest in metadata["assets"].items():
        assert hashlib.sha256((control / "policy" / name).read_bytes()).hexdigest() == digest

    # Use the independent training playback's quaternion/observation utilities.
    playback = training_root / "whole_body_tracking/scripts/play_mujoco.py"
    spec = importlib.util.spec_from_file_location("x1_reference_math", playback)
    math = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(math)
    names = conf["joint_list"]
    assert names == list(metadata["joints"])
    blob = (control / "policy/dance_x1_v11_4999.bin").read_bytes()
    assert blob[:8] == b"X1DNCE2\0" and struct.unpack_from("<7I", blob, 8) == (2, 29, 160, 512, 256, 128, 29)
    offset = 36
    for name in names:
        assert blob[offset:offset + 32].split(b"\0", 1)[0].decode() == name
        offset += 32

    def read(count):
        nonlocal offset
        value = np.frombuffer(blob, dtype="<f4", count=count, offset=offset).copy()
        offset += count * 4
        return value

    default, kp, kd, effort, scale, armature = [read(29) for _ in range(6)]
    mean, inv_std = read(160), read(160)
    layers = []
    for n_in, n_out in zip((160, 512, 256, 128), (512, 256, 128, 29)):
        layers.append((torch.from_numpy(read(n_in * n_out).reshape(n_out, n_in)),
                       torch.from_numpy(read(n_out))))
    assert offset == len(blob)

    def actor(raw):
        value = torch.from_numpy((raw - mean) * inv_std)
        for index, (weight, bias) in enumerate(layers):
            value = torch.nn.functional.linear(value, weight, bias)
            if index < 3:
                value = torch.nn.functional.elu(value)
        return value.numpy()

    motion = np.load(training_root / "whole_body_tracking/motions/dance_x1_train.npz")
    order = [list(motion["joint_names"]).index(name) for name in names]
    ref_q, ref_v = motion["joint_pos"][:, order], motion["joint_vel"][:, order]
    anchor_index = list(motion["body_names"]).index(conf["anchor_frame"])
    ref_pos = motion["body_pos_w"][:, anchor_index]
    ref_quat = motion["body_quat_w"][:, anchor_index]
    lower = np.array([cfg["joint_limits"][name]["lower"] for name in names])
    upper = np.array([cfg["joint_limits"][name]["upper"] for name in names])
    assert np.all(ref_q >= lower - 1e-4) and np.all(ref_q <= upper + 1e-4)

    model_path = training_root / "whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/xyber_x1/mjcf/xyber_x1_flat.xml"
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.timestep = 0.001
    data = mujoco.MjData(model)
    joints = np.array([model.joint(name).id for name in names])
    qadr, vadr = model.jnt_qposadr[joints], model.jnt_dofadr[joints]
    actuators = np.array([np.flatnonzero(model.actuator_trnid[:, 0] == joint).item() for joint in joints])
    model.dof_armature[vadr] = armature
    model.dof_damping[vadr] = 0
    model.dof_frictionloss[vadr] = 0
    anchor = model.body(conf["anchor_frame"]).id

    # Start at the same shallow stand used by the proven AMP deployment.
    data.qpos[2] = 0.7
    data.qpos[3:7] = [np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
    stand_kp, stand_kd = np.zeros(29), np.zeros(29)
    for controller in (cfg["controllers"]["pd_zero"], cfg["controllers"]["pd_stand"]):
        for name, q, p, d in zip(controller["joint_list"], controller["init_state"],
                                 controller["stiffness"], controller["damping"]):
            index = names.index(name)
            data.qpos[qadr[index]], stand_kp[index], stand_kd[index] = q, p, d
    mujoco.mj_forward(model, data)
    feet = [model.body(side + "_ankle_roll_link").id for side in ("left", "right")]
    foot_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in feet
                  and model.geom_contype[g] and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
    data.qpos[2] -= min(data.geom_xpos[g, 2] - model.geom_size[g, 0] for g in foot_geoms) - 0.001
    mujoco.mj_forward(model, data)
    stand = data.qpos[qadr].copy()
    for _ in range(round(stand_hold_s * 1000)):
        data.ctrl[actuators] = np.clip(stand_kp * (stand - data.qpos[qadr]) - stand_kd * data.qvel[vadr], -effort, effort)
        mujoco.mj_step(model, data)

    # One Guide from stand authorizes preparation -> automatic playback.
    # Inference/reference time remain stopped during preparation and readiness hold.
    pose_cfg = cfg["controllers"]["pd_dance_pose"]
    assert pose_cfg["joint_list"] == names
    pose_target = np.asarray(pose_cfg["init_state"])
    if candidate == "configured":
        profile = conf["start_pose"]
        if profile not in ("default_q", "motion_start"):
            raise ValueError("unknown configured start pose")
        np.testing.assert_allclose(pose_target, default if profile == "default_q" else ref_q[0], atol=1e-6, rtol=0)
    else:
        # Offline experiments only. Production C++ still enforces the configured
        # reference-start pose. Candidate entry gating below is hypothetical.
        pose_target = default.astype(float).copy()
        if candidate.startswith("amp"):
            pose_target = stand.copy()
        if candidate == "mid_crouch":
            pose_target = (default + stand) / 2
        if candidate.endswith("neutral_elbows"):
            for side in ("left", "right"):
                pose_target[names.index(side + "_elbow_pitch_joint")] = 0.3
                pose_target[names.index(side + "_shoulder_roll_joint")] = -0.1
        if candidate.startswith("ref_blend_"):
            alpha = float(candidate.rsplit("_", 1)[1]) / 100
            pose_target = default + alpha * (ref_q[0] - default)
    candidate_context = {"preparation_candidate": candidate,
                         "configured_start_pose": conf["start_pose"],
                         "experimental_candidate_entry_gate": candidate != "configured"}
    pose_kp, pose_kd = np.asarray(pose_cfg["stiffness"]), np.asarray(pose_cfg["damping"])
    transition_ticks = round(pose_cfg["transition_duration_s"] * 1000)
    prepare_start = data.qpos[qadr].copy()
    prepare_min_z, prepare_max_tilt = float(data.qpos[2]), 0.0
    hold_min_margin, hold_max_tilt, hold_max_speed = float("inf"), 0.0, 0.0
    hold_samples, both_feet_samples = 0, 0

    def com_margin():
        # Signed distance of projected whole-robot CoM to the convex hull of
        # current foot/floor contact points. Only a static-contact diagnostic.
        from scipy.spatial import ConvexHull, QhullError
        contacts, touching_feet = [], set()
        for contact in data.contact:
            bodies = (int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2]))
            foot = next((body for body in bodies if body in feet), None)
            if foot is not None and 0 in bodies:
                contacts.append(contact.pos[:2].copy())
                touching_feet.add(foot)
        if len(contacts) < 3:
            return -1.0, len(touching_feet) == 2
        try:
            hull = ConvexHull(np.asarray(contacts))
        except QhullError:
            return -1.0, len(touching_feet) == 2
        com = data.subtree_com[model.body("base_link").id, :2]
        return float(np.min(-(hull.equations[:, :2] @ com + hull.equations[:, 2]))), len(touching_feet) == 2

    ready_since_tick = None
    required_ready_hold = cfg["dance_ready_hold_s"]
    automatic_start = False
    continuous_ready_s = 0.0

    def entry_ready():
        tilt = float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(data.qpos[3:7])[2, 2], -1, 1))))
        return (np.max(np.abs(data.qpos[qadr] - pose_target)) <= conf["max_start_pose_error"]
                and np.linalg.norm(data.qvel[:3]) <= conf["max_start_speed"]
                and tilt <= conf["max_tilt_deg"]
                and np.all(data.qpos[qadr] >= lower - conf["joint_limit_tolerance"])
                and np.all(data.qpos[qadr] <= upper + conf["joint_limit_tolerance"]))

    prepare_budget_ticks = transition_ticks + round(ready_timeout_s * 1000)
    for prepare_tick in range(prepare_budget_ticks + 1):
        tilt = float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(data.qpos[3:7])[2, 2], -1, 1))))
        prepare_max_tilt = max(prepare_max_tilt, tilt)
        if tilt > pose_cfg["max_tilt_deg"]:
            return {**candidate_context,
                    "implementation": "Python numerical probe; Ubuntu C++/ROS/DCU not executed",
                    "workflow": "A zero -> B stand -> Guide -> dance_ready -> automatic dance_x1",
                    "mujoco_version": mujoco.__version__, "stand_hold_s": stand_hold_s,
                    "ready_wait_timeout_s": ready_timeout_s, "yaw_rad": yaw,
                    "completed": False, "phase": "dance preparation", "fault": "tilt",
                    "preparation_elapsed_s": prepare_tick / 1000, "policy_steps": 0,
                    "min_base_z_m": prepare_min_z, "max_tilt_deg": prepare_max_tilt}
        if prepare_tick >= transition_ticks and entry_ready():
            if ready_since_tick is None:
                ready_since_tick = prepare_tick
        else:
            ready_since_tick = None
        continuous_ready_s = ((prepare_tick - ready_since_tick) / 1000
                              if ready_since_tick is not None else 0.0)
        if ready_since_tick is not None and continuous_ready_s >= required_ready_hold:
            automatic_start = True
            break
        if prepare_tick == prepare_budget_ticks:
            break  # Probe time budget only; production continues waiting for readiness.
        if prepare_tick >= transition_ticks and prepare_tick % 20 == 0:
            margin, both_feet = com_margin()
            hold_min_margin = min(hold_min_margin, margin)
            hold_max_tilt = max(hold_max_tilt, tilt)
            hold_max_speed = max(hold_max_speed, float(np.linalg.norm(data.qvel[:3])))
            hold_samples += 1
            both_feet_samples += int(both_feet)
        blend = min((prepare_tick + 1) / transition_ticks, 1.0)
        prepare_target = prepare_start + blend * (pose_target - prepare_start)
        data.ctrl[actuators] = np.clip(pose_kp * (prepare_target - data.qpos[qadr]) - pose_kd * data.qvel[vadr], -effort, effort)
        mujoco.mj_step(model, data)
        prepare_min_z = min(prepare_min_z, float(data.qpos[2]))

    # The original Guide already authorized this handoff. No second press/hold
    # is involved: start at the first qualifying instant, as in the control loop.
    expected_start = pose_target
    pose_error = float(np.max(np.abs(data.qpos[qadr] - expected_start)))
    start_speed = float(np.linalg.norm(data.qvel[:3]))
    hold_context = {"hold_min_com_margin_m": hold_min_margin if hold_samples else None,
                    "hold_max_tilt_deg": hold_max_tilt if hold_samples else None,
                    "hold_max_speed_m_s": hold_max_speed if hold_samples else None,
                    "hold_both_feet_contact_fraction": both_feet_samples / hold_samples if hold_samples else None,
                    "handoff_pose_error_rad": pose_error, "handoff_speed_m_s": start_speed,
                    "required_ready_hold_s": required_ready_hold,
                    "continuous_ready_s": continuous_ready_s,
                    "guide_authorized_preparation": True, "automatic_start": automatic_start,
                    "preparation_elapsed_s": prepare_tick / 1000}
    if not automatic_start:
        return {**candidate_context, **hold_context,
                "implementation": "Python numerical probe; Ubuntu C++/ROS/DCU not executed",
                "workflow": "A zero -> B stand -> Guide -> dance_ready -> automatic dance_x1",
                "mujoco_version": mujoco.__version__, "stand_hold_s": stand_hold_s,
                "ready_wait_timeout_s": ready_timeout_s, "yaw_rad": yaw,
                "completed": False, "phase": "readiness wait", "fault": "probe readiness time budget",
                "max_start_pose_error_rad": pose_error, "start_speed_m_s": start_speed,
                "policy_steps": 0, "min_base_z_m": prepare_min_z, "max_tilt_deg": prepare_max_tilt}

    def quat_yaw(quat):
        rotation = math.quat_to_rotmat(quat)
        return np.arctan2(rotation[1, 0], rotation[0, 0])

    angle = quat_yaw(data.xquat[anchor]) - quat_yaw(ref_quat[0])
    alignment = np.array([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])
    translation = data.xpos[anchor] - math.quat_rotate(alignment, ref_pos[0])
    target = data.qpos[qadr].copy()
    last_action = np.zeros(29, np.float32)
    frame_count, clip_count = 0, 0
    anchor_errors, tilts = [], []
    min_z, max_target_error, max_action, max_limit_violation = float("inf"), 0.0, 0.0, 0.0
    fault = None
    lower_target = lower + conf["joint_limit_margin"]
    upper_target = upper - conf["joint_limit_margin"]
    ticks = len(ref_q) * 20
    for tick in range(ticks):
        q, velocity = data.qpos[qadr].copy(), data.qvel[vadr].copy()
        tilt = float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(data.qpos[3:7])[2, 2], -1, 1))))
        tilts.append(tilt)
        violation = float(np.max(np.maximum(lower - q, q - upper)))
        max_limit_violation = max(max_limit_violation, violation)
        error = float(np.max(np.abs(q - target)))
        max_target_error = max(max_target_error, error)
        if tilt > conf["max_tilt_deg"]:
            fault = "tilt"
        elif violation > conf["joint_limit_tolerance"]:
            fault = "measured joint limit"
        elif conf["max_tracking_error"] > 0 and error > conf["max_tracking_error"]:
            fault = "target tracking error"
        elif tick and np.linalg.norm(translation + math.quat_rotate(alignment, ref_pos[max(0, frame_count - 1)]) - data.xpos[anchor]) > conf["max_anchor_error"]:
            fault = "anchor tracking error"
        if fault:
            break
        if tick % 20 == 0:
            ref_position = translation + math.quat_rotate(alignment, ref_pos[frame_count])
            ref_orientation = math.quat_mul(alignment, ref_quat[frame_count])
            delta, rotation = math.subtract_frame_transform(data.xpos[anchor], data.xquat[anchor], ref_position, ref_orientation)
            raw = np.concatenate((ref_q[frame_count], ref_v[frame_count], delta,
                                  math.quat_to_rotmat(rotation)[:, :2].flatten(),
                                  math.quat_rotate_inv(data.qpos[3:7], data.qvel[:3]), data.qvel[3:6],
                                  q - default, velocity, last_action)).astype(np.float32)
            last_action = actor(raw)
            if not np.isfinite(last_action).all() or np.max(np.abs(last_action)) > 20:
                fault = "policy output"
                break
            max_action = max(max_action, float(np.max(np.abs(last_action))))
            raw_target = default + scale * last_action
            clip_count += int(np.count_nonzero((raw_target < lower_target) | (raw_target > upper_target)))
            target = np.clip(raw_target, lower_target, upper_target)
            frame_count += 1
            anchor_errors.append(float(np.linalg.norm(delta)))
        # Position target is ZOH; the PD torque uses fresh state at 1000 Hz.
        data.ctrl[actuators] = np.clip(kp * (target - q) - kd * velocity, -effort, effort)
        mujoco.mj_step(model, data)
        min_z = min(min_z, float(data.qpos[2]))

    return {**candidate_context, **hold_context,
            "implementation": "Python numerical probe; Ubuntu C++/ROS/DCU not executed",
            "workflow": "A zero -> B stand -> Guide -> dance_ready -> automatic dance_x1", "phase": "playback",
            "mujoco_version": mujoco.__version__, "stand_hold_s": stand_hold_s, "yaw_rad": yaw,
            "ready_wait_timeout_s": ready_timeout_s, "preparation_s": pose_cfg["transition_duration_s"],
            "completed": fault is None and frame_count == len(ref_q), "fault": fault,
            "policy_steps": frame_count, "physics_ticks": tick + (fault is None),
            "min_base_z_m": min_z, "max_tilt_deg": max(tilts),
            "mean_anchor_error_m": float(np.mean(anchor_errors)) if anchor_errors else None,
            "max_anchor_error_m": max(anchor_errors) if anchor_errors else None,
            "max_target_error_rad": max_target_error, "max_action": max_action,
            "max_measured_limit_violation_rad": max_limit_violation, "clipped_target_components": clip_count}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--stand-hold", type=float, default=3.0)
    parser.add_argument("--ready-timeout", type=float, default=10.0,
                        help="probe budget after preparation transition; playback starts automatically when ready")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--candidate", default="configured",
                        choices=("configured", "default_q", "amp_stand", "mid_crouch",
                                 "neutral_elbows", "amp_neutral_elbows", "ref_blend_15", "ref_blend_25", "ref_blend_50"),
                        help="Offline experiment only; alternate entry gate is NOT enabled in C++")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    if (not np.isfinite(args.stand_hold) or args.stand_hold < 0 or
            not np.isfinite(args.ready_timeout) or args.ready_timeout < 0 or not np.isfinite(args.yaw)):
        parser.error("stand hold/ready timeout must be nonnegative and yaw must be finite")
    torch.set_num_threads(1)
    result = run(args.training_root, args.stand_hold, args.yaw, args.ready_timeout, args.candidate)
    output = json.dumps(result, indent=2) + "\n"
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(output)
    print(output, end="")
    if not result["completed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
