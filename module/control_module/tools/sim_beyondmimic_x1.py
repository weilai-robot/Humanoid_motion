#!/usr/bin/env python3
"""Offline numerical probe of the dance assets and 1 kHz / 50 Hz ZOH contract.

Requires numpy, torch, mujoco and PyYAML (or ruamel.yaml). This is a Python
simulation, not a build/test of the Ubuntu ROS 2 executable or DCU hardware.
It uses the training MJCF, identified armature, effort saturation and the
dedicated deployment YAML. No ROS messages, motors or viewers are opened.
"""

from __future__ import annotations

import argparse
import ctypes
import hashlib
import importlib.util
import json
import struct
import subprocess
import tempfile
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
        candidate: str = "configured", state_feedback=None, estimator_parameters=None,
        estimator_noise_seed=None, config_path=None) -> dict:
    control = Path(__file__).resolve().parents[1]
    cfg = load_yaml(config_path or control / "cfg/rl_x1_dance.yaml")
    conf = cfg["controllers"]["dance_x1"]
    state_feedback = state_feedback or ("imu" if conf["state_source"] == "imu" else "truth")
    model_path = control / "policy" / Path(conf["model_file"]).name
    assert cfg["control_frequecy"] == 1000 and conf["decimation"] == 20
    assert conf["prepare_ticks"] == 0 and not conf["joint_sign"]
    metadata = json.loads(model_path.with_suffix(".json").read_text())
    for name, digest in metadata["assets"].items():
        assert hashlib.sha256((control / "policy" / name).read_bytes()).hexdigest() == digest

    # Use the independent training playback's quaternion/observation utilities.
    playback = training_root / "whole_body_tracking/scripts/play_mujoco.py"
    spec = importlib.util.spec_from_file_location("x1_reference_math", playback)
    math = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(math)
    names = conf["joint_list"]
    assert names == list(metadata["joints"])
    blob = model_path.read_bytes()
    obs_dim = struct.unpack_from("<7I", blob, 8)[2]
    without_translation = conf.get("observation_layout", "full") == "wo_state_estimation"
    assert obs_dim == (154 if without_translation else 160)
    assert (state_feedback == "imu") == without_translation
    assert blob[:8] == b"X1DNCE2\0" and struct.unpack_from("<7I", blob, 8) == (2, 29, obs_dim, 512, 256, 128, 29)
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
    mean, inv_std = read(obs_dim), read(obs_dim)
    layers = []
    for n_in, n_out in zip((obs_dim, 512, 256, 128), (512, 256, 128, 29)):
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

    motion_blob = (control / "policy" / Path(conf["motion_file"]).name).read_bytes()
    version, frame_count, joint_count, fps = struct.unpack_from("<4I", motion_blob, 8)
    assert motion_blob[:8] == b"X1MOTN1\0" and (version, joint_count, fps) == (1, 29, 50)
    motion_offset = 24
    for name in names:
        assert motion_blob[motion_offset:motion_offset+32].split(b"\0", 1)[0].decode() == name
        motion_offset += 32
    frames = np.frombuffer(motion_blob, dtype="<f4", offset=motion_offset).reshape(frame_count, 65)
    assert np.isfinite(frames).all()
    ref_q, ref_v, ref_pos, ref_quat = frames[:, :29], frames[:, 29:58], frames[:, 58:61], frames[:, 61:65]
    lower = np.array([cfg["joint_limits"][name]["lower"] for name in names])
    upper = np.array([cfg["joint_limits"][name]["upper"] for name in names])
    assert np.all(ref_q >= lower - 1e-4) and np.all(ref_q <= upper + 1e-4)

    mjcf_path = training_root / "whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/xyber_x1/mjcf/xyber_x1_flat.xml"
    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    model.opt.timestep = 0.001
    data = mujoco.MjData(model)
    from dance_estimator_probe import CxxEstimator
    estimator = CxxEstimator(model, estimator_parameters, conf.get("state_estimator"), estimator_noise_seed) if state_feedback in ("shadow", "proprioceptive") else None
    fk_temp = tempfile.TemporaryDirectory(prefix="x1_dance_fk_") if state_feedback == "imu" else None
    fk = None
    if fk_temp:
        library = Path(fk_temp.name) / "waist_fk.so"
        subprocess.run(["clang++", "-std=c++17", "-O2", "-shared", "-fPIC", "-I", str(control / "include"),
                        str(control / "test/dance_state_adapter_capi.cc"), "-o", str(library)], check=True)
        fk_library = ctypes.CDLL(str(library))
        fk = fk_library.dance_waist_fk
        pointer = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags="C_CONTIGUOUS")
        fk.argtypes = [pointer, pointer]
        fk.restype = ctypes.c_int
    max_fk_orientation_error = 0.0

    def finish(result):
        result.update(state_feedback=state_feedback, observation_dim=obs_dim,
                      observation_layout=conf.get("observation_layout", "full"),
                      model_sha256=hashlib.sha256(blob).hexdigest(),
                      max_fk_orientation_error=max_fk_orientation_error,
                      estimator=estimator.metrics() if estimator else None)
        if estimator:
            estimator.close()
        if fk_temp:
            fk_temp.cleanup()
        return result

    sensor_time = None
    def estimate():
        nonlocal sensor_time
        # mj_step leaves kinematic/sensor fields at its pre-integration state.
        # Sample all feedback at one source time, including the truth baseline.
        if sensor_time != data.time:
            mujoco.mj_forward(model, data)
            sensor_time = float(data.time)
        return estimator.step(data) if estimator else None

    def feedback():
        nonlocal max_fk_orientation_error
        estimated = estimate()
        if state_feedback == "imu":
            waist = [data.qpos[model.jnt_qposadr[model.joint(name).id]] for name in
                     ("lumbar_yaw_joint", "lumbar_roll_joint", "lumbar_pitch_joint")]
            inputs = np.r_[data.qpos[3:7], waist].astype(np.float64)
            output = np.empty(4, dtype=np.float64)
            assert fk(inputs, output) == 1
            error = min(np.linalg.norm(output-data.xquat[anchor]), np.linalg.norm(output+data.xquat[anchor]))
            max_fk_orientation_error = max(max_fk_orientation_error, float(error))
            assert error < 1e-6
            return np.zeros(3), output, np.zeros(3)
        if state_feedback == "proprioceptive":
            if estimated is None:
                return None
            waist = [data.qpos[model.jnt_qposadr[model.joint(name).id]] for name in
                     ("lumbar_yaw_joint", "lumbar_roll_joint", "lumbar_pitch_joint")]
            # Evaluate the same training waist chain on estimated base state.
            # No simulated world pose enters the policy in this mode.
            yaw_origin = np.array([1., 0, 0, -2.274557e-05]); yaw_origin /= np.linalg.norm(yaw_origin)
            qyaw = math.quat_mul(math.quat_mul(estimated[6:10], yaw_origin),
                                 np.array([np.cos(waist[0]/2), 0, 0, np.sin(waist[0]/2)]))
            p = estimated[:3]+math.quat_rotate(estimated[6:10], np.array([.00245, 0, .11553403]))
            p += math.quat_rotate(qyaw, np.array([0, 0, .0405]))
            qroll = math.quat_mul(math.quat_mul(qyaw, np.array([.5, .5, -.5, -.5])),
                                  np.array([np.cos(waist[1]/2), 0, 0, np.sin(waist[1]/2)]))
            qanchor = math.quat_mul(math.quat_mul(qroll, np.array([np.sqrt(.5), 0, np.sqrt(.5), 0])),
                                    np.array([np.cos(waist[2]/2), 0, 0, -np.sin(waist[2]/2)]))
            return p, qanchor, estimated[3:6]
        return data.xpos[anchor].copy(), data.xquat[anchor].copy(), math.quat_rotate_inv(data.qpos[3:7], data.qvel[:3])
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
        estimate()
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
        measured = feedback()
        speed_ready = (np.max(np.abs(data.qvel[vadr])) <= conf["max_start_joint_speed"]
                       and np.linalg.norm(data.qvel[3:6]) <= conf["max_start_angular_speed"]
                       if without_translation else measured is not None and np.linalg.norm(measured[2]) <= conf["max_start_speed"])
        return (np.max(np.abs(data.qpos[qadr] - pose_target)) <= conf["max_start_pose_error"]
                and measured is not None and speed_ready
                and tilt <= conf["max_tilt_deg"]
                and np.all(data.qpos[qadr] >= lower - conf["joint_limit_tolerance"])
                and np.all(data.qpos[qadr] <= upper + conf["joint_limit_tolerance"]))

    prepare_budget_ticks = transition_ticks + round(ready_timeout_s * 1000)
    for prepare_tick in range(prepare_budget_ticks + 1):
        estimate()
        tilt = float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(data.qpos[3:7])[2, 2], -1, 1))))
        prepare_max_tilt = max(prepare_max_tilt, tilt)
        if tilt > pose_cfg["max_tilt_deg"]:
            return finish({**candidate_context,
                    "implementation": "Python numerical probe; Ubuntu C++/ROS/DCU not executed",
                    "workflow": "A zero -> B stand -> Guide -> dance_ready -> automatic dance_x1",
                    "mujoco_version": mujoco.__version__, "stand_hold_s": stand_hold_s,
                    "ready_wait_timeout_s": ready_timeout_s, "yaw_rad": yaw,
                    "completed": False, "phase": "dance preparation", "fault": "tilt",
                    "preparation_elapsed_s": prepare_tick / 1000, "policy_steps": 0,
                    "min_base_z_m": prepare_min_z, "max_tilt_deg": prepare_max_tilt})
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
        return finish({**candidate_context, **hold_context,
                "implementation": "Python numerical probe; Ubuntu C++/ROS/DCU not executed",
                "workflow": "A zero -> B stand -> Guide -> dance_ready -> automatic dance_x1",
                "mujoco_version": mujoco.__version__, "stand_hold_s": stand_hold_s,
                "ready_wait_timeout_s": ready_timeout_s, "yaw_rad": yaw,
                "completed": False, "phase": "readiness wait", "fault": "probe readiness time budget",
                "max_start_pose_error_rad": pose_error, "start_speed_m_s": start_speed,
                "policy_steps": 0, "min_base_z_m": prepare_min_z, "max_tilt_deg": prepare_max_tilt})

    def quat_yaw(quat):
        rotation = math.quat_to_rotmat(quat)
        return np.arctan2(rotation[1, 0], rotation[0, 0])

    measured = feedback()
    if estimator:
        estimator.playback_start = len(estimator.errors)
    angle = quat_yaw(measured[1]) - quat_yaw(ref_quat[0])
    alignment = np.array([np.cos(angle / 2), 0, 0, np.sin(angle / 2)])
    translation = measured[0] - math.quat_rotate(alignment, ref_pos[0])
    truth_translation = data.xpos[anchor] - math.quat_rotate(alignment, ref_pos[0])
    target = data.qpos[qadr].copy()
    last_action = np.zeros(29, np.float32)
    frame_count, clip_count = 0, 0
    anchor_errors, truth_anchor_errors, tilts = [], [], []
    min_z, max_target_error, max_action, max_limit_violation = float("inf"), 0.0, 0.0, 0.0
    fault = None
    lower_target = lower + conf["joint_limit_margin"]
    upper_target = upper - conf["joint_limit_margin"]
    ticks = len(ref_q) * 20
    for tick in range(ticks):
        measured = feedback()
        if measured is None:
            fault = "body state estimate unavailable: " + estimator.problem()
            break
        anchor_position, anchor_orientation, base_velocity = measured
        q, velocity = data.qpos[qadr].copy(), data.qvel[vadr].copy()
        tilt = float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(data.qpos[3:7])[2, 2], -1, 1))))
        tilts.append(tilt)
        violation = float(np.max(np.maximum(lower - q, q - upper)))
        # Record measured excursions/PD error without aborting playback, as in C++.
        max_limit_violation = max(max_limit_violation, violation)
        error = float(np.max(np.abs(q - target)))
        max_target_error = max(max_target_error, error)
        if tilt > conf["max_tilt_deg"]:
            fault = "tilt"
        if fault:
            break
        if tick % 20 == 0:
            ref_position = translation + math.quat_rotate(alignment, ref_pos[frame_count])
            ref_orientation = math.quat_mul(alignment, ref_quat[frame_count])
            delta, rotation = math.subtract_frame_transform(anchor_position, anchor_orientation, ref_position, ref_orientation)
            terms = [ref_q[frame_count], ref_v[frame_count]]
            if not without_translation:
                terms.append(delta)
            terms.append(math.quat_to_rotmat(rotation)[:, :2].flatten())
            if not without_translation:
                terms.append(base_velocity)
            terms.extend((data.qvel[3:6], q-default, velocity, last_action))
            raw = np.concatenate(terms).astype(np.float32)
            assert raw.shape == (obs_dim,)
            last_action = actor(raw)
            if not np.isfinite(last_action).all() or np.max(np.abs(last_action)) > 20:
                fault = "policy output"
                break
            max_action = max(max_action, float(np.max(np.abs(last_action))))
            raw_target = default + scale * last_action
            clip_count += int(np.count_nonzero((raw_target < lower_target) | (raw_target > upper_target)))
            target = np.clip(raw_target, lower_target, upper_target)
            frame_count += 1
            if not without_translation:
                anchor_errors.append(float(np.linalg.norm(delta)))
            truth_anchor_errors.append(float(np.linalg.norm(truth_translation + math.quat_rotate(alignment, ref_pos[frame_count-1])-data.xpos[anchor])))
        # Position target is ZOH; the PD torque uses fresh state at 1000 Hz.
        data.ctrl[actuators] = np.clip(kp * (target - q) - kd * velocity, -effort, effort)
        mujoco.mj_step(model, data)
        min_z = min(min_z, float(data.qpos[2]))

    result = {**candidate_context, **hold_context,
            "implementation": "Python numerical probe; Ubuntu C++/ROS/DCU not executed",
            "workflow": "A zero -> B stand -> Guide -> dance_ready -> automatic dance_x1", "phase": "playback",
            "mujoco_version": mujoco.__version__, "stand_hold_s": stand_hold_s, "yaw_rad": yaw,
            "ready_wait_timeout_s": ready_timeout_s, "preparation_s": pose_cfg["transition_duration_s"],
            "completed": fault is None and frame_count == len(ref_q), "fault": fault,
            "policy_steps": frame_count, "physics_ticks": tick + (fault is None),
            "min_base_z_m": min_z, "max_tilt_deg": max(tilts) if tilts else None,
            "mean_anchor_error_m": float(np.mean(anchor_errors)) if anchor_errors else None,
            "mean_true_anchor_error_m": float(np.mean(truth_anchor_errors)) if truth_anchor_errors else None,
            "max_true_anchor_error_m": max(truth_anchor_errors) if truth_anchor_errors else None,
            "max_anchor_error_m": max(anchor_errors) if anchor_errors else None,
            "max_target_error_rad": max_target_error, "max_action": max_action,
            "max_measured_limit_violation_rad": max_limit_violation, "clipped_target_components": clip_count}
    return finish(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--stand-hold", type=float, default=3.0)
    parser.add_argument("--ready-timeout", type=float, default=10.0,
                        help="probe budget after preparation transition; playback starts automatically when ready")
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--config", type=Path, help="deployment YAML; defaults to rl_x1_dance.yaml")
    parser.add_argument("--state-feedback", choices=("imu", "truth", "shadow", "proprioceptive"),
                        help="defaults from YAML; imu uses production C++ waist FK without translation feedback")
    parser.add_argument("--estimator-parameters", type=float, nargs=4, metavar=("ACC_NOISE", "VEL_NOISE", "POS_NOISE", "CONTACT_GATE"),
                        help="offline candidate only; output records parameters, production YAML is not changed")
    parser.add_argument("--estimator-noise-seed", type=int, help="perturb estimator inputs with explicit synthetic noise; not a hardware noise model")
    parser.add_argument("--candidate", default="configured",
                        choices=("configured", "default_q", "amp_stand", "mid_crouch",
                                 "neutral_elbows", "amp_neutral_elbows", "ref_blend_15", "ref_blend_25", "ref_blend_50"),
                        help="Offline experiment only; alternate entry gate is NOT enabled in C++")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args()
    if (not np.isfinite(args.stand_hold) or args.stand_hold < 0 or
            not np.isfinite(args.ready_timeout) or args.ready_timeout < 0 or not np.isfinite(args.yaw)):
        parser.error("stand hold/ready timeout must be nonnegative and yaw must be finite")
    if args.estimator_parameters is not None and (not np.isfinite(args.estimator_parameters).all() or min(args.estimator_parameters) <= 0):
        parser.error("estimator candidate parameters must be positive and finite")
    torch.set_num_threads(1)
    result = run(args.training_root, args.stand_hold, args.yaw, args.ready_timeout, args.candidate, args.state_feedback,
                 args.estimator_parameters, args.estimator_noise_seed, args.config)
    output = json.dumps(result, indent=2) + "\n"
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(output)
    print(output, end="")
    if not result["completed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
