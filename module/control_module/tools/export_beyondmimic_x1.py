#!/usr/bin/env python3
"""Export the X1 BeyondMimic checkpoint and reference for DanceController.

This is an offline asset conversion. It does not require Isaac Lab or ONNX.
The actor and empirical normalizer come from the exact RSL-RL checkpoint;
the reference is reordered by joint name into the policy's PhysX order.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
import struct
from pathlib import Path

import numpy as np
import torch


JOINTS = [
    "left_hip_pitch_joint", "lumbar_yaw_joint", "right_hip_pitch_joint",
    "left_hip_roll_joint", "lumbar_roll_joint", "right_hip_roll_joint",
    "left_hip_yaw_joint", "lumbar_pitch_joint", "right_hip_yaw_joint",
    "left_knee_pitch_joint", "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
    "right_knee_pitch_joint", "left_ankle_pitch_joint", "left_shoulder_roll_joint",
    "right_shoulder_roll_joint", "right_ankle_pitch_joint", "left_ankle_roll_joint",
    "left_shoulder_yaw_joint", "right_shoulder_yaw_joint", "right_ankle_roll_joint",
    "left_elbow_pitch_joint", "right_elbow_pitch_joint", "left_elbow_yaw_joint",
    "right_elbow_yaw_joint", "left_wrist_pitch_joint", "right_wrist_pitch_joint",
    "left_wrist_roll_joint", "right_wrist_roll_joint",
]

MODEL_MAGIC = b"X1DNCE2\0"
MOTION_MAGIC = b"X1MOTN1\0"
LAYERS = (160, 512, 256, 128, 29)
ROBOT_CONFIG = Path("whole_body_tracking/source/whole_body_tracking/whole_body_tracking/robots/x1.py")
NORMALIZER_EPS = np.float32(1e-2)  # RSL-RL EmpiricalNormalization default


def joint_parameters(training_root: Path) -> tuple[np.ndarray, ...]:
    # Read literal config values without importing Isaac Sim or executing repo code.
    tree = ast.parse((training_root / ROBOT_CONFIG).read_text())
    robot = next(node.value for node in tree.body if isinstance(node, ast.Assign)
                 and any(isinstance(t, ast.Name) and t.id == "X1_CYLINDER_CFG" for t in node.targets))
    keywords = {kw.arg: kw.value for kw in robot.keywords}
    initial = {kw.arg: kw.value for kw in keywords["init_state"].keywords}
    defaults = ast.literal_eval(initial["joint_pos"])
    actuators = [{kw.arg: ast.literal_eval(kw.value) for kw in call.keywords}
                 for call in keywords["actuators"].values]

    def resolve(value, name):
        if not isinstance(value, dict):
            return float(value)
        matches = [v for pattern, v in value.items() if re.fullmatch(pattern, name)]
        if len(matches) != 1:
            raise ValueError(f"ambiguous/missing actuator parameter for {name}")
        return float(matches[0])

    default, kp, kd, effort, action_scale, armature = (np.zeros(len(JOINTS), np.float32) for _ in range(6))
    for i, name in enumerate(JOINTS):
        matches = [a for a in actuators if any(re.fullmatch(p, name) for p in a["joint_names_expr"])]
        if len(matches) != 1:
            raise ValueError(f"joint classification failed for {name}: {matches}")
        actuator = matches[0]
        default[i] = resolve(defaults, name) if any(re.fullmatch(p, name) for p in defaults) else 0.0
        kp[i] = resolve(actuator["stiffness"], name)
        kd[i] = resolve(actuator["damping"], name)
        effort[i] = resolve(actuator["effort_limit_sim"], name)
        armature[i] = resolve(actuator["armature"], name)
        if kp[i] <= 0 or not 0 < kd[i] <= 8 or effort[i] <= 0 or armature[i] < 0:
            raise ValueError(f"invalid PD/effort/armature for {name}")
        action_scale[i] = 0.25 * effort[i] / kp[i]
    return default, kp, kd, effort, action_scale, armature


def write_names(handle) -> None:
    for name in JOINTS:
        encoded = name.encode("ascii")
        if len(encoded) > 31:
            raise ValueError(name)
        handle.write(encoded.ljust(32, b"\0"))


def write_float_array(handle, array) -> None:
    values = np.asarray(array, dtype="<f4")
    if not np.isfinite(values).all():
        raise ValueError("asset contains nonfinite floats")
    handle.write(values.tobytes(order="C"))


def export_model(checkpoint: Path, output: Path, training_root: Path) -> dict:
    ckpt = torch.load(checkpoint, map_location="cpu", weights_only=True)
    weights = ckpt.get("model_state_dict", ckpt.get("model_dict"))
    norm = ckpt.get("obs_norm_state_dict", ckpt.get("obs_normalizer"))
    if weights is None or norm is None:
        raise ValueError("checkpoint has no actor or empirical normalizer")
    mean = np.asarray(norm.get("_mean", norm.get("mean")), dtype=np.float32).reshape(-1)
    std = np.asarray(norm.get("_std"), dtype=np.float32).reshape(-1)
    if mean.shape != (160,) or std.shape != (160,) or np.any(std < 0):
        raise ValueError("unexpected normalizer shape or standard deviation")
    inv_std = 1.0 / (std + NORMALIZER_EPS)
    params = joint_parameters(training_root)

    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as handle:
        handle.write(MODEL_MAGIC)
        handle.write(struct.pack("<7I", 2, len(JOINTS), *LAYERS))
        write_names(handle)
        for array in (*params, mean, inv_std):
            write_float_array(handle, array)
        for i, (in_dim, out_dim) in enumerate(zip(LAYERS, LAYERS[1:])):
            key = i * 2
            w = weights[f"actor.{key}.weight"].detach().cpu().numpy()
            b = weights[f"actor.{key}.bias"].detach().cpu().numpy()
            if w.shape != (out_dim, in_dim) or b.shape != (out_dim,):
                raise ValueError(f"unexpected actor layer {i}: {w.shape}, {b.shape}")
            write_float_array(handle, w)
            write_float_array(handle, b)

    # Read the serialized bytes back, then compare deployed arithmetic against RSL-RL.
    blob = output.read_bytes()
    if blob[:8] != MODEL_MAGIC or struct.unpack_from("<7I", blob, 8) != (2, 29, *LAYERS):
        raise ValueError("serialized model header mismatch")
    offset = 8 + 7 * 4
    for name in JOINTS:
        if blob[offset:offset + 32].split(b"\0", 1)[0].decode("ascii") != name:
            raise ValueError("serialized joint order mismatch")
        offset += 32

    def read_values(count: int) -> np.ndarray:
        nonlocal offset
        values = np.frombuffer(blob, dtype="<f4", count=count, offset=offset).copy()
        offset += count * 4
        return values

    for original in params:
        if not np.array_equal(read_values(29), original):
            raise ValueError("serialized joint parameters mismatch")
    deployed_mean, deployed_inv_std = read_values(160), read_values(160)
    deployed_layers = []
    for in_dim, out_dim in zip(LAYERS, LAYERS[1:]):
        deployed_layers.append((read_values(in_dim * out_dim).reshape(out_dim, in_dim), read_values(out_dim)))
    if offset != len(blob):
        raise ValueError("serialized model length mismatch")

    rng = np.random.default_rng(20260926)
    max_error = 0.0
    for raw in (rng.normal(size=(128, 160)).astype(np.float32), np.zeros((1, 160), np.float32)):
        value = ((raw - deployed_mean) * deployed_inv_std).astype(np.float32)
        expected = value.copy()
        for i, (w, b) in enumerate(deployed_layers):
            expected = expected @ w.T + b
            if i != 3:
                negative = expected < 0
                expected[negative] = np.expm1(expected[negative])
        modules = []
        for i in range(4):
            key = i * 2
            modules.append(torch.nn.Linear(LAYERS[i], LAYERS[i + 1]))
            modules[-1].weight.data = weights[f"actor.{key}.weight"].detach().clone()
            modules[-1].bias.data = weights[f"actor.{key}.bias"].detach().clone()
            if i != 3:
                modules.append(torch.nn.ELU())
        with torch.no_grad():
            # Independent normalization, using saved RSL-RL std + eps.
            normalized = (torch.from_numpy(raw) - torch.from_numpy(mean)) / (torch.from_numpy(std) + float(NORMALIZER_EPS))
            torch_out = torch.nn.Sequential(*modules)(normalized).numpy()
        max_error = max(max_error, float(np.max(np.abs(expected - torch_out))))
        if not np.allclose(expected, torch_out, atol=1e-4, rtol=2e-5):
            raise ValueError("exported arithmetic differs from actor")
    return {"normalizer": "(obs - _mean) / (_std + 0.01)",
            "actor_max_abs_error": max_error,
            "joints": {name: dict(zip(("default_pos", "kp", "kd", "effort_limit", "action_scale", "armature"),
                                      (float(p[i]) for p in params))) for i, name in enumerate(JOINTS)}}


def export_motion(motion: Path, output: Path) -> None:
    data = np.load(motion, allow_pickle=False)
    fps = float(data["fps"])
    names = [str(name) for name in data["joint_names"]]
    body_names = [str(name) for name in data["body_names"]]
    if fps != 50.0 or len(names) != len(JOINTS) or set(names) != set(JOINTS):
        raise ValueError("motion fps or joint names mismatch")
    if "lumbar_pitch_link" not in body_names:
        raise ValueError("motion has no lumbar_pitch_link anchor")
    order = [names.index(name) for name in JOINTS]
    anchor_index = body_names.index("lumbar_pitch_link")
    pos = np.asarray(data["joint_pos"][:, order], np.float32)
    vel = np.asarray(data["joint_vel"][:, order], np.float32)
    anchor_pos = np.asarray(data["body_pos_w"][:, anchor_index], np.float32)
    anchor_quat = np.asarray(data["body_quat_w"][:, anchor_index], np.float32)
    if pos.shape != vel.shape or pos.shape[1] != 29 or anchor_pos.shape != (len(pos), 3):
        raise ValueError("motion tensor shapes mismatch")
    if anchor_quat.shape != (len(pos), 4) or np.max(np.abs(np.linalg.norm(anchor_quat, axis=1) - 1)) > 1e-3:
        raise ValueError("motion quaternions are invalid")
    frames = np.concatenate((pos, vel, anchor_pos, anchor_quat), axis=1)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("wb") as handle:
        handle.write(MOTION_MAGIC)
        handle.write(struct.pack("<4I", 1, len(pos), len(JOINTS), int(fps)))
        write_names(handle)
        write_float_array(handle, frames)
    blob = output.read_bytes()
    header_size = 8 + 4 * 4 + len(JOINTS) * 32
    if blob[:8] != MOTION_MAGIC or struct.unpack_from("<4I", blob, 8) != (1, len(pos), 29, 50):
        raise ValueError("serialized motion header mismatch")
    restored = np.frombuffer(blob, dtype="<f4", offset=header_size).reshape(frames.shape)
    if not np.array_equal(restored, frames):
        raise ValueError("serialized motion frames mismatch")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--motion", type=Path, required=True)
    parser.add_argument("--model-out", type=Path, required=True)
    parser.add_argument("--motion-out", type=Path, required=True)
    args = parser.parse_args()
    metadata = export_model(args.checkpoint, args.model_out, args.training_root)
    export_motion(args.motion, args.motion_out)
    metadata["format_version"] = 2
    metadata["policy_hz"] = 50
    metadata["sources"] = {label: {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
                           for label, path in (("checkpoint", args.checkpoint), ("motion", args.motion),
                                               ("robot_config", args.training_root / ROBOT_CONFIG))}
    metadata["assets"] = {path.name: hashlib.sha256(path.read_bytes()).hexdigest()
                          for path in (args.model_out, args.motion_out)}
    args.model_out.with_suffix(".json").write_text(json.dumps(metadata, indent=2) + "\n")
    for path in (args.model_out, args.motion_out):
        print(f"{path}: {path.stat().st_size} bytes sha256={hashlib.sha256(path.read_bytes()).hexdigest()}")


if __name__ == "__main__":
    main()
