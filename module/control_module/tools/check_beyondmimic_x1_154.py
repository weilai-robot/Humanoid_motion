#!/usr/bin/env python3
"""Generate/check v12 golden inputs and commands using MuJoCo + source PyTorch.

Does not read the serialized deployment model or use the C++ inference code.
The fixture is a static default-q pose, identity base IMU, zero gyro/qd/action.
"""
from __future__ import annotations

import argparse
import csv
import importlib.util
import io
from pathlib import Path

import mujoco
import numpy as np
import torch

from export_beyondmimic_x1 import JOINTS, joint_parameters
from sim_beyondmimic_x1 import load_yaml


def fixtures(training_root: Path) -> dict[str, str]:
    control = Path(__file__).resolve().parents[1]
    cfg = load_yaml(control / "cfg/rl_x1_dance.yaml")
    assert cfg["controllers"]["dance_x1"]["observation_layout"] == "wo_state_estimation"
    default, kp, kd, effort, scale, _ = joint_parameters(training_root)
    spec = importlib.util.spec_from_file_location("training_math", training_root / "whole_body_tracking/scripts/play_mujoco.py")
    math = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(math)
    model = mujoco.MjModel.from_xml_path(str(training_root / "whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/xyber_x1/mjcf/xyber_x1_flat.xml"))
    data = mujoco.MjData(model)
    data.qpos[3:7] = [1, 0, 0, 0]
    for name, q in zip(JOINTS, default):
        data.qpos[model.jnt_qposadr[model.joint(name).id]] = q
    mujoco.mj_forward(model, data)
    motion = np.load(training_root / "whole_body_tracking/motions/dance_x1_train.npz", allow_pickle=False)
    order = [list(motion["joint_names"]).index(name) for name in JOINTS]
    anchor = list(motion["body_names"]).index("lumbar_pitch_link")
    ref = motion["body_quat_w"][0, anchor]
    actual = data.xquat[model.body("lumbar_pitch_link").id]
    yaw = lambda q: np.arctan2(math.quat_to_rotmat(q)[1, 0], math.quat_to_rotmat(q)[0, 0])
    delta = yaw(actual)-yaw(ref)
    alignment = np.array([np.cos(delta/2), 0, 0, np.sin(delta/2)])
    relative = math.quat_mul(math.quat_inv(actual), math.quat_mul(alignment, ref))
    relative /= np.linalg.norm(relative)
    raw = np.r_[motion["joint_pos"][0, order], motion["joint_vel"][0, order],
                math.quat_to_rotmat(relative)[:, :2].flatten(), np.zeros(3+29+29+29)].astype(np.float32)
    assert raw.shape == (154,)
    checkpoint = torch.load(training_root / "trained_models/dance_x1_v12_model_4999.pt", map_location="cpu", weights_only=True)
    norm = checkpoint["obs_norm_state_dict"]
    normalized = (torch.from_numpy(raw)-norm["_mean"].reshape(-1))/(norm["_std"].reshape(-1)+.01)
    layers = []
    for i, (n_in, n_out) in enumerate(zip((154, 512, 256, 128), (512, 256, 128, 29))):
        layer = torch.nn.Linear(n_in, n_out)
        layer.load_state_dict({"weight": checkpoint["model_state_dict"][f"actor.{2*i}.weight"],
                               "bias": checkpoint["model_state_dict"][f"actor.{2*i}.bias"]})
        layers.append(layer)
        if i < 3:
            layers.append(torch.nn.ELU())
    with torch.no_grad():
        action = torch.nn.Sequential(*layers)(normalized).numpy()
    limits = cfg["joint_limits"]
    lower = np.array([limits[name]["lower"] for name in JOINTS], dtype=np.float32)
    upper = np.array([limits[name]["upper"] for name in JOINTS], dtype=np.float32)
    target = np.clip(default+scale*action, lower, upper)
    commands, observations = io.StringIO(), io.StringIO()
    writer = csv.writer(commands, lineterminator="\n")
    writer.writerow(["joint", "position", "effort", "kp", "kd"])
    for i, name in enumerate(JOINTS):
        ankle = "_ankle_" in name
        torque = np.clip(kp[i]*(target[i]-default[i]), -effort[i], effort[i])
        writer.writerow([name, float(0 if ankle else target[i]), float(torque if ankle else 0),
                         float(0 if ankle else kp[i]), float(0 if ankle else kd[i])])
    writer = csv.writer(observations, lineterminator="\n")
    writer.writerow(["index", "raw", "normalized"])
    for i in range(154):
        writer.writerow([i, float(raw[i]), float(normalized[i])])
    return {"dance_x1_v12_154_first_frame.csv": commands.getvalue(),
            "dance_x1_v12_154_observation.csv": observations.getvalue()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--write-fixtures", action="store_true")
    args = parser.parse_args()
    torch.set_num_threads(1)
    destination = Path(__file__).resolve().parents[1] / "test/data"
    for name, contents in fixtures(args.training_root).items():
        path = destination / name
        if args.write_fixtures:
            path.write_text(contents)
        else:
            expected = list(csv.reader(io.StringIO(contents)))
            saved = list(csv.reader(io.StringIO(path.read_text())))
            assert len(expected) == len(saved) and expected[0] == saved[0], name
            for actual, reference in zip(saved[1:], expected[1:]):
                assert actual[0] == reference[0]
                np.testing.assert_allclose(np.array(actual[1:], float), np.array(reference[1:], float), atol=2e-6, rtol=2e-6)
        print(f"PASS: independent MuJoCo/PyTorch fixture {name}")


if __name__ == "__main__":
    main()
