#!/usr/bin/env python3
"""Independently compare estimator FK/Jacobians/load cues to MuJoCo."""
import argparse
import hashlib
import json
from pathlib import Path
import mujoco
import numpy as np
from dance_estimator_probe import CxxEstimator


def check(training_root, fixture_out=None):
    path = training_root / "whole_body_tracking/source/whole_body_tracking/whole_body_tracking/assets/xyber_x1/mjcf/xyber_x1_flat.xml"
    model = mujoco.MjModel.from_xml_path(str(path))
    data = mujoco.MjData(model)
    probe = CxxEstimator(model)
    rng = np.random.default_rng(22026)
    for case in range(100):
        mujoco.mj_resetData(model, data)
        data.qpos[:3] = rng.normal(size=3)
        data.qpos[3:7] = rng.normal(size=4)
        data.qpos[3:7] /= np.linalg.norm(data.qpos[3:7])
        data.qpos[probe.qadr] = rng.uniform(-.4, .4, 12)
        data.qvel[:] = rng.normal(size=model.nv)
        mujoco.mj_forward(model, data)
        values = np.concatenate((data.qpos[3:7], data.qvel[3:6], np.zeros(3),
                                 data.qpos[probe.qadr], data.qvel[probe.vadr], rng.normal(size=12)))
        for side, name in enumerate(("left", "right")):
            body = model.body(name+"_ankle_roll_link").id
            geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == body and model.geom_contype[g]
                     and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
            geom = min(geoms, key=lambda g: data.geom_xpos[g, 2]-model.geom_size[g, 0])
            point = data.geom_xpos[geom]
            jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jp, jr, point, body)
            result = np.zeros(33)
            probe.lib.dance_leg_kinematics(side, values, result)
            np.testing.assert_allclose(result[:3], data.xpos[body]-data.qpos[:3], atol=1e-12)
            np.testing.assert_allclose(result[3:6], point-data.qpos[:3], atol=1e-12)
            np.testing.assert_allclose(result[6:9], jp @ data.qvel-data.qvel[:3], atol=1e-11)
            j = jp[:, probe.vadr[side*6:side*6+6]]
            np.testing.assert_allclose(result[13:31].reshape(6, 3).T, j, atol=1e-12)
            # Compute static gravity using independent MuJoCo dynamics, then
            # compare the damped wrench inversion rather than reusing C++ FK.
            velocity = data.qvel.copy()
            data.qvel[:] = 0
            mujoco.mj_forward(model, data)
            grav = data.qfrc_bias[probe.vadr[side*6:side*6+6]].copy()
            data.qvel[:] = velocity
            mujoco.mj_forward(model, data)
            torque = values[34+6*side:40+6*side]
            force = np.linalg.solve(j @ j.T+1e-4*np.eye(3), j @ (grav-torque))
            np.testing.assert_allclose(result[31], force[2], atol=1e-9)
    if fixture_out:
        mujoco.mj_resetData(model, data)
        metadata = json.loads((Path(__file__).resolve().parents[1] / "policy/dance_x1_v11_4999.json").read_text())
        for name, params in metadata["joints"].items():
            data.qpos[model.jnt_qposadr[model.joint(name).id]] = params["default_pos"]
        data.qpos[3:7] = [np.cos(.4), 0, 0, np.sin(.4)]
        mujoco.mj_forward(model, data)
        gravity = data.qfrc_bias[probe.vadr].copy()
        torque = gravity.copy()
        lowest = []
        for side, name in enumerate(("left", "right")):
            body = model.body(name+"_ankle_roll_link").id
            geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == body and model.geom_contype[g]
                     and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE]
            geom = min(geoms, key=lambda g: data.geom_xpos[g, 2]-model.geom_size[g, 0])
            lowest.append(data.geom_xpos[geom, 2]-data.qpos[2]-model.geom_size[geom, 0])
            jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
            mujoco.mj_jac(model, data, jp, jr, data.geom_xpos[geom], body)
            torque[side*6:side*6+6] -= jp[:, probe.vadr[side*6:side*6+6]].T @ [0, 0, 150]
        values = np.r_[data.qpos[3:7], [0, 0, 0], [0, 0, 9.81], data.qpos[probe.qadr], np.zeros(12),
                       torque, gravity, -min(lowest)]
        xml = path.parent / "robot/xyber_x1/xyber_x1_serial.xml"
        np.savetxt(fixture_out, values[None], delimiter=",", fmt="%.17g",
                   header="Independent MuJoCo static fixture: q4, gyro3, accel3, q12, qd12, torque12, gravity_torque12, base_height\n"
                          + "Training XML SHA256: " + hashlib.sha256(xml.read_bytes()).hexdigest())
    probe.close()
    print("PASS: 100 random mixed-axis FK, point Jacobian, velocity and gravity/load comparisons against MuJoCo")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--fixture-out", type=Path)
    args = parser.parse_args()
    check(args.training_root, args.fixture_out)
