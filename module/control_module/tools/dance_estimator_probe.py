"""MuJoCo sensor adapter for testing the production C++ body estimator.

Only IMU sensors and leg encoder/actuator feedback enter the estimator.
MuJoCo floating-base pose/velocity and contacts are used solely for scoring.
"""
import ctypes
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile

import numpy as np


class CxxEstimator:
    def __init__(self, model, parameters=None, config=None, noise_seed=None):
        self.model = model
        self.tmp = tempfile.TemporaryDirectory(prefix="x1_body_estimator_")
        control = Path(__file__).resolve().parents[1]
        lib = Path(self.tmp.name) / "estimator.so"
        command = [os.environ.get("CXX", "c++"), "-std=c++17", "-O2", "-shared", "-fPIC",
                   "-I", str(control / "include"), str(control / "test/dance_state_estimator_capi.cc"), "-o", str(lib)]
        subprocess.run(command, check=True, capture_output=True, text=True)
        self.lib = ctypes.CDLL(str(lib))
        ptr = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        self.lib.dance_estimator_create.restype = ctypes.c_void_p
        self.lib.dance_estimator_create.argtypes = [ptr]
        self.lib.dance_estimator_destroy.argtypes = [ctypes.c_void_p]
        self.lib.dance_estimator_problem.argtypes = [ctypes.c_void_p]
        self.lib.dance_estimator_problem.restype = ctypes.c_char_p
        self.lib.dance_estimator_update.argtypes = [ctypes.c_void_p, ctypes.c_int64, ptr, ptr]
        self.lib.dance_leg_kinematics.argtypes = [ctypes.c_int, ptr, ptr]
        self.config = dict(acceleration_scale=1., initialization_s=.5, contact_on_force=35., contact_off_force=15.,
                           acceleration_noise=.1, bias_noise=.01, position_noise=.03, velocity_noise=.3,
                           height_noise=.01, max_gap_s=.01, max_flight_s=.35, contact_velocity_limit=.25,
                           imu_position_in_base=[0., 0., 0.])
        self.config.update(config or {})
        if parameters is not None:
            self.config.update(zip(("acceleration_noise", "velocity_noise", "position_noise", "contact_velocity_limit"), parameters))
        keys = ("acceleration_scale", "initialization_s", "contact_on_force", "contact_off_force", "acceleration_noise",
                "bias_noise", "position_noise", "velocity_noise", "height_noise", "max_gap_s", "max_flight_s", "contact_velocity_limit")
        self.parameters = np.r_[[self.config[k] for k in keys], self.config["imu_position_in_base"]].astype(np.float64)
        if self.parameters.shape != (15,) or not np.isfinite(self.parameters).all() or min(self.parameters[:12]) <= 0:
            raise ValueError("invalid estimator configuration")
        self.handle = self.lib.dance_estimator_create(self.parameters)
        self.noise_seed = noise_seed
        self.rng = np.random.default_rng(noise_seed) if noise_seed is not None else None
        names = [side + "_" + suffix + "_joint" for side in ("left", "right")
                 for suffix in ("hip_pitch", "hip_roll", "hip_yaw", "knee_pitch", "ankle_pitch", "ankle_roll")]
        ids = np.array([model.joint(name).id for name in names])
        self.qadr, self.vadr = model.jnt_qposadr[ids], model.jnt_dofadr[ids]
        self.sensors = [model.sensor(name).adr.item() for name in
                        ("body-orientation", "body-angular-velocity", "body-linear-acceleration")]
        self.output = None
        self.time = None
        self.values = np.zeros(46, dtype=np.float64)
        self.errors = []
        self.origin = None
        self.rejected = 0
        self.first_ready = None
        self.trace = []
        self.playback_start = 0
        self.source_hash = hashlib.sha256((control / "include/control_module/dance_state_estimator.h").read_bytes()).hexdigest()

    def step(self, data):
        if self.time == data.time:
            return self.output
        self.time = float(data.time)
        quat, gyro, accel = self.sensors
        self.values[:] = np.concatenate((data.sensordata[quat:quat+4], data.sensordata[gyro:gyro+3],
                                         data.sensordata[accel:accel+3], data.qpos[self.qadr],
                                         data.qvel[self.vadr], data.qfrc_actuator[self.vadr]))
        if self.rng is not None:
            # Estimator-input robustness probe, not a calibrated hardware noise model.
            self.values[4:7] += self.rng.normal(0, .002, 3)
            self.values[7:10] += self.rng.normal(0, .05, 3)+[.04, -.03, .08]
            self.values[10:22] += self.rng.normal(0, .001, 12)
            self.values[22:34] += self.rng.normal(0, .01, 12)
            self.values[34:46] += self.rng.normal(0, .5, 12)
        result = np.zeros(20, dtype=np.float64)
        accepted = self.lib.dance_estimator_update(self.handle, round(data.time*1e9)+1, self.values, result)
        self.output = result if accepted else None
        if accepted:
            if self.origin is None:
                self.origin = result[:3]-data.qpos[:3]
                self.origin[2] = 0  # only the horizontal odometry origin is unobservable
                self.first_ready = data.time
            r = np.empty(9)
            import mujoco
            mujoco.mju_quat2Mat(r, data.qpos[3:7])
            true_velocity = r.reshape(3, 3).T @ data.qvel[:3]
            self.errors.append([* (result[:3]-data.qpos[:3]-self.origin),
                                * (result[3:6]-true_velocity)])
            if round(data.time*1000) % 20 == 0:
                true_contact = [False, False]
                for contact in data.contact:
                    bodies = {self.model.geom_bodyid[contact.geom1], self.model.geom_bodyid[contact.geom2]}
                    for side, name in enumerate(("left", "right")):
                        if 0 in bodies and self.model.body(name+"_ankle_roll_link").id in bodies:
                            true_contact[side] = True
                self.trace.append([data.time, *self.errors[-1], *result[16:20], *true_contact])
        elif self.first_ready is not None:
            self.rejected += 1
        return self.output

    def problem(self):
        return self.lib.dance_estimator_problem(self.handle).decode()

    def metrics(self):
        errors = np.asarray(self.errors)
        playback = errors[self.playback_start:]
        return {"source": "production C++ IMU/leg estimator; no base truth or contact sensor input",
                "config": self.config, "estimator_input_noise_seed": self.noise_seed,
                "source_sha256": self.source_hash, "first_ready_s": self.first_ready,
                "accepted_samples": len(errors), "rejected_after_ready": self.rejected,
                "last_problem": self.problem(),
                "position_rmse_m": float(np.sqrt(np.mean(np.sum(errors[:, :3]**2, axis=1)))) if len(errors) else None,
                "velocity_rmse_m_s": float(np.sqrt(np.mean(np.sum(errors[:, 3:]**2, axis=1)))) if len(errors) else None,
                "max_position_error_m": float(np.max(np.linalg.norm(errors[:, :3], axis=1))) if len(errors) else None,
                "max_velocity_error_m_s": float(np.max(np.linalg.norm(errors[:, 3:], axis=1))) if len(errors) else None,
                "playback_position_rmse_m": float(np.sqrt(np.mean(np.sum(playback[:, :3]**2, axis=1)))) if len(playback) else None,
                "playback_velocity_rmse_m_s": float(np.sqrt(np.mean(np.sum(playback[:, 3:]**2, axis=1)))) if len(playback) else None}

    def close(self):
        trace_path = os.environ.get("X1_ESTIMATOR_TRACE")
        if trace_path:
            np.savetxt(trace_path, self.trace, delimiter=",",
                       header="time,ex,ey,ez,evx,evy,evz,force_l,force_r,contact_l,contact_r,true_contact_l,true_contact_r")
        self.lib.dance_estimator_destroy(self.handle)
        self.tmp.cleanup()
