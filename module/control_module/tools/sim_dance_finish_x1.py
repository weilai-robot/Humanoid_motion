#!/usr/bin/env python3
"""Offline stopping validation. Requires the original dance probe dependencies,
ONNX Runtime and a C++ compiler. No ROS, DCU, physical robot or viewer is used.
True world velocity and contacts score the result; only IMU, encoders and
actuator torque enter the native C++ finishing controller.
"""
from pathlib import Path
import argparse, ctypes, hashlib, json, sys, tempfile, subprocess
import numpy as np, torch, mujoco
control = Path(__file__).resolve().parents[1]
from sim_beyondmimic_x1 import run, load_yaml
PARAMETER_KEYS = ('blend_s', 'upper_delay_s', 'upper_blend_s', 'landing_min_s', 'landing_hold_s', 'stand_blend_s', 'verify_s', 'max_tilt_deg', 'quiet_tilt_deg', 'quiet_gyro', 'quiet_joint_speed', 'quiet_base_speed', 'contact_on', 'contact_off', 'max_foot_height_delta', 'max_foot_velocity_delta', 'recovery_tilt_deg', 'recovery_gyro', 'recovery_speed', 'recovery_delay_s', 'upper_target_speed', 'force_filter_s')
parser = argparse.ArgumentParser(description='Simulate deployed dance -> native C++ stopping core -> ten seconds of independently scored quiet standing.')
parser.add_argument('--training-root', type=Path, required=True)
parser.add_argument('--yaw', type=float, default=0.0)
parser.add_argument('--noise-seed', type=int)
parser.add_argument('--push', type=float, default=0, help='Signed forward push in newtons for 0.15 seconds after first standing confirmation; simulation only')
parser.add_argument('--onnxruntime-site', type=Path, help='Optional existing site-packages containing onnxruntime; appended after loading primary NumPy/MuJoCo/Torch')
parser.add_argument('--out', type=Path, required=True)
args = parser.parse_args()
if args.onnxruntime_site:
    sys.path.append(str(args.onnxruntime_site))
import onnxruntime as ort
torch.set_num_threads(1)

def hook(context):
    (data, model) = (context['data'], context['model'])
    names = context['names']
    (qadr, vadr, act) = (context['qadr'], context['vadr'], context['actuators'])
    math = context['math']
    cfg = context['cfg']['controllers']['rl_dance_finish']
    assert cfg['joint_list'] == names
    assert cfg['amp_conf']['constant_cmd'] == [0.0, 0.0, 0.0] and cfg['amp_conf']['settle_ticks'] == 0
    finish_cfg = cfg['dance_finish']
    params = np.array([finish_cfg[k] for k in PARAMETER_KEYS], np.float64)
    default = np.array(cfg['init_state'])
    kp = np.array(cfg['stiffness'])
    kd = np.array(cfg['damping'])
    standkp = np.array(finish_cfg['stand_stiffness'])
    standkd = np.array(finish_cfg['stand_damping'])
    source_hashes = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in (control/'include/control_module/dance_finish_core.h', control/'test/dance_finish_capi.cc', control/'policy'/Path(cfg['onnx_conf']['policy_file']).name)}
    options = ort.SessionOptions()
    options.intra_op_num_threads = 1
    options.inter_op_num_threads = 1
    policy = ort.InferenceSession(str(control / 'policy' / Path(cfg['onnx_conf']['policy_file']).name), sess_options=options, providers=['CPUExecutionProvider'])
    tmp = tempfile.TemporaryDirectory(prefix='finish_core_')
    libpath = Path(tmp.name) / 'core.so'
    subprocess.run(['clang++', '-std=c++17', '-O2', '-shared', '-fPIC', '-I', str(control / 'include'), str(control / 'test/dance_finish_capi.cc'), '-o', str(libpath)], check=True)
    lib = ctypes.CDLL(str(libpath))
    ptr = np.ctypeslib.ndpointer(dtype=np.float64, ndim=1, flags='C_CONTIGUOUS')
    iptr = np.ctypeslib.ndpointer(dtype=np.int32, ndim=1, flags='C_CONTIGUOUS')
    lib.dance_finish_create.argtypes = [ptr, ptr, iptr]
    lib.dance_finish_create.restype = ctypes.c_void_p
    lib.dance_finish_begin.argtypes = [ctypes.c_void_p, ptr, ctypes.c_int64]
    lib.dance_finish_begin.restype = ctypes.c_int
    lib.dance_finish_update.argtypes = [ctypes.c_void_p, ptr, ptr, ctypes.c_int64, ctypes.c_int64, ctypes.c_int, ptr]
    lib.dance_finish_reason.argtypes = [ctypes.c_void_p]
    lib.dance_finish_reason.restype = ctypes.c_char_p
    lib.dance_finish_destroy.argtypes = [ctypes.c_void_p]
    legs = np.array([names.index(side + '_' + joint + '_joint') for side in ('left', 'right') for joint in ('hip_pitch', 'hip_roll', 'hip_yaw', 'knee_pitch', 'ankle_pitch', 'ankle_roll')], np.int32)
    joints = np.r_[context['lower'], context['upper'], default, kp, kd, standkp, standkd].astype(np.float64)
    core = lib.dance_finish_create(params, joints, legs)
    assert core
    assert lib.dance_finish_begin(core, context['target'].copy().astype(np.float64), 1000000000)
    hist = np.zeros((3, 96), np.float32)
    last = np.zeros(29, np.float32)
    proposed = context['target'].copy().astype(np.float64)
    output = np.zeros(99, np.float64)
    events = []
    samples = []
    oldphase = -1
    first_confirmed = None
    stable_since = None
    quiet_origin = None
    push_start = None
    push_done = False
    postmax = 0.0
    minz = 10.0
    passed = False
    rng = np.random.default_rng(args.noise_seed) if args.noise_seed is not None else None
    for tick in range(30001):
        mujoco.mj_forward(model, data)
        t = tick * 0.001
        q = data.qpos[qadr].copy()
        qd = data.qvel[vadr].copy()
        gyro = data.qvel[3:6].copy()
        torque = data.qfrc_actuator[vadr].copy()
        quat = data.qpos[3:7].copy()
        if rng is not None:
            q += rng.normal(0, 0.0005, 29)
            qd += rng.normal(0, 0.005, 29)
            gyro += rng.normal(0, 0.001, 3)
            torque += rng.normal(0, 0.3, 29)
        if tick % 20 == 0:
            gravity = math.quat_rotate_inv(quat, np.array([0.0, 0.0, -1.0]))
            obs = np.r_[gyro, gravity, np.zeros(3), q - default, qd, last].astype(np.float32)
            hist[:-1] = hist[1:].copy()
            hist[-1] = obs
            tensor = np.concatenate([hist[:, l:r].flatten() for (l, r) in zip([0, 3, 6, 9, 38, 67], [3, 6, 9, 38, 67, 96])])[None, :]
            action = policy.run(None, {'obs': np.clip(tensor, -100, 100)})[0][0]
            last = action.copy()
            proposed = np.clip(default + 0.25 * action, context['lower'], context['upper']).astype(np.float64)
        values = np.r_[quat, gyro, q, qd, torque].astype(np.float64)
        lib.dance_finish_update(core, values, proposed, 1000000000 + tick * 1000000, 1000000000 + tick * 1000000, 1, output)
        phase = int(output[96])
        confirmed = bool(output[97])
        reason = lib.dance_finish_reason(core).decode()
        if phase != oldphase:
            ev = dict(t_s=t, phase=phase, reason=reason, quiet_s=float(output[94]), force=output[87:89].tolist(), contact=output[89:91].tolist(), est_velocity=output[91:94].tolist(), true_speed=float(np.linalg.norm(data.qvel[:3])), max_qd=float(np.max(np.abs(data.qvel[vadr]))))
            events.append(ev)
            print(json.dumps(ev), flush=True)
            oldphase = phase
        tilt = float(np.degrees(np.arccos(np.clip(math.quat_to_rotmat(quat)[2, 2], -1, 1))))
        postmax = max(postmax, tilt)
        minz = min(minz, float(data.qpos[2]))
        truefeet = set()
        for contact in data.contact:
            b = {int(model.geom_bodyid[contact.geom1]), int(model.geom_bodyid[contact.geom2])}
            for foot in context['feet']:
                if 0 in b and foot in b:
                    truefeet.add(foot)
        quiet_truth = tilt < 10 and np.linalg.norm(data.qvel[:3]) < 0.05 and (np.linalg.norm(data.qvel[3:6]) < 0.2) and (np.max(np.abs(data.qvel[vadr])) < 0.5) and (len(truefeet) == 2)
        if quiet_truth and confirmed:
            if stable_since is None:
                stable_since = t
                quiet_origin = data.qpos[:2].copy()
        else:
            stable_since = None
            quiet_origin = None
        if confirmed and first_confirmed is None:
            first_confirmed = t
            print('CONFIRMED', t, flush=True)
            if args.push:
                push_start = t + 2.0
        force = np.zeros(6)
        if push_start is not None and push_start <= t < push_start + 0.15:
            force[:3] = math.quat_rotate(math.yaw_quat(quat), np.array([args.push, 0, 0]))
            push_done = True
            stable_since = None
        data.xfrc_applied[model.body('base_link').id] = force
        if tick % 500 == 0:
            samples.append(dict(t_s=t, phase=phase, confirmed=confirmed, reason=reason, tilt_deg=tilt, true_speed=float(np.linalg.norm(data.qvel[:3])), true_gyro=float(np.linalg.norm(data.qvel[3:6])), max_qd=float(np.max(np.abs(data.qvel[vadr]))), true_both_feet=len(truefeet) == 2, force=output[87:89].tolist(), estimated_velocity=output[91:94].tolist(), quiet_s=float(output[94])))
        if phase == 5 or tilt > 35 or data.qpos[2] < 0.35:
            break
        if stable_since is not None and t - stable_since >= 10 and (not args.push or push_done):
            passed = np.linalg.norm(data.qpos[:2]-quiet_origin) <= .05
            break
        data.ctrl[act] = np.clip(output[29:58] * (output[:29] - data.qpos[qadr]) - output[58:87] * data.qvel[vadr], -context['effort'], context['effort'])
        mujoco.mj_step(model, data)
    result = dict(parameters={k:finish_cfg[k] for k in PARAMETER_KEYS}, source_sha256=source_hashes, versions=dict(numpy=np.__version__,torch=torch.__version__,mujoco=mujoco.__version__,onnxruntime=ort.__version__), quiet_xy_drift_m=float(np.linalg.norm(data.qpos[:2]-quiet_origin)) if quiet_origin is not None else None, passed=bool(passed), yaw=args.yaw, noise_seed=args.noise_seed, push_N=args.push, elapsed_s=t, first_confirmed_s=first_confirmed, final_phase=phase, final_reason=reason, max_tilt_deg=postmax, min_z_m=minz, final_true_speed=float(np.linalg.norm(data.qvel[:3])), final_true_gyro=float(np.linalg.norm(data.qvel[3:6])), final_max_joint_speed=float(np.max(np.abs(data.qvel[vadr]))), continuous_verified_quiet_s=t - stable_since if stable_since is not None else 0, recoveries=float(output[98]), events=events, samples=samples)
    lib.dance_finish_destroy(core)
    tmp.cleanup()
    return result
r = run(args.training_root, 3.0, args.yaw, 10.0, postroll_hook=hook)
args.out.parent.mkdir(parents=True, exist_ok=True)
args.out.write_text(json.dumps(r, indent=2) + '\n')
print(json.dumps({k: v for (k, v) in r['postroll'].items() if k not in ['events', 'samples']}, indent=2))
if not r['postroll']['passed']:
    raise SystemExit(1)
