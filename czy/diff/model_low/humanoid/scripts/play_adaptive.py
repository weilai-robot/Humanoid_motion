# Playback script to demonstrate the low-speed transition gait for X1
# (±0.2 m/s walking + in-place stepping at zero command).
# Based on play.py structure (headless-safe, no pygame/joystick dependency).
# Features:
#   - Stepped commands (0.2 -> 0 -> -0.2 -> 0 -> 0.2 -> 0 m/s, then yaw
#     0.6 rad/s) so the rapid decel/accel transitions, in-place stepping and
#     in-place turning are directly visible in the video and in the CSV.
#   - Overlays on the recorded video: command/actual/average speed and yaw,
#     per-foot contact state (ON/OFF + force), and the gait cycle time.
#   - Writes isaac_diag.csv with per-step speed, foot height, foot force, etc.

import os
import sys
import glob
import csv
import math
import re
import shutil
import base64
import subprocess
import numpy as np
import cv2
from datetime import datetime

# NOTE: isaacgym must be imported before torch, otherwise gymdeps raises
# "PyTorch was imported before isaacgym modules".
from isaacgym import gymapi
from isaacgym.torch_utils import *

import torch

from humanoid import LEGGED_GYM_ROOT_DIR
from humanoid.envs import *
from humanoid.utils import get_args, task_registry

# ---------------------------------------------------------------------------
# Speed profile: each entry is (num_control_steps, command_x [m/s], command_yaw [rad/s]).
# Control loop runs at 50 Hz, so 500 steps == 10 s per segment.
# Low-speed transition model: ±0.2 m/s; at 0 command the phase keeps
# advancing (in-place stepping), gait cycle fixed at 0.7 s.
# Final segment adds in-place turning (yaw 0.6 rad/s).
# ---------------------------------------------------------------------------
VEL_PROFILE = [
    (500, 0.2, 0.0),   # slow forward walk
    (500, 0.0, 0.0),   # rapid decel -> in-place stepping
    (500, -0.2, 0.0),  # backward walk
    (500, 0.0, 0.0),   # decel -> stepping
    (500, 0.2, 0.0),   # rapid accel to forward
    (500, 0.0, 0.0),   # stop, keep stepping
    (500, 0.0, 0.6),   # in-place turning (left, 0.6 rad/s) while stepping
]
TOTAL_STEPS = sum(steps for steps, _, _ in VEL_PROFILE)

RENDER = True
FIX_COMMAND = True
CONTACT_THRESHOLD_N = 1.0
CHECKPOINT_URL = None  # set from --checkpoint_url_b64 in __main__ (cloud replay mode)


def current_command(step_idx):
    """Return the (command_x, command_yaw) active at control step step_idx."""
    acc = 0
    for steps, vx, yaw in VEL_PROFILE:
        if step_idx < acc + steps:
            return vx, yaw
        acc += steps
    return 0.0, 0.0


def find_latest_checkpoint():
    """Search logs/x1_dh_stand/exported_data/** for the newest model_*.pt."""
    base = os.path.join(LEGGED_GYM_ROOT_DIR, "logs", "x1_dh_stand", "exported_data")
    if not os.path.exists(base):
        return None
    models = sorted(
        glob.glob(os.path.join(base, "**", "model_*.pt"), recursive=True),
        key=os.path.getmtime,
    )
    return models[-1] if models else None


def extract_checkpoint_url_b64(argv):
    """Pop --checkpoint_url_b64=<url-safe-b64> from argv (unknown to get_args)."""
    prefix = "--checkpoint_url_b64="
    for i, a in enumerate(argv):
        if a.startswith(prefix):
            argv.pop(i)
            padded = a[len(prefix):]
            padded += "=" * (-len(padded) % 4)
            return base64.urlsafe_b64decode(padded).decode("utf-8")
    return None


def download_checkpoint(url):
    """Download checkpoint from a signed OSS URL into logs/x1_dh_stand/gm_play/.

    The OSS filename carries a timestamp suffix (model_5000_2026...A845.pt); the
    runner later reassembles model_<N>.pt from the checkpoint number, so save
    under the normalized name."""
    download_dir = os.path.join(LEGGED_GYM_ROOT_DIR, "logs", "x1_dh_stand", "gm_play")
    os.makedirs(download_dir, exist_ok=True)
    from urllib.parse import unquote, urlparse
    path = unquote(urlparse(url).path)
    base = path.rsplit("/", 1)[-1] if "/" in path else path
    m = re.match(r"model_(\d+)", base)
    name = f"model_{m.group(1)}.pt" if m else "model_downloaded.pt"
    download_path = os.path.join(download_dir, name)
    print(f"[play_adaptive] Downloading checkpoint from OSS -> {download_path}")
    try:
        result = subprocess.run(
            ["curl", "-L", "--retry", "3", "-o", download_path, url],
            capture_output=True, text=True, timeout=300,
        )
        if result.returncode == 0 and os.path.exists(download_path) and os.path.getsize(download_path) > 1_000_000:
            print(f"[play_adaptive] Downloaded {os.path.getsize(download_path)} bytes")
            return download_path
        print(f"[play_adaptive] Download failed: rc={result.returncode} {result.stderr[:200]}")
    except Exception as e:
        print(f"[play_adaptive] Download error: {e}")
    return None


def package_artifacts_for_upload(video_path, csv_path):
    """Package video+csv as PTs into logs/x1_dh_stand/gm_play/ so the GM SDK
    uploads them (see czy/skills/flux-cli/references/replay.md)."""
    try:
        upload_dir = os.path.join(LEGGED_GYM_ROOT_DIR, "logs", "x1_dh_stand", "gm_play")
        os.makedirs(upload_dir, exist_ok=True)
        import torch as _torch
        with open(video_path, "rb") as f:
            vp = os.path.join(upload_dir, "model_isaac_video.pt")
            _torch.save({"bytes": f.read(), "filename": os.path.basename(video_path)}, vp)
            print(f"[play_adaptive] Packaged video PT -> {vp}")
        with open(csv_path, "rb") as f:
            cp = os.path.join(upload_dir, "model_isaac_csv.pt")
            _torch.save({"bytes": f.read(), "filename": os.path.basename(csv_path)}, cp)
            print(f"[play_adaptive] Packaged csv PT -> {cp}")
    except Exception as e:
        print(f"[play_adaptive] Artifact packaging skipped: {e}")


def save_diag_csv(diag, out_dir, num_actions=12, dt=0.02):
    """Write per-step diagnostics to isaac_diag.csv."""
    os.makedirs(out_dir, exist_ok=True)
    csv_path = os.path.join(out_dir, "isaac_diag.csv")
    header = ["step", "time_s", "command_x", "base_vel_x", "base_vel_y", "base_vel_yaw",
              "base_height", "base_pos_x", "base_pos_y", "base_yaw",
              "foot_z_l", "foot_z_r", "foot_force_l", "foot_force_r",
              "foot_yaw_l", "foot_yaw_r"]
    header += [f"dof_pos_{i}" for i in range(num_actions)]
    header += [f"dof_vel_{i}" for i in range(num_actions)]
    header += [f"dof_torque_{i}" for i in range(num_actions)]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        for i in range(len(diag["command_x"])):
            row = [i, round(i * dt, 6), diag["command_x"][i],
                   diag["base_vel_x"][i], diag["base_vel_y"][i], diag["base_vel_yaw"][i],
                   diag["base_height"][i],
                   diag["base_pos_x"][i], diag["base_pos_y"][i], diag["base_yaw"][i],
                   diag["foot_z_l"][i], diag["foot_z_r"][i],
                   diag["foot_force_l"][i], diag["foot_force_r"][i],
                   diag["foot_yaw_l"][i], diag["foot_yaw_r"][i]]
            row += diag["dof_pos"][i]
            row += diag["dof_vel"][i]
            row += diag["dof_torque"][i]
            writer.writerow(row)
    print(f"[play_adaptive] Saved diagnostic CSV -> {csv_path}")
    return csv_path


def draw_outlined_text(image, text, pos, color, scale=0.9):
    cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, pos, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def play(args):
    env_cfg, train_cfg = task_registry.get_cfgs(name=args.task)

    # Playback overrides (clean, deterministic single-robot rollout)
    env_cfg.env.num_envs = min(env_cfg.env.num_envs, 1)
    env_cfg.terrain.mesh_type = "plane"
    env_cfg.env.episode_length_s = 1000
    env_cfg.noise.add_noise = False
    for key in ["randomize_friction", "push_robots", "randomize_base_mass", "randomize_com",
                "randomize_gains", "randomize_torque", "randomize_link_mass",
                "randomize_motor_offset", "randomize_joint_friction",
                "randomize_joint_damping", "randomize_joint_armature",
                "randomize_lag_timesteps", "add_lag", "add_dof_lag"]:
        if hasattr(env_cfg.domain_rand, key):
            setattr(env_cfg.domain_rand, key, False)
    env_cfg.commands.heading_command = False
    env_cfg.noise.curriculum = False
    train_cfg.seed = 12345
    # exp2.1: 回放直接使用最终周期 cycle_time（0.58），不做周期退火
    if hasattr(env_cfg.rewards, "cycle_time_start"):
        env_cfg.rewards.cycle_time_start = None

    # Locate checkpoint: explicit --load_run/--checkpoint > --checkpoint_url_b64 download > latest local
    if args.load_run and args.checkpoint:
        train_cfg.runner.resume = True
        train_cfg.runner.load_run = args.load_run
        train_cfg.runner.checkpoint = args.checkpoint
        print(f"[play_adaptive] Using explicit load_run={args.load_run} checkpoint={args.checkpoint}")
    else:
        ckpt = None
        if CHECKPOINT_URL:
            ckpt = download_checkpoint(CHECKPOINT_URL)
        if ckpt is None:
            ckpt = find_latest_checkpoint()
        if ckpt is None:
            print("[play_adaptive] ERROR: no checkpoint under logs/x1_dh_stand/exported_data")
            sys.exit(1)
        # Copy into the gm_play layout that task_registry expects
        train_cfg.runner.resume = True
        train_cfg.runner.load_run = "gm_play"
        log_dir = os.path.join(LEGGED_GYM_ROOT_DIR, "logs", train_cfg.runner.experiment_name,
                               "exported_data", "gm_play")
        os.makedirs(log_dir, exist_ok=True)
        dest = os.path.join(log_dir, os.path.basename(ckpt))
        if not os.path.exists(dest):
            shutil.copy2(ckpt, dest)
            print(f"[play_adaptive] Copied checkpoint: {ckpt} -> {dest}")
        model_name = os.path.basename(ckpt)
        # filename may carry an OSS timestamp suffix, e.g. model_5000_20260818114923A845.pt
        match = re.match(r"model_(\d+)", model_name)
        if not match:
            print(f"[play_adaptive] ERROR: cannot parse checkpoint number from {model_name}")
            sys.exit(1)
        train_cfg.runner.checkpoint = int(match.group(1))

    # Build environment and policy
    # Keep GPU camera sensors alive under --headless (base_task otherwise forces
    # graphics_device_id=-1 and every camera frame comes back empty -> 0-byte video)
    if RENDER:
        env_cfg.env.enable_headless_render = True
    env, _ = task_registry.make_env(name=args.task, args=args, env_cfg=env_cfg)
    ppo_runner, train_cfg, _ = task_registry.make_alg_runner(
        env=env, name=args.task, args=args, train_cfg=train_cfg
    )
    policy = ppo_runner.get_inference_policy(device=env.device)
    print("[play_adaptive] Policy loaded successfully!")

    # Camera (follow view) + video writer
    camera_properties = gymapi.CameraProperties()
    camera_properties.width = 1920
    camera_properties.height = 1080
    h1 = env.gym.create_camera_sensor(env.envs[0], camera_properties)
    camera_offset = gymapi.Vec3(2.0, -2.0, 1.5)
    camera_rotation = gymapi.Quat.from_axis_angle(gymapi.Vec3(-0.3, 0.2, 1), np.deg2rad(135))
    actor_handle = env.gym.get_actor_handle(env.envs[0], 0)
    body_handle = env.gym.get_actor_rigid_body_handle(env.envs[0], actor_handle, 0)
    env.gym.attach_camera_to_body(
        h1, env.envs[0], body_handle,
        gymapi.Transform(camera_offset, camera_rotation),
        gymapi.FOLLOW_POSITION,
    )

    out_dir = os.path.join(LEGGED_GYM_ROOT_DIR, "logs", train_cfg.runner.experiment_name, "play_output")
    os.makedirs(out_dir, exist_ok=True)
    video_path = os.path.join(out_dir, "play_adaptive.mp4")
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    video = cv2.VideoWriter(video_path, fourcc, 25.0, (1920, 1080))
    print(f"[play_adaptive] Recording video to: {video_path}")

    left_foot_idx = env.feet_indices[0].item()
    right_foot_idx = env.feet_indices[1].item()

    # Per-step diagnostics
    diag = {k: [] for k in ["command_x", "base_vel_x", "base_vel_y", "base_vel_yaw",
                            "base_height", "base_pos_x", "base_pos_y", "base_yaw",
                            "foot_z_l", "foot_z_r",
                            "foot_force_l", "foot_force_r", "foot_yaw_l", "foot_yaw_r",
                            "dof_pos", "dof_vel", "dof_torque"]}

    obs = env.get_observations()
    vel_sum = 0.0
    step_accum = 0
    frame_count = 0

    for i in range(TOTAL_STEPS):
        cmd_x, cmd_yaw = current_command(i)

        if FIX_COMMAND:
            env.commands[:, 0] = cmd_x
            env.commands[:, 1] = 0.0
            env.commands[:, 2] = cmd_yaw
            env.commands[:, 3] = 0.0

        actions = policy(obs.detach())
        obs, critic_obs, rews, dones, infos = env.step(actions.detach())

        # Collect diagnostics
        real_cmd_x = env.commands[0, 0].item()
        cur_vel_x = env.base_lin_vel[0, 0].item()
        vel_sum += cur_vel_x
        step_accum += 1
        diag["command_x"].append(real_cmd_x)
        diag["base_vel_x"].append(cur_vel_x)
        diag["base_vel_y"].append(env.base_lin_vel[0, 1].item())
        diag["base_vel_yaw"].append(env.base_ang_vel[0, 2].item())
        diag["base_height"].append(env.root_states[0, 2].item())
        # World-frame base pose (for straight-line walking verification)
        diag["base_pos_x"].append(env.root_states[0, 0].item())
        diag["base_pos_y"].append(env.root_states[0, 1].item())
        diag["foot_z_l"].append(env.rigid_state[0, left_foot_idx, 2].item())
        diag["foot_z_r"].append(env.rigid_state[0, right_foot_idx, 2].item())
        diag["foot_force_l"].append(env.contact_forces[0, left_foot_idx, 2].item())
        diag["foot_force_r"].append(env.contact_forces[0, right_foot_idx, 2].item())
        # Real foot heading relative to base yaw (same convention as play_gm.py)
        feet_quat = env.feet_quat  # (num_envs, num_feet, 4) wxyz
        fqw, fqx, fqy, fqz = feet_quat[..., 0:1], feet_quat[..., 1:2], feet_quat[..., 2:3], feet_quat[..., 3:4]
        foot_fwd_x = 2.0 * (fqx * fqz + fqw * fqy)
        foot_fwd_y = 2.0 * (fqy * fqz - fqw * fqx)
        foot_fwd = torch.cat([foot_fwd_x, foot_fwd_y], dim=-1)  # (num_envs, num_feet, 2)
        base_quat = env.root_states[0, 3:7]
        base_yaw = torch.atan2(2.0 * (base_quat[3] * base_quat[2] + base_quat[0] * base_quat[1]),
                               1.0 - 2.0 * (base_quat[1] * base_quat[1] + base_quat[2] * base_quat[2]))
        foot_yaw_world = torch.atan2(foot_fwd[..., 1], foot_fwd[..., 0])
        foot_yaw_rel = (foot_yaw_world - base_yaw + torch.pi) % (2.0 * torch.pi) - torch.pi
        diag["base_yaw"].append(base_yaw.item())
        diag["foot_yaw_l"].append(foot_yaw_rel[0, 0].item())
        diag["foot_yaw_r"].append(foot_yaw_rel[0, 1].item())
        diag["dof_pos"].append(env.dof_pos[0].cpu().numpy().tolist())
        diag["dof_vel"].append(env.dof_vel[0].cpu().numpy().tolist())
        diag["dof_torque"].append(env.torques[0].cpu().numpy().tolist())

        # Render + overlay
        frame_count += 1
        env.gym.fetch_results(env.sim, True)
        env.gym.step_graphics(env.sim)
        env.gym.render_all_camera_sensors(env.sim)
        if frame_count % 2 == 0:
            img = env.gym.get_camera_image(env.sim, env.envs[0], h1, gymapi.IMAGE_COLOR)
            if img is not None and len(img) > 0:
                img = np.reshape(img, (1080, 1920, 4))
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                img = img[..., :3]

                avg_vel = vel_sum / step_accum if step_accum > 0 else 0.0
                cycle_time = env_cfg.rewards.cycle_time

                l_on = diag["foot_force_l"][-1] > CONTACT_THRESHOLD_N
                r_on = diag["foot_force_r"][-1] > CONTACT_THRESHOLD_N

                base_x = img.shape[1] - 1150
                base_y = 60
                lh = 50
                draw_outlined_text(img, f"CMD: {real_cmd_x:.2f} | REAL: {cur_vel_x:.2f} | AVG: {avg_vel:.2f} | CYCLE: {cycle_time:.2f}s",
                                   (base_x, base_y), (255, 255, 0), 1.0)
                l_color = (0, 255, 0) if l_on else (0, 0, 255)
                draw_outlined_text(img, f"L-FOOT: {'ON ' if l_on else 'OFF'} ({diag['foot_force_l'][-1]:.1f} N)",
                                   (base_x, base_y + lh), l_color)
                r_color = (0, 255, 0) if r_on else (0, 0, 255)
                draw_outlined_text(img, f"R-FOOT: {'ON ' if r_on else 'OFF'} ({diag['foot_force_r'][-1]:.1f} N)",
                                   (base_x, base_y + 2 * lh), r_color)

                if l_on and r_on:
                    state_text, state_color = "STATE: *** DOUBLE SUPPORT ***", (0, 255, 255)
                elif not l_on and not r_on:
                    state_text, state_color = "STATE: >>> FLIGHT PHASE <<<", (255, 0, 255)
                else:
                    state_text, state_color = "STATE: SINGLE SUPPORT", (200, 200, 200)
                draw_outlined_text(img, state_text, (base_x, base_y + 3 * lh), state_color, 1.0)

                video.write(img)

        if i % 500 == 0:
            print(f"[play_adaptive] Step {i}/{TOTAL_STEPS} | CMD={real_cmd_x:.2f} | vel_x={cur_vel_x:.3f} | height={env.root_states[0, 2].item():.3f}")

    video.release()
    print(f"[play_adaptive] Video saved to {video_path}")

    csv_path = save_diag_csv(diag, out_dir, env_cfg.env.num_actions, env.dt)

    print("\n[play_adaptive] === Summary ===")
    for seg_i, (steps, vel, yaw) in enumerate(VEL_PROFILE):
        seg_start = sum(s for s, _, _ in VEL_PROFILE[:seg_i])
        seg_vels = diag["base_vel_x"][seg_start:seg_start + steps]
        seg_yaws = diag["base_vel_yaw"][seg_start:seg_start + steps]
        print(f"  Segment {seg_i}: cmd={vel:+.2f} m/s, yaw={yaw:+.2f} rad/s | avg_real_vx={np.mean(seg_vels):+.3f} m/s, avg_real_yaw={np.mean(seg_yaws):+.3f} rad/s")
    print(f"  Video: {video_path}")
    print(f"  CSV:   {csv_path}")

    # Cloud mode: package artifacts so the GM SDK uploads them
    if CHECKPOINT_URL:
        package_artifacts_for_upload(video_path, csv_path)
        print("[play_adaptive] Keeping artifacts for SDK upload (60s)...")
        import time
        time.sleep(60)


if __name__ == "__main__":
    globals()["CHECKPOINT_URL"] = extract_checkpoint_url_b64(sys.argv)
    if CHECKPOINT_URL:
        print("[play_adaptive] Cloud mode: will download checkpoint from signed URL")
    args = get_args()
    play(args)
