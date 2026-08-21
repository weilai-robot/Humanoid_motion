# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-FileCopyrightText: Copyright (c) 2021 ETH Zurich, Nikita Rudin
# SPDX-FileCopyrightText: Copyright (c) 2024 Beijing RobotEra TECHNOLOGY CO.,LTD. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Copyright (c) 2024, AgiBot Inc. All rights reserved.


import os
import re
import sys
import glob
import time
import base64
import shutil
import threading
import subprocess

from humanoid import LEGGED_GYM_ROOT_DIR
from humanoid.envs import *
from humanoid.utils import get_args, task_registry


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


def download_checkpoint_for_resume(url, load_run, checkpoint):
    """exp2.1 GM resume: 把签名 OSS checkpoint 下载到 get_load_path 期望的 exported_data 布局。"""
    resume_dir = os.path.join(LEGGED_GYM_ROOT_DIR, "logs", "x1_dh_stand", "exported_data", load_run)
    os.makedirs(resume_dir, exist_ok=True)
    dest = os.path.join(resume_dir, "model_{}.pt".format(checkpoint))
    print("[train] Downloading resume checkpoint -> {}".format(dest))
    result = subprocess.run(["curl", "-L", "--retry", "3", "-o", dest, url],
                            capture_output=True, text=True, timeout=300)
    if result.returncode != 0 or not os.path.exists(dest) or os.path.getsize(dest) < 1_000_000:
        raise RuntimeError("checkpoint download failed: rc={} {}".format(result.returncode, result.stderr[:200]))
    print("[train] Downloaded {} bytes".format(os.path.getsize(dest)))
    return resume_dir


def start_model_mirroring(log_dir, mirror_dir, interval=30.0):
    """exp2.1: 后台线程把训练新保存的 model_*.pt 镜像到 SDK 识别的 PT 目录（checkpoint 下载目录）。

    GM SDK 只从启动阶段识别的 PT 目录上传；训练新建的 run 目录只触发
    "New file detected globally" 而不进上传队列（TASK_20260821_010 实测丢失全部
    训练产物，机制见 czy/skills/flux-cli/references/replay.md）。镜像让每个
    checkpoint 在训练过程中即被持续上传。返回 (stop_event, copy_pass)。"""
    stop = threading.Event()

    def copy_pass(force=False):
        copied = []
        for f in sorted(glob.glob(os.path.join(log_dir, "model_*.pt"))):
            try:
                if not force and time.time() - os.path.getmtime(f) < 5.0:
                    continue  # 刚写入的文件可能未写完，等下一轮
                dest = os.path.join(mirror_dir, os.path.basename(f))
                if not os.path.exists(dest):
                    tmp = dest + ".tmp"
                    shutil.copy2(f, tmp)
                    os.replace(tmp, dest)  # 原子替换，避免 SDK 读到半截文件
                    copied.append(os.path.basename(f))
            except Exception as e:
                print("[train] Mirror skip {}: {}".format(f, e))
        for name in copied:
            print("[train] Mirrored {} -> {}".format(name, mirror_dir))
        return copied

    def _worker():
        while not stop.wait(interval):
            copy_pass()

    threading.Thread(target=_worker, daemon=True).start()
    return stop, copy_pass


def train(args, mirror_dir=None):
    env, env_cfg = task_registry.make_env(name=args.task, args=args)
    ppo_runner, train_cfg, log_dir = task_registry.make_alg_runner(env=env, name=args.task, args=args)
    stop_mirror = None
    final_pass = None
    if mirror_dir and log_dir:
        stop_mirror, final_pass = start_model_mirroring(log_dir, mirror_dir)
    try:
        ppo_runner.learn(num_learning_iterations=train_cfg.runner.max_iterations, init_at_random_ep_len=False)
    finally:
        if stop_mirror is not None:
            stop_mirror.set()
            time.sleep(5.0)  # 等最终 save 完全落盘
            final_pass(force=True)
            print("[train] Waiting 120s for SDK to upload mirrored models...")
            time.sleep(120)

if __name__ == '__main__':
    url = extract_checkpoint_url_b64(sys.argv)
    args = get_args()
    if url:
        # exp2.1 GM resume: 平台经签名 URL 交付 checkpoint（挂载不可靠），运行时下载后走常规 --resume
        if not args.load_run:
            args.load_run = "gm_resume"
        if args.checkpoint is None or args.checkpoint < 0:
            m = re.search(r"model_(\d+)", url)
            args.checkpoint = int(m.group(1)) if m else 3000
        mirror_dir = download_checkpoint_for_resume(url, args.load_run, args.checkpoint)
        train(args, mirror_dir=mirror_dir)
    else:
        train(args)
