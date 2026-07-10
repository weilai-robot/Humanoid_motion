#!/bin/bash

# 自动探测并 source ROS2 环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
    source "${SCRIPT_DIR}/ros2_source.sh"
fi

if [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi

# ── 显示环境自动检测 ────────────────────────────────────────────────
# 如果当前没有可用的 DISPLAY（Docker/无头环境），自动启动 Xvfb 虚拟显示
if [ -z "$DISPLAY" ]; then
    if command -v Xvfb &> /dev/null; then
        echo "[run_sim] 未检测到 DISPLAY，启动 Xvfb 虚拟显示 :99 ..."
        Xvfb :99 -screen 0 1280x720x24 -ac +extension GLX +render -noreset &
        XVFB_PID=$!
        export DISPLAY=:99
        sleep 1  # 等待 Xvfb 就绪
        echo "[run_sim] Xvfb 已启动 (PID=$XVFB_PID)，DISPLAY=$DISPLAY"
    else
        echo "[run_sim] 警告: 未找到 Xvfb，MuJoCo 渲染可能失败。"
        echo "[run_sim] 请安装: apt-get install -y xvfb"
    fi
else
    echo "[run_sim] 使用已有显示: DISPLAY=$DISPLAY"
fi
# ───────────────────────────────────────────────────────────────────

./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml
# gdb --args ./aimrt_main --cfg_file_path=./cfg/x1_cfg_sim.yaml

# 退出时清理 Xvfb
if [ -n "$XVFB_PID" ]; then
    kill $XVFB_PID 2>/dev/null
fi
