#!/bin/bash

# ============================================================
# run_real_nav.sh — 真机「导航联合」小脑启动 (不加载手柄)
#
# 与 run.sh 的差异:
#   1. 使用 x1_cfg_real_nav.yaml — 不加载 JoyStickModule,
#      导航期间 /cmd_vel_limiter 唯一来源为 Nav2 → odom_bridge
#   2. cap_net_raw 已具备时跳过 setcap (run.sh 每次都会 sudo)
#
# 前置:
#   - 已在构建机重新构建部署（本脚本与 x1_cfg_real_nav.yaml
#     由 ./scripts/build.sh 安装到 build/）
#   - 模式切换必须用外部脚本（手柄不可用）:
#     ../scripts/switch_x1_mode.sh ready
#
# 用法:
#   cd build && bash ./run_real_nav.sh
# ============================================================

# 以脚本所在目录为工作目录（cfg/log 均按相对路径解析）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# 自动探测并 source ROS2 环境
if [ -f "${SCRIPT_DIR}/ros2_source.sh" ]; then
    source "${SCRIPT_DIR}/ros2_source.sh"
fi

if [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi

# ── 配置检查 ─────────────────────────────────────────────────
if [ ! -f ./cfg/x1_cfg_real_nav.yaml ]; then
    echo "[run_real_nav][ERROR] 缺少配置 ./cfg/x1_cfg_real_nav.yaml"
    echo "  请在构建机重新执行 ./scripts/build.sh 后再部署"
    exit 1
fi

# ── 颈部双舵机使能（小脑/大脑运行期间保持不动）─────────────────
# 串口可用环境变量覆盖: NECK_SERVO_PORT
# 默认失败仅告警放行; 设 NECK_SERVO_REQUIRED=1 则使能失败中止启动
NECK_PORT="${NECK_SERVO_PORT:-/dev/ttyUSB0}"
if [ -e "${NECK_PORT}" ]; then
    timeout 10 python3 "${SCRIPT_DIR}/neck_hold.py" --port "${NECK_PORT}"
    neck_rc=$?
    if [ ${neck_rc} -ne 0 ]; then
        if [ "${NECK_SERVO_REQUIRED:-0}" = "1" ]; then
            echo "[run_real_nav] 颈部舵机使能失败 (rc=${neck_rc}), NECK_SERVO_REQUIRED=1, 中止启动"
            exit 1
        fi
        echo "[run_real_nav][WARN] 颈部舵机使能失败 (rc=${neck_rc}), 继续启动 —— 头部可能松动!"
    fi
else
    echo "[run_real_nav][WARN] 未发现颈部舵机串口 ${NECK_PORT}, 跳过使能"
fi

# ── 退出/终止时下使能颈部舵机 ─────────────────────────────────
# 覆盖: aimrt_main 正常退出、Ctrl+C(SIGINT)、kill(SIGTERM)、挂断(SIGHUP)
_neck_released=0
release_neck() {
    [ "${_neck_released}" = 1 ] && return
    _neck_released=1
    if [ -e "${NECK_PORT}" ]; then
        timeout 5 python3 -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}'); from htd35h_controller import HTD35HController; import time; c=HTD35HController(port='${NECK_PORT}', baudrate=115200); c.enable_torque_all([1,2], False); time.sleep(0.3); c.close()" 2>/dev/null
        echo "[run_real_nav] 颈部舵机已下使能"
    fi
}
trap release_neck EXIT INT TERM HUP

# ── cap_net_raw（EtherCAT 原始套接字权限）─────────────────────
# 已具备则跳过; 缺失则尝试一次性设置（需要 sudo）
if command -v getcap >/dev/null 2>&1 && getcap ./aimrt_main 2>/dev/null | grep -q "cap_net_raw"; then
    echo "[run_real_nav] cap_net_raw 已就绪（跳过 setcap）"
else
    echo "[run_real_nav] 设置 cap_net_raw（需要 sudo 密码）..."
    sudo setcap cap_net_raw=ep ./aimrt_main || \
        echo "[run_real_nav][WARN] setcap 失败 —— EtherCAT 可能无法通信, 请检查权限"
fi

./aimrt_main --cfg_file_path=./cfg/x1_cfg_real_nav.yaml
aimrt_rc=$?

# aimrt_main 退出后, 由 EXIT trap 自动执行 release_neck 下使能
exit ${aimrt_rc}
