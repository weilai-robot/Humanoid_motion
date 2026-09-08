#!/bin/bash
###
 # @Author: richie.li
 # @Date: 2024-11-18 17:03:42
 # @LastEditors: richie.li
 # @LastEditTime: 2024-11-18 17:12:12
### 

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

# ── 颈部双舵机锁定（小脑/大脑运行期间保持不动）─────────────────
# Mid-360 装在头部: 默认 home(回中500,500后锁定), 保证雷达外参与标定一致
# 串口/模式可用环境变量覆盖: NECK_SERVO_PORT / NECK_SERVO_MODE(lock|home)
# 默认失败仅告警放行; 设 NECK_SERVO_REQUIRED=1 则锁定失败中止启动
NECK_PORT="${NECK_SERVO_PORT:-/dev/ttyUSB0}"
NECK_MODE="${NECK_SERVO_MODE:-home}"
if [ -e "${NECK_PORT}" ]; then
    timeout 10 python3 "${SCRIPT_DIR}/neck_hold.py" --port "${NECK_PORT}" --mode "${NECK_MODE}"
    neck_rc=$?
    if [ ${neck_rc} -ne 0 ]; then
        if [ "${NECK_SERVO_REQUIRED:-0}" = "1" ]; then
            echo "[run_with_recording] 颈部舵机锁定失败 (rc=${neck_rc}), NECK_SERVO_REQUIRED=1, 中止启动"
            exit 1
        fi
        echo "[run_with_recording][WARN] 颈部舵机锁定失败 (rc=${neck_rc}), 继续启动 —— 头部可能松动!"
    fi
else
    echo "[run_with_recording][WARN] 未发现颈部舵机串口 ${NECK_PORT}, 跳过锁定"
fi

sudo setcap cap_net_raw=ep ./aimrt_main

set -m

bash record.sh &
record_pid=$!
pgid=$(ps -o pgid= -p $record_pid | tr -d ' ')
# echo "Child script PGID: $pgid"

./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml

kill -TERM -$pgid 2>/dev/null
