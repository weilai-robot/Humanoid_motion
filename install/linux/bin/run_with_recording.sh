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

# ── 颈部双舵机使能（小脑/大脑运行期间保持不动）─────────────────
# 串口可用环境变量覆盖: NECK_SERVO_PORT
# 默认失败仅告警放行; 设 NECK_SERVO_REQUIRED=1 则使能失败中止启动
NECK_PORT="${NECK_SERVO_PORT:-/dev/ttyUSB0}"
if [ -e "${NECK_PORT}" ]; then
    timeout 10 python3 "${SCRIPT_DIR}/neck_hold.py" --port "${NECK_PORT}"
    neck_rc=$?
    if [ ${neck_rc} -ne 0 ]; then
        if [ "${NECK_SERVO_REQUIRED:-0}" = "1" ]; then
            echo "[run_with_recording] 颈部舵机使能失败 (rc=${neck_rc}), NECK_SERVO_REQUIRED=1, 中止启动"
            exit 1
        fi
        echo "[run_with_recording][WARN] 颈部舵机使能失败 (rc=${neck_rc}), 继续启动 —— 头部可能松动!"
    fi
else
    echo "[run_with_recording][WARN] 未发现颈部舵机串口 ${NECK_PORT}, 跳过使能"
fi

# ── 退出/终止时下使能颈部舵机（与 run.sh 一致）─────────────────
# 覆盖: aimrt_main 正常退出、Ctrl+C(SIGINT)、kill(SIGTERM)、挂断(SIGHUP)
_neck_released=0
release_neck() {
    [ "${_neck_released}" = 1 ] && return
    _neck_released=1
    if [ -e "${NECK_PORT}" ]; then
        timeout 5 python3 -c "import sys; sys.path.insert(0, '${SCRIPT_DIR}'); from htd35h_controller import HTD35HController; import time; c=HTD35HController(port='${NECK_PORT}', baudrate=115200); c.enable_torque_all([1,2], False); time.sleep(0.3); c.close()" 2>/dev/null
        echo "[run_with_recording] 颈部舵机已下使能"
    fi
}
trap release_neck EXIT INT TERM HUP

sudo setcap cap_net_raw=ep ./aimrt_main

set -m

bash record.sh &
record_pid=$!
pgid=$(ps -o pgid= -p $record_pid | tr -d ' ')
# echo "Child script PGID: $pgid"

./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml
aimrt_rc=$?

kill -TERM -$pgid 2>/dev/null

# aimrt_main 退出后, 由 EXIT trap 自动执行 release_neck 下使能
exit ${aimrt_rc}
