#!/bin/bash
# 实时线程调度验证（文档 4.2 节方法）
# 逐线程校验 ecat_io_loop / rl_control_pub 的 cls(SCHED_FIFO)/rtprio/psr。
#
# 关键认知（核实 comm_utils.cpp:40-75 与 ethercat_manager.cpp:316 / control_module.cc:236）：
#   - SetRealTimeThread 内 setname 发生在 setschedparam 之前，故即使 RT 设置失败
#     （非 root、内核配置问题等），线程名仍在但 cls=TS（假实时）。本脚本必须同时
#     校验 comm + cls/rtprio/psr，才能识别"配了但没拿到"的假实时。
#   - 线程名经 pthread_setname_np 截断为 15 字符：ecat_io_loop(12) / rl_control_pub(14)。
#     注意 rl_control_pub 不是 rl_control_pub_thread（后者是 AimRT executor 名，21 字符）。
#   - 调用方（control_module.cc:237 / ethercat_manager.cpp:317）在 RT 设置失败时只打
#     ERROR 不退出，进程静默降级继续跑——本脚本正是兜底查出这种情况。
#   - 注意与本目录 rt_walk_verify.py 区分：那是直线行走偏航 PD 验证，与线程调度无关。
#
# 用法: rt_verify.sh [pid] [dcu_cfg] [rl_cfg]
#   pid      默认 pidof aimrt_main
#   dcu_cfg  默认 ./cfg/dcu_driver_module/dcu_x1.yaml
#   rl_cfg   默认 ./cfg/control_module/rl_x1.yaml
# 退出码：0=全过 1=有失败

set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPER="${HERE}/cfg_helper.py"

PID="${1:-$(pidof aimrt_main 2>/dev/null | awk '{print $1}')}"
DCU_CFG="${2:-./cfg/dcu_driver_module/dcu_x1.yaml}"
RL_CFG="${3:-./cfg/control_module/rl_x1.yaml}"
VERIFY_TIMEOUT="${VERIFY_TIMEOUT:-15}"

G=$'\033[32m'; Y=$'\033[33m'; R=$'\033[31m'; B=$'\033[36m'; N=$'\033[0m'

# ---- 前置检查 ----
if [ -z "$PID" ]; then
    echo "${R}[rt_verify] 找不到 aimrt_main 进程${N}"; exit 1
fi
if ! kill -0 "$PID" 2>/dev/null; then
    echo "${R}[rt_verify] 进程 $PID 不存在${N}"; exit 1
fi
for t in ps chrt taskset python3; do
    command -v "$t" >/dev/null 2>&1 || { echo "${R}[rt_verify] 缺少工具: $t${N}"; exit 1; }
done

# ---- 从 yaml 读期望值 ----
ECAT_PRIO="$(python3 "$HELPER" get "$DCU_CFG" ethercat.rt_priority 2>/dev/null)"
ECAT_CPU="$(python3 "$HELPER" get "$DCU_CFG" ethercat.bind_cpu 2>/dev/null)"
CTRL_PRIO="$(python3 "$HELPER" get "$RL_CFG" rt_priority 2>/dev/null)"
CTRL_CPU="$(python3 "$HELPER" get "$RL_CFG" bind_cpu 2>/dev/null)"
if [ -z "$ECAT_PRIO" ] || [ -z "$ECAT_CPU" ] || [ -z "$CTRL_PRIO" ] || [ -z "$CTRL_CPU" ]; then
    echo "${R}[rt_verify] 无法从 yaml 读期望值（dcu=$DCU_CFG rl=$RL_CFG）${N}"; exit 1
fi
echo "${B}[rt_verify]${N} 期望: ecat_io_loop prio=$ECAT_PRIO cpu=$ECAT_CPU | rl_control_pub prio=$CTRL_PRIO cpu=$CTRL_CPU"

# ---- 轮询等两个目标线程出现 ----
echo "${B}[rt_verify]${N} 轮询等待目标线程就绪（最多 ${VERIFY_TIMEOUT}s）..."
elapsed=0
while [ "$elapsed" -lt "$VERIFY_TIMEOUT" ]; do
    if ps -T -p "$PID" -o comm= 2>/dev/null | grep -qw 'ecat_io_loop' && \
       ps -T -p "$PID" -o comm= 2>/dev/null | grep -qw 'rl_control_pub'; then
        break
    fi
    sleep 1; elapsed=$((elapsed+1))
done
found_ecat="$(ps -T -p "$PID" -o comm= 2>/dev/null | grep -w 'ecat_io_loop' || true)"
found_ctrl="$(ps -T -p "$PID" -o comm= 2>/dev/null | grep -w 'rl_control_pub' || true)"
if [ -z "$found_ecat" ] || [ -z "$found_ctrl" ]; then
    echo "${R}[rt_verify] 超时：目标线程未全部出现（ecat_io_loop=${found_ecat:-无} rl_control_pub=${found_ctrl:-无}）${N}"
    echo "${B}[rt_verify]${N} 当前线程表："
    ps -T -p "$PID" -o tid,comm,cls,rtprio,psr 2>/dev/null
    exit 1
fi

# ---- 打印全线程表 ----
echo "${B}[rt_verify]${N} 当前线程表："
ps -T -p "$PID" -o tid,comm,cls,rtprio,psr 2>/dev/null

# ---- 逐线程校验 ----
check_thread() {
    local name="$1" exp_prio="$2" exp_cpu="$3"
    local line tid cls rtprio psr
    line="$(ps -T -p "$PID" -o tid=,comm=,cls=,rtprio=,psr= 2>/dev/null | awk -v n="$name" '$2==n{print; exit}')"
    if [ -z "$line" ]; then
        echo "  ${R}FAIL ${name}: 线程不存在${N}"
        return 1
    fi
    read -r tid _ cls rtprio psr <<<"$line"
    local ok=1
    [ "$cls" = "FF" ] || { echo "  ${R}FAIL ${name}: cls=$cls（期望 FF=SCHED_FIFO，疑似假实时）${N}"; ok=0; }
    [ "$rtprio" = "$exp_prio" ] || { echo "  ${R}FAIL ${name}: rtprio=$rtprio（期望 $exp_prio）${N}"; ok=0; }
    [ "$psr" = "$exp_cpu" ] || echo "  ${Y}WARN ${name}: psr=$psr（期望 $exp_cpu，CPU 可能迁移，看下方 taskset 确认）${N}"
    if [ "$ok" = 1 ]; then
        echo "  ${G}PASS ${name}: tid=$tid cls=$cls rtprio=$rtprio psr=$psr${N}"
    fi
    echo "    chrt:      $(chrt -p "$tid" 2>/dev/null | tr '\n' ' ')"
    echo "    affinity:  $(taskset -cp "$tid" 2>/dev/null | tail -1)"
    return $((1-ok))
}

fail=0
check_thread ecat_io_loop "$ECAT_PRIO" "$ECAT_CPU" || fail=1
check_thread rl_control_pub "$CTRL_PRIO" "$CTRL_CPU" || fail=1

if [ "$fail" = 0 ]; then
    echo "${G}[rt_verify] 全部通过${N}"; exit 0
else
    echo "${R}[rt_verify] 存在失败项，详见上方${N}"; exit 1
fi
