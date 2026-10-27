#!/bin/bash
# rt_perm_check.sh — RT 调度资格预检仪器（"当前用户能否给自己的线程设置实时调度"）
# -----------------------------------------------------------------------------
# 被 run_main.sh 以参数调用；也可单独运行排查（在 build 部署目录下直接敲）。
#
# 为什么需要它：C++ 侧申请 SCHED_FIFO 失败时只打一行 ERROR 并继续跑（静默假实时，
# 线程名在但站在普通队列里）。本仪器在启动前把"资格"显式查清——失败即拦截并给出
# 补救指引（无逃生阀，带病运行请绕过启动器直跑 aimrt_main，见 README.md）。
#
# 三判据（任一命中即通过）：
#   ① 当前用户是 root —— 天然有资格
#   ② 目标二进制带 cap_sys_nice 文件能力（当前部署未用，判据留作将来）
#   ③ 当前会话的实时优先级限额（ulimit -r，即 RLIMIT_RTPRIO）≥ 需求值
#      限额由 /etc/security/limits.d/<user>-rt.conf 配置，登录时发放给会话
#
# 用法: rt_perm_check.sh [--need-prio N] [--bin <path>]
#   --need-prio  需要的最高实时优先级（默认 90，应由编排者从 cfg 读出传入）
#   --bin        目标二进制路径（默认 ./aimrt_main，判据②用）
# 退出码: 0=有资格  1=无资格（已打印补救指引）  64=用法错

set -uo pipefail

NEED_PRIO="90"
BIN="./aimrt_main"

usage() {
    cat <<'EOF'
用法: rt_perm_check.sh [--need-prio N] [--bin <path>]
  --need-prio  需要的最高实时优先级（默认 90）
  --bin        目标二进制（默认 ./aimrt_main）
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --need-prio) NEED_PRIO="${2:?--need-prio 需要参数}"; shift 2;;
        --bin) BIN="${2:?--bin 需要参数}"; shift 2;;
        -h|--help) usage; exit 0;;
        *) echo "[perm_check] 未知参数: $1（见 --help）" >&2; exit 64;;
    esac
done

G=$'\033[32m'; Y=$'\033[33m'; R=$'\033[31m'; B=$'\033[36m'; N=$'\033[0m'
pok(){ echo "${B}[perm_check]${N} ${G}✓${N} $*"; }
png(){ echo "${B}[perm_check]${N} ${R}✗${N} $*"; }
pinfo(){ echo "${B}[perm_check]${N} $*"; }

# ---- 判据① root ----
if [ "$(id -u)" -eq 0 ]; then
    pok "判据① root 直通 —— 天然有资格设置实时调度"
    exit 0
fi

# ---- 判据② 文件能力 cap_sys_nice ----
if command -v getcap >/dev/null 2>&1 && getcap "$BIN" 2>/dev/null | grep -q "cap_sys_nice"; then
    pok "判据② ${BIN} 携带 cap_sys_nice —— 文件能力路径"
    exit 0
fi

# ---- 判据③ 实时优先级限额（RLIMIT_RTPRIO） ----
RTPRIO="$(ulimit -r)"
case "$RTPRIO" in
    unlimited)
        pok "判据③ 实时优先级限额 unlimited"
        exit 0
        ;;
    ''|*[!0-9]*)
        ;;  # 非数字（异常值），落到失败分支
    *)
        if [ "$RTPRIO" -ge "$NEED_PRIO" ]; then
            pok "判据③ 实时优先级限额 ${RTPRIO} ≥ ${NEED_PRIO}"
            exit 0
        fi
        ;;
esac

# ---- 全部判据未命中：拦截 + 诊断 + 补救 ----
png "当前用户 $(id -un) 无法申请实时调度（限额 ${RTPRIO:-读取失败}，需 ≥ ${NEED_PRIO}）"
pinfo "  后果: 线程将退化为普通调度（假实时）——C++ 侧失败只打 ERROR 不退出，性能静默劣化"
pinfo "  诊断: 用户=$(id -un)（非 root）｜ getcap ${BIN}=$(getcap "$BIN" 2>/dev/null || echo '无') ｜ ulimit -r=${RTPRIO:-?}"

# 专属提示：limits.d 已配置但本会话没吃到（限额在登录时发放，改文件不影响已登录会话）
if grep -qs "^$(id -un) " /etc/security/limits.d/*.conf 2>/dev/null; then
    echo "${B}[perm_check]${N} ${Y}⚠ 检测到 limits.d 已为 $(id -un) 配置过限额，但当前会话没吃到 —— 请【退出并重新登录】后再启动${N}"
fi

pinfo "补救（二选一）:"
pinfo "  ① 手柄模式: 换 sudo 启动（root 天然有资格，现场习惯不变）"
pinfo "  ② 联合模式: 管理员执行下面命令后【重新登录】:"
pinfo "       printf '%s soft rtprio 99\\n%s hard rtprio 99\\n' '$(id -un)' '$(id -un)' | sudo tee /etc/security/limits.d/$(id -un)-rt.conf"
exit 1
