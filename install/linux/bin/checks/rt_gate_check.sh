#!/bin/bash
# rt_gate_check.sh — 环境门禁仪器（status 凭证两步判定 + 网口/CPU 活体检查）
# -----------------------------------------------------------------------------
# 换芯自 rt_env_check.sh：原"环境快档"的
# ①实时内核 ②隔离车道两项静态检查，交给开机自检产出的 status 凭证
# （f1-rt-verify.service 开机自动跑 verify-rt-config.sh --report 写
# /run/f1-rt-verify/——/run 是 tmpfs，每次开机清空，存在的必是本次开机写的）。
# 本仪器保留报告管不了的【运行时活状态】两项（网口 UP / bind CPU 在线），
# 并新增 status 两步判定。
#
# 被 run_main.sh 以参数调用（参数从 cfg 读）；也可单独运行排查——
# 门禁被拦时手敲本仪器即可复现完整判定链（status 在不在 / RESULT / 网口 / CPU）。
#
# 四步判定（任何一步失败即拦截）：
#   ① status 文件存在 —— 不存在 = 开机自检没跑成（service 未装/失败/被禁）
#   ② status 的 RESULT 行以 PASS 开头 —— PASS / PASS_WITH_EXPECTED 均放行；
#      FAIL 时看同目录 report.txt 全文找失败项
#   ③ EtherCAT 网口 UP —— 查链路（验收时插着线 ≠ 现在插着；对端 DCU 断电也在此现形）
#   ④ bind CPU 在线 —— 防运行期被管理员下线
#
# 用法: rt_gate_check.sh --status <path> --nic <ifname> --bind-cpus <c1,c2,...>
#   --status      status 凭证路径（f1-rt-verify.service 产出，编排者传
#                 /run/f1-rt-verify/status）
#   --nic         EtherCAT 网卡名（cfg: ethercat.ifname）
#   --bind-cpus   命脉线程绑定的 CPU，逗号分隔（cfg: ethercat.bind_cpu + bind_cpu）
# 退出码: 0=全部通过  1=有失败项  64=用法错

set -uo pipefail

STATUS=""
NIC=""
BIND_CPUS=""

usage() {
    cat <<'EOF'
用法: rt_gate_check.sh --status <path> --nic <ifname> --bind-cpus <c1,c2,...>
  --status      status 凭证路径（/run/f1-rt-verify/status）
  --nic         EtherCAT 网卡名（cfg: ethercat.ifname）
  --bind-cpus   命脉线程绑定的 CPU，逗号分隔（cfg: ethercat.bind_cpu + bind_cpu）
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --status) STATUS="${2:?--status 需要参数}"; shift 2;;
        --nic) NIC="${2:?--nic 需要参数}"; shift 2;;
        --bind-cpus) BIND_CPUS="${2:?--bind-cpus 需要参数}"; shift 2;;
        -h|--help) usage; exit 0;;
        *) echo "[gate_check] 未知参数: $1（见 --help）" >&2; exit 64;;
    esac
done

[ -n "$STATUS" ] || { echo "[gate_check] 缺少 --status（f1-rt-verify.service 凭证路径）" >&2; exit 64; }
[ -n "$NIC" ] || { echo "[gate_check] 缺少 --nic（cfg ethercat.ifname）" >&2; exit 64; }
[ -n "$BIND_CPUS" ] || { echo "[gate_check] 缺少 --bind-cpus（cfg bind_cpu）" >&2; exit 64; }

G=$'\033[32m'; Y=$'\033[33m'; R=$'\033[31m'; B=$'\033[36m'; N=$'\033[0m'
gok(){ echo "${B}[gate_check]${N} ${G}✓${N} $*"; }
gng(){ echo "${B}[gate_check]${N} ${R}✗${N} $*"; }

CPUS="$(echo "$BIND_CPUS" | tr ',' ' ')"
fail=0

# ---- ① status 凭证存在 ----
if [ -f "$STATUS" ]; then
    gok "验收凭证存在（$STATUS）"
else
    gng "验收凭证不存在（$STATUS）—— 开机自检没跑成（service 未安装/执行失败/被禁）"
    gng "  排查: journalctl -u f1-rt-verify.service -b"
    gng "  补跑: sudo systemctl start f1-rt-verify.service"
    fail=1
fi

# ---- ② RESULT 判定（仅凭证存在时读） ----
if [ -f "$STATUS" ]; then
    RESULT_LINE="$(grep -m1 '^RESULT:' "$STATUS" 2>/dev/null | tr -d '\r')"
    case "$RESULT_LINE" in
        RESULT:\ PASS*)
            gok "验收凭证: ${RESULT_LINE#RESULT: }（环境合格）"
            ;;
        "")
            gng "status 缺 RESULT 行（文件损坏？）—— 看 $STATUS 内容"
            fail=1
            ;;
        *)
            gng "验收凭证: ${RESULT_LINE#RESULT: } —— 本次开机验收存在未通过项"
            gng "  看全文: $(dirname "$STATUS")/report.txt"
            fail=1
            ;;
    esac
fi

# ---- ③ EtherCAT 网口 UP（活体：查链路，验收时插着线 ≠ 现在插着） ----
if ip link show "$NIC" >/dev/null 2>&1; then
    if ip link show "$NIC" 2>/dev/null | grep -q "state UP"; then
        gok "EtherCAT 网口 ${NIC} 存在且 UP（链路通）"
    else
        gng "EtherCAT 网口 ${NIC} 存在但非 UP（$(ip link show "$NIC" 2>/dev/null | grep -o 'state [A-Z]*' || echo '状态未知')）—— 检查网线与对端 DCU"
        fail=1
    fi
else
    gng "EtherCAT 网口 ${NIC} 不存在（cfg ethercat.ifname=${NIC}；用 ip link 确认实际网卡名）"
    fail=1
fi

# ---- ④ bind CPU 在线（活体：防运行期被下线） ----
offline_bad=""
for c in $CPUS; do
    [ -n "$c" ] || continue
    on="/sys/devices/system/cpu/cpu${c}/online"
    # 无 online 文件的 CPU（如 cpu0）视为恒在线
    if [ -f "$on" ] && [ "$(cat "$on" 2>/dev/null)" != "1" ]; then
        offline_bad="$offline_bad $c"
    fi
done
if [ -z "$offline_bad" ]; then
    gok "bind CPU 在线（${CPUS}）"
else
    gng "bind CPU 不在线:${offline_bad} —— 线程将绑到不存在的核上"
    fail=1
fi

if [ "$fail" -eq 0 ]; then
    echo "${B}[gate_check]${N} ${G}✓ 环境门禁通过（凭证 PASS + 网口 UP + CPU 在线）${N}"
    exit 0
else
    echo "${B}[gate_check]${N} ${R}✗ 环境门禁存在失败项（见上）${N}"
    exit 1
fi
