#!/bin/bash
# 机器人主启动脚本（手动执行；手柄模式习惯 sudo，联合模式非 sudo，身份规则见 README.md）
# 职责：身份检查 → preflight 三路预检 → source ROS2 → 后台拉起 aimrt_main
#       → 启动后 RT 验证 → trap 信号/退出回收（含下使能兜底逻辑）
#
# preflight 三路（任何一路不过即拒绝上电，无逃生阀；排障请绕过启动器直跑 aimrt_main）：
#   程序域    本脚本自检：残留进程 / cfg 可解析 / 三件套 / ONNX / ROS2 source
#   RT 资格   checks/rt_perm_check.sh —— 当前用户能否设置实时调度
#             （root / 文件 cap_sys_nice / rtprio 限额三判据，阈值从 cfg 读）
#   环境门禁  checks/rt_gate_check.sh —— 读 /run/f1-rt-verify/status（开机自检
#             f1-rt-verify.service 跑 verify --report 产出的凭证：两步判定
#             PASS/FAIL）+ 网口 UP / bind CPU 在线两项活体检查，参数从 cfg 读
#   全量环境核验不在启动时做 —— 由 checks/f1-rt-verify.service 每次开机自动
#   验收并产出 /run/ 凭证（安装法见 README.md）；本脚本的门禁读该凭证。
#
# 参照 install/linux/bin/run.sh 的部署逻辑：脚本与 aimrt_main/cfg/libpkg1.so 同处
# build 部署目录，编译时由源码 install/linux/bin/ 复制而来，运行时 cwd=该 build 目录，
# 全部用 ./ 相对路径（与 run.sh 一致，不 cd）。请先 cd 到 build 目录再执行。
#
# 用法: bash run_main.sh [--check-only] [--cfg <顶层cfg>]
#   --check-only   只跑 preflight 不启动（排障 / 为将来 systemd ExecStartPre 预留）
#   --cfg <path>   顶层 cfg，默认 ./cfg/x1_cfg.yaml
# 环境变量可覆盖：SHUTDOWN_TIMEOUT(30) RT_VERIFY_FAIL_ACTION(exit|warn) VERIFY_TIMEOUT(15)

set -uo pipefail

# ===== 参数 =====
CHECK_ONLY=0
CFG="${CFG:-./cfg/x1_cfg.yaml}"
while [ $# -gt 0 ]; do
    case "$1" in
        --check-only) CHECK_ONLY=1; shift;;
        --cfg) CFG="${2:?--cfg 需要参数}"; shift 2;;
        *) echo "未知参数: $1" >&2; exit 64;;
    esac
done

# ===== 常量 =====
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECKS_DIR="${HERE}/checks"
HELPER="${CHECKS_DIR}/cfg_helper.py"              # yaml 查询小工具（检查家族共用）
RT_VERIFY="${CHECKS_DIR}/rt_verify.sh"            # 启动后线程级 RT 验证（假实时实证）
RT_PERM_CHECK="${CHECKS_DIR}/rt_perm_check.sh"    # 启动前 RT 权限资格预检
RT_GATE_CHECK="${CHECKS_DIR}/rt_gate_check.sh"    # 环境门禁（status 凭证 + 网口/CPU 活体）
RT_STATUS_PATH="/run/f1-rt-verify/status"         # 开机自检产出的验收凭证
SHUTDOWN_TIMEOUT="${SHUTDOWN_TIMEOUT:-30}"
RT_VERIFY_FAIL_ACTION="${RT_VERIFY_FAIL_ACTION:-exit}"
VERIFY_TIMEOUT="${VERIFY_TIMEOUT:-15}"
AIMRT_PID=""

G=$'\033[32m'; Y=$'\033[33m'; R=$'\033[31m'; B=$'\033[36m'; N=$'\033[0m'
log(){ echo "${B}[run_main]${N} $*"; }
ok(){ echo "  ${G}✓${N} $*"; }
ng(){ echo "  ${R}✗${N} $*"; }

# ===== 信号/退出回收（文档 4.6）=====
# 脚本是 aimrt_main 的父进程。终止信号到来时 trap 先接管，向 aimrt_main 转发 SIGTERM
# （不是 SIGKILL）——这一步是灵魂：只有 aimrt_main 走正常 Shutdown，DcuDriverModule::Shutdown()
# 才会执行 DisableAllActuator()（逐执行器下发 STATE_DISABLE 并轮询确认）→ Stop()（停 EtherCAT、
# 从站进 INIT）。电机的下使能就发生在这一步。脚本只做进程级的事（转发→等→超时强杀→回收），
# 不做业务级的事（不直接发模式指令、不直接控电机）——后者必须经 EtherCAT→DCU→CAN 通道，
# 只有 aimrt_main 活着才能做，shell 在进程外做不了。超时强杀时电机可能仍使能→告警人工断电。
cleanup() {
    trap '' SIGINT SIGTERM SIGHUP
    if [ -z "$AIMRT_PID" ] || ! kill -0 "$AIMRT_PID" 2>/dev/null; then
        exit 0
    fi
    log "收到终止信号，向 aimrt_main($AIMRT_PID) 转发 SIGTERM（给 Shutdown 下使能电机的机会）"
    kill -TERM "$AIMRT_PID" 2>/dev/null
    ( sleep "$SHUTDOWN_TIMEOUT" && kill -KILL "$AIMRT_PID" 2>/dev/null ) &
    local killer=$!
    wait "$AIMRT_PID" 2>/dev/null
    local ec=$?
    kill "$killer" 2>/dev/null; wait "$killer" 2>/dev/null
    if [ "$ec" -eq 137 ]; then
        echo "${R}[run_main] aimrt_main 被 SIGKILL 强杀（超时），电机可能未下使能，请人工断电确认！${N}"
    else
        log "aimrt_main 退出，code=$ec"
    fi
    exit "$ec"
}

# ===== 颈部舵机下使能兜底（EXIT trap，覆盖所有退出路径）=====
# cleanup() 末尾的 exit "$ec" 会触发本 trap 自动执行 release_neck；信号 cleanup /
# preflight 失败 exit / --check-only exit / aimrt_main 自然退出 exit 均覆盖。
# _neck_enabled 守卫：未使能过则不下使能，避免误操作串口。
release_neck() {
    [ "${_neck_enabled:-0}" = 1 ] || return
    if [ -e "${NECK_PORT:-}" ]; then
        timeout 5 python3 -c "import sys; sys.path.insert(0, '${HERE}'); from htd35h_controller import HTD35HController; import time; c=HTD35HController(port='${NECK_PORT}', baudrate=115200); c.enable_torque_all([1,2], False); time.sleep(0.3); c.close()" 2>/dev/null
        log "颈部舵机已下使能"
    fi
}
trap release_neck EXIT

# ===== 1. 身份检查（兼容 root / 非 root）=====
IS_ROOT=0
if [ "$(id -u)" -eq 0 ]; then IS_ROOT=1; ok "[1] root 身份（免 setcap）"; else ok "[1] 非 root 身份（将检查 setcap）"; fi

# ===== 2. preflight 三路预检 =====
log "preflight 检查（cfg=$CFG）"
preflight_fail=0

# ---- 2a. 程序域 ----
if [ -z "$(pidof aimrt_main 2>/dev/null)" ]; then ok "[2] 无残留 aimrt_main"; else ng "[2] 残留 aimrt_main（$(pidof aimrt_main)）请先停止"; preflight_fail=1; fi

RL_CFG=""; DCU_CFG=""
if [ -f "$CFG" ] && python3 "$HELPER" module-cfg "$CFG" ControlModule >/dev/null 2>&1 && python3 "$HELPER" module-cfg "$CFG" DcuDriverModule >/dev/null 2>&1; then
    RL_CFG="$(python3 "$HELPER" module-cfg "$CFG" ControlModule 2>/dev/null)"
    DCU_CFG="$(python3 "$HELPER" module-cfg "$CFG" DcuDriverModule 2>/dev/null)"
    ok "[3] 顶层 cfg 可解析 → Control:$RL_CFG DcuDriver:$DCU_CFG"
else
    ng "[3] 顶层 cfg 解析失败（$CFG）"; preflight_fail=1
fi

miss_bin=""
for b in aimrt_main libpkg1.so libaimrt_ros2_plugin.so; do
    [ -f "./$b" ] || miss_bin="$miss_bin $b"
done
if [ -z "$miss_bin" ]; then ok "[4] 三件套二进制就位"; else ng "[4] 缺失二进制:$miss_bin（请确认 cwd=build 部署目录）"; preflight_fail=1; fi

# ---- 2b. RT 资格预检（资格仪器，阈值与参数从 cfg 读） ----
if [ -n "$DCU_CFG" ] && [ -n "$RL_CFG" ]; then
    # 需求阈值 = 两条命脉线程优先级的最大值（ecat ethercat.rt_priority / ctrl rt_priority）
    NEED_RT=""
    for p in "$(python3 "$HELPER" get "$DCU_CFG" ethercat.rt_priority 2>/dev/null)" \
             "$(python3 "$HELPER" get "$RL_CFG" rt_priority 2>/dev/null)"; do
        case "$p" in
            ''|*[!0-9]*) ;;  # 空或非数字跳过
            *) if [ -z "$NEED_RT" ] || [ "$p" -gt "$NEED_RT" ]; then NEED_RT="$p"; fi ;;
        esac
    done
    [ -n "$NEED_RT" ] || NEED_RT=90   # cfg 读不到时保守取 90（当前配置即 90/80）

    log "RT 权限预检（rt_perm_check.sh，需实时优先级 ≥ ${NEED_RT}）"
    if bash "$RT_PERM_CHECK" --need-prio "$NEED_RT" --bin ./aimrt_main; then
        ok "[5] RT 权限预检通过"
    else
        ng "[5] RT 权限预检未通过 —— 当前用户无法设置实时调度（补救指引见上方仪器输出）"
        preflight_fail=1
    fi
else
    ng "[5] RT 权限预检跳过（cfg 未定位）"; preflight_fail=1
fi

# ---- 2c. 环境门禁（仪器：rt_gate_check.sh，参数从 cfg 读） ----
if [ -n "$DCU_CFG" ] && [ -n "$RL_CFG" ]; then
    IFNAME="$(python3 "$HELPER" get "$DCU_CFG" ethercat.ifname 2>/dev/null)"
    ECAT_CPU="$(python3 "$HELPER" get "$DCU_CFG" ethercat.bind_cpu 2>/dev/null)"
    CTRL_CPU="$(python3 "$HELPER" get "$RL_CFG" bind_cpu 2>/dev/null)"
    if [ -n "$IFNAME" ] && [ -n "$ECAT_CPU" ] && [ -n "$CTRL_CPU" ]; then
        log "环境门禁检查（rt_gate_check.sh，status=${RT_STATUS_PATH}，nic=${IFNAME}，bind_cpu=${ECAT_CPU},${CTRL_CPU}）"
        if bash "$RT_GATE_CHECK" --status "$RT_STATUS_PATH" --nic "$IFNAME" --bind-cpus "${ECAT_CPU},${CTRL_CPU}"; then
            ok "[6] 环境门禁通过（status 判定 + 网口 UP + CPU 在线）"
        else
            ng "[6] 环境门禁未通过 —— 凭证/现场异常（详见上方仪器输出）"
            preflight_fail=1
        fi
    else
        ng "[6] 环境门禁跳过（cfg 字段缺失：ifname=${IFNAME:-未读到} ecat_cpu=${ECAT_CPU:-未读到} ctrl_cpu=${CTRL_CPU:-未读到}）"
        preflight_fail=1
    fi
else
    ng "[6] 环境门禁跳过（cfg 未定位）"; preflight_fail=1
fi

# ---- 2d. 程序域（续） ----
if [ -n "$RL_CFG" ]; then
    onnx_miss=""
    while IFS= read -r m; do
        [ -n "$m" ] || continue
        [ -f "$m" ] || onnx_miss="$onnx_miss $m"
    done < <(python3 "$HELPER" onnx-models "$RL_CFG" 2>/dev/null)
    if [ -z "$onnx_miss" ]; then ok "[7] ONNX 模型就位"; else ng "[7] 缺失 ONNX:$onnx_miss"; preflight_fail=1; fi
else
    ng "[7] 跳过（RL_CFG 未定位）"; preflight_fail=1
fi

if [ -f "${HERE}/ros2_source.sh" ]; then
    if ( source "${HERE}/ros2_source.sh" ) >/dev/null 2>&1; then
        source "${HERE}/ros2_source.sh"
        ok "[8] ROS2 source 成功"
    else
        ng "[8] ROS2 source 失败（ros2_source.sh 找不到 ROS2）"; preflight_fail=1
    fi
else
    ng "[8] 找不到 ros2_source.sh"; preflight_fail=1
fi

if [ "$preflight_fail" -ne 0 ]; then
    echo "${R}[run_main] preflight 未通过，拒绝启动${N}"; exit 1
fi

# ===== 3. local_setup（沿用 run.sh / ros2_setup.sh 三候选）=====
# set -u 兼容（2026-10-21 老 minipc 实测修复）：ament 生成的 local_setup.bash
# 引用未定义变量（AMENT_TRACE_SETUP_FILES 等）——nounset 下 sourcing 即中止。
# ROS2 标准做法：source 期间临时关 -u，完毕恢复。
# （朴素版 run.sh 无 set -u 故从未触发；285H 非 sudo 场景环境变量已就位躲过）
set +u
if [ -f ./build/install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./build/install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ./install/share/ros2_plugin_proto/local_setup.bash ]; then
    source ./install/share/ros2_plugin_proto/local_setup.bash
elif [ -f ../share/ros2_plugin_proto/local_setup.bash ]; then
    source ../share/ros2_plugin_proto/local_setup.bash
fi
set -u

if [ "$CHECK_ONLY" = 1 ]; then
    log "--check-only：preflight 全过，不启动"; exit 0
fi

# ===== 颈部双舵机使能（小脑运行期间保持不动）=====
# 串口可用环境变量覆盖: NECK_SERVO_PORT
# 默认失败仅告警放行; 设 NECK_SERVO_REQUIRED=1 则使能失败中止启动
NECK_PORT="${NECK_SERVO_PORT:-/dev/ttyUSB0}"
_neck_enabled=0
if [ -e "${NECK_PORT}" ]; then
    timeout 10 python3 "${HERE}/neck_hold.py" --port "${NECK_PORT}"
    neck_rc=$?
    if [ "${neck_rc}" -ne 0 ]; then
        if [ "${NECK_SERVO_REQUIRED:-0}" = "1" ]; then
            ng "颈部舵机使能失败 (rc=${neck_rc}), NECK_SERVO_REQUIRED=1, 中止"; exit 1
        fi
        echo "${Y}[run_main][WARN] 颈部使能失败 (rc=${neck_rc}), 继续 —— 头部可能松动!${N}"
    else
        ok "颈部舵机使能"; _neck_enabled=1
    fi
else
    echo "${Y}[run_main][WARN] 未发现颈部串口 ${NECK_PORT}, 跳过使能${N}"
fi

# ===== setcap（EtherCAT 原始套接字权限，须在 aimrt_main 启动前）=====
# root 免（默认有 CAP_NET_RAW）；非 root 时 getcap 先查→已就绪跳过→否则 sudo setcap。
# tmux 里 sudo 可能卡密码，故优先 getcap 跳过；root 运行或已 setcap 过则全程不触密码。
if [ "$IS_ROOT" = 1 ]; then
    ok "setcap: root 免（默认有 CAP_NET_RAW）"
elif command -v getcap >/dev/null 2>&1 && getcap ./aimrt_main 2>/dev/null | grep -q "cap_net_raw"; then
    ok "setcap: cap_net_raw 已就绪（跳过）"
else
    echo "${Y}[run_main] cap_net_raw 未就绪, 尝试 sudo setcap（tmux 里可能卡密码）...${N}"
    if sudo setcap cap_net_raw=ep ./aimrt_main; then ok "setcap 完成"; else
        echo "${Y}[run_main][WARN] setcap 失败 —— EtherCAT 可能无法通信${N}"
    fi
fi

# ===== 4. 后台拉起 aimrt_main =====
log "启动 aimrt_main（cfg=$CFG，后台运行；日志见终端与 ./log/x1_rl_ctrl.log）"
./aimrt_main --cfg_file_path="$CFG" &
AIMRT_PID=$!
log "aimrt_main PID=$AIMRT_PID"

# ===== 5. trap 信号（覆盖 RT 验证与 wait 全程）=====
trap cleanup SIGINT SIGTERM SIGHUP

# ===== 6. 启动后 RT 验证（文档 4.2）=====
log "启动后 RT 验证（轮询最多 ${VERIFY_TIMEOUT}s）"
if bash "$RT_VERIFY" "$AIMRT_PID" "$DCU_CFG" "$RL_CFG"; then
    ok "RT 验证通过"
else
    echo "${R}[run_main] RT 验证失败${N}"
    if [ "$RT_VERIFY_FAIL_ACTION" = "exit" ]; then
        log "RT_VERIFY_FAIL_ACTION=exit → cleanup 停止 aimrt_main"
        cleanup
    else
        echo "${Y}[run_main] RT_VERIFY_FAIL_ACTION=warn → 仅告警，继续运行${N}"
    fi
fi

# ===== 7. 前台等待 =====
log "aimrt_main 运行中，Ctrl+C 或 kill -TERM $AIMRT_PID 终止"
wait "$AIMRT_PID" 2>/dev/null
ec=$?
log "aimrt_main 自然退出，code=$ec"
exit "$ec"
