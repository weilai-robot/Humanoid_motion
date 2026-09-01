#!/usr/bin/env bash
# =============================================================================
# F1 minipc 实时环境验证脚本 (配合 f1-rt-config-merged.sh)
# 覆盖 md §8 验收标准 + 交叉评估新增项（cpufreq/boost, default_smp_affinity, ethtool）
# 只读不改系统。重启前后均可跑：未重启的运行时项标 WARN，值错标 FAIL。
#
# 本版相对原稿的修正：
#   1) 【致命 bug】pass/warn/fail 三个函数里 "$(c_grn PASS')" 多了一个单引号，
#      导致引号未闭合，整个脚本从该行往后被吞进一个字符串，bash -n 直接报语法错误，
#      脚本根本跑不起来。已修正为 "$(c_grn 'PASS')"。
#   2) sect() 用 c_cyn "...\n" 想换行，但 \n 是作为 printf 的数据(%s)传入，
#      不会被解释成换行，只会原样打印 "\n" 两个字符。已改为在格式串里换行。
#   3) 同步 f1-rt-config-merged.sh 的变更：GRUB 参数由 11 个增加到 12 个
#      （新增 irqaffinity=${HK_CPUS}），检查列表和提示文案同步更新。
#   4) 新增：显式校验 xHCI/r8152 中断亲和性里【不包含】隔离核 4-7
#      （不只是检查"绑没绑 CPU8"，因为原文档 echo 256 的 bug 恰恰是
#       CPU8 检查会不通过、但更危险的"污染 4/6"这件事本身没被专门校验出来）。
#   5) 新增：校验 GRUB drop-in 里【不含】mce=off（防止以后被误加回来）。
#   6) 新增：ethtool autoneg 当前状态仅作提示，不计入 PASS/FAIL
#      （对应 config 脚本里默认关闭、需现场验证的 ENABLE_AUTONEG_OFF 开关）。
# =============================================================================

set -uo pipefail

# ---------- 常量（与 f1-rt-config-merged.sh 一致）----------
readonly TAG="f1-rt"
readonly ISOL_CPUS="4,5,6,7"
readonly HK_CPUS="0-3,8-19"
readonly NIC_ECAT="enp2s0"
readonly GRUB_DROPIN="/etc/default/grub.d/99-${TAG}.cfg"
readonly SYSCTL_DROPIN="/etc/sysctl.d/99-${TAG}.conf"
readonly RT_ENV_SCRIPT="/opt/f1/scripts/run/set_rt_env.sh"
readonly RT_SETUP_SVC="f1-rt-setup.service"
readonly THP_SVC="f1-rt-disable-thp.service"
readonly UDEV_GOVERNOR="/etc/udev/rules.d/99-performance-governor.rules"

PASS=0; WARN=0; FAIL=0

# ---------- helpers ----------
c_grn() { printf '\033[32m%s\033[0m' "$*"; }
c_red() { printf '\033[31m%s\033[0m' "$*"; }
c_ylw() { printf '\033[33m%s\033[0m' "$*"; }
c_cyn() { printf '\033[36m%s\033[0m' "$*"; }

pass() { printf '  [%s] %s\n' "$(c_grn 'PASS')" "$*"; PASS=$((PASS+1)); }
warn() { printf '  [%s] %s\n' "$(c_ylw 'WARN')" "$*"; WARN=$((WARN+1)); }
fail() { printf '  [%s] %s\n' "$(c_red 'FAIL')" "$*"; FAIL=$((FAIL+1)); }
sect() { printf '\n%s\n' "$(c_cyn "=== $* ===")"; }

# 判断 CPU 是否在掩码列表里（smp_affinity_list 格式：逗号/连字符，如 "0,8-10"）
cpu_in_list() {
    local cpu="$1" list="$2"
    # 展开为单 CPU 集合
    local set=""
    IFS=',' read -ra parts <<<"$list"
    for p in "${parts[@]}"; do
        p="${p// /}"
        if [[ "$p" == *-* ]]; then
            local a b; IFS='-' read -r a b <<<"$p"
            set+=" $(seq "$a" "$b")"
        else
            set+=" $p"
        fi
    done
    for c in $set; do [[ "$c" == "$cpu" ]] && return 0; done
    return 1
}

# ---------- 1. 内核层 ----------
check_kernel() {
    sect "内核层 (md §8.1)"

    # PREEMPT_RT
    if [[ "$(cat /sys/kernel/realtime 2>/dev/null)" == "1" ]] || uname -v | grep -qi 'PREEMPT RT'; then
        pass "PREEMPT_RT 内核 ($(uname -r))"
    else
        fail "非 PREEMPT_RT 内核 ($(uname -r))，需装 5.15.0-1100-realtime"
    fi

    # GRUB 参数：文件层 + 运行时层（12 项，含 irqaffinity）
    local cmdline; cmdline="$(cat /proc/cmdline 2>/dev/null)"
    local need_reboot=0
    for kv in "isolcpus=${ISOL_CPUS}" "nohz_full=${ISOL_CPUS}" "rcu_nocbs=${ISOL_CPUS}" \
              "irqaffinity=${HK_CPUS}" \
              "intel_pstate=disable" "processor.max_cstate=0" "intel_idle.max_cstate=0" \
              "watchdog_thresh=60" "audit=0" "tsc=reliable" "clocksource=tsc" \
              "transparent_hugepage=never"; do
        if [[ -f "$GRUB_DROPIN" ]] && grep -qF "$kv" "$GRUB_DROPIN"; then
            : # 文件层 OK
        else
            fail "GRUB drop-in 缺少参数: $kv"
        fi
        if [[ -n "$cmdline" ]] && echo " $cmdline " | grep -qF " $kv "; then
            : # 运行时 OK
        else
            need_reboot=1
        fi
    done
    if [[ $need_reboot -eq 1 ]]; then
        warn "GRUB 参数文件已写但 /proc/cmdline 未生效——需重启"
    else
        pass "GRUB 全部 12 参数运行时生效"
    fi

    # 防回归：确认 mce=off 没被误加回来（执行器安全系统不应关闭硬件故障上报）
    if [[ -f "$GRUB_DROPIN" ]] && grep -qF "mce=off" "$GRUB_DROPIN"; then
        fail "GRUB drop-in 含 mce=off——会丢失硬件故障(ECC/总线/过热)上报，执行器系统不应关闭"
    else
        pass "GRUB drop-in 未包含 mce=off (符合预期)"
    fi

    # 隔离核无普通线程 (md §8.1)
    local nonrt
    nonrt="$(ps -eo psr,cls,comm 2>/dev/null | awk '$1>=4 && $1<=7 && $2!="FF" && $2!="" {print}')"
    if [[ -z "$nonrt" ]]; then
        pass "隔离核 CPU 4-7 无非实时线程"
    else
        fail "隔离核 CPU 4-7 存在非实时线程:"; echo "$nonrt" | head -5 | sed 's/^/      /'
    fi
}

# ---------- 2. OS 层 ----------
check_os() {
    sect "OS 层 (md §8.2 + 交叉评估新增)"

    # governor 全核 performance
    local bad_gov=""
    for g in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
        local v; v="$(cat "$g" 2>/dev/null)"
        [[ "$v" == "performance" ]] || bad_gov+="$(basename "$(dirname "$(dirname "$g")")")=$v "
    done
    if [[ -z "$bad_gov" ]]; then
        pass "所有 CPU governor = performance"
    else
        fail "部分 CPU governor 非 performance: $bad_gov"
    fi

    # cpufreq/boost=0 (禁用睿频，交叉评估修正项)
    if [[ -f /sys/devices/system/cpu/cpufreq/boost ]]; then
        local boost; boost="$(cat /sys/devices/system/cpu/cpufreq/boost 2>/dev/null)"
        if [[ "$boost" == "0" ]]; then
            pass "cpufreq/boost=0 (睿频已禁用，ACPI 路径)"
        else
            fail "cpufreq/boost=$boost (睿频未禁用)"
        fi
    else
        warn "/sys/devices/system/cpu/cpufreq/boost 不存在 (intel_pstate 未 disable 或驱动差异)"
    fi

    # THP = never
    local thp; thp="$(cat /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null | grep -o '\[never\]')"
    if [[ -n "$thp" ]]; then
        pass "THP = never"
    else
        fail "THP 非 never: $(cat /sys/kernel/mm/transparent_hugepage/enabled 2>/dev/null)"
    fi

    # swap 关闭
    if [[ -z "$(swapon --show 2>/dev/null)" ]]; then
        pass "Swap 已关闭"
    else
        fail "Swap 仍激活"; swapon --show 2>/dev/null | sed 's/^/      /'
    fi

    # sysctl
    if [[ -f "$SYSCTL_DROPIN" ]]; then
        pass "sysctl drop-in 存在"
        local s_rt; s_rt="$(sysctl -n kernel.sched_rt_runtime_us 2>/dev/null)"
        [[ "$s_rt" == "-1" ]] && pass "sched_rt_runtime_us = -1 (RT 节流已解除)" \
            || fail "sched_rt_runtime_us = $s_rt (期望 -1)"
        local tm; tm="$(sysctl -n kernel.timer_migration 2>/dev/null)"
        [[ "$tm" == "0" ]] && pass "timer_migration = 0" || fail "timer_migration = $tm (期望 0)"
        local nb; nb="$(sysctl -n kernel.numa_balancing 2>/dev/null)"
        [[ "$nb" == "0" ]] && pass "numa_balancing = 0" || fail "numa_balancing = $nb (期望 0)"
    else
        fail "sysctl drop-in 不存在: $SYSCTL_DROPIN"
    fi

    # irqbalance 停用 + mask
    local irqs; irqs="$(systemctl is-active irqbalance 2>/dev/null)"
    if [[ "$irqs" == "inactive" || "$irqs" == "masked" || "$irqs" == "disabled" ]]; then
        pass "irqbalance 已停用 ($irqs)"
    else
        fail "irqbalance 活动中 ($irqs)，将绕过 isolcpus 重路由 IRQ"
    fi

    # 注：RT 权限（rtprio/memlock limits）已取消——控制服务以 root (sudo) 运行，
    #     pthread_setschedparam/mlockall 在 root 下不受 PAM rtprio/memlock limit 约束，
    #     无需为非 root 用户配 limits（f1-rt-config-claude-merge.sh 不再写 limits drop-in）。
}

# ---------- 3. IRQ 亲和性 ----------
check_irq() {
    sect "IRQ 亲和性 (md §3.3/§3.4 + default_smp_affinity 兜底)"

    # default_smp_affinity 不含 4-7
    local def; def="$(cat /proc/irq/default_smp_affinity 2>/dev/null)"
    local def_list; def_list="$(cat /proc/irq/default_smp_affinity_list 2>/dev/null)"
    local def_bad=0
    for c in 4 5 6 7; do
        if cpu_in_list "$c" "$def_list"; then def_bad=1; fi
    done
    if [[ $def_bad -eq 0 ]]; then
        pass "default_smp_affinity 不含实时核 4-7 (mask=$def)"
    else
        fail "default_smp_affinity 含实时核 ($def_list)，热插拔 IRQ 将污染 4-7"
    fi

    # NVMe 队列中断 → CPU 0
    local nvme_irqs; nvme_irqs="$(grep nvme /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' ')"
    if [[ -z "$nvme_irqs" ]]; then
        warn "/proc/interrupts 无 nvme 条目 (NVMe 未加载?)"
    else
        local nvme_ok=1
        for n in $nvme_irqs; do
            local alist; alist="$(cat /proc/irq/$n/effective_affinity_list 2>/dev/null || cat /proc/irq/$n/smp_affinity_list 2>/dev/null)"
            if cpu_in_list 4 "$alist" || cpu_in_list 5 "$alist" || cpu_in_list 6 "$alist" || cpu_in_list 7 "$alist"; then
                fail "NVMe IRQ $n 落实时核 ($alist)"
                nvme_ok=0
            fi
        done
        [[ $nvme_ok -eq 1 ]] && pass "NVMe 队列中断全部移出实时核 4-7"
    fi

    # xHCI/r8152 → CPU 8，且【不得】落在隔离核 4-7
    # 说明：只查"是否绑到 CPU8"不够——原文档 echo 256 的十六进制 bug
    # 实际会把亲和性打到 CPU 1/2/4/6/9，其中 4/6 恰好是隔离核，
    # 这才是真正危险的后果，必须单独显式校验。
    local xhci_irqs; xhci_irqs="$(grep -iE 'xhci|r8152' /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' ')"
    if [[ -z "$xhci_irqs" ]]; then
        warn "无 xHCI/r8152 中断 (USB 网口/Livox 未插?)"
    else
        local xhci_ok=1 xhci_isol=0
        for n in $xhci_irqs; do
            local alist; alist="$(cat /proc/irq/$n/effective_affinity_list 2>/dev/null || cat /proc/irq/$n/smp_affinity_list 2>/dev/null)"
            if ! cpu_in_list 8 "$alist"; then
                fail "xHCI/r8152 IRQ $n 未绑 CPU 8 (当前 $alist)"
                xhci_ok=0
            fi
            for c in 4 5 6 7; do
                if cpu_in_list "$c" "$alist"; then
                    fail "xHCI/r8152 IRQ $n 落在隔离核 CPU $c ($alist)——检查 smp_affinity 是否被当成十进制误写(应为十六进制)"
                    xhci_isol=1
                fi
            done
        done
        [[ $xhci_ok -eq 1 && $xhci_isol -eq 0 ]] && pass "xHCI/r8152 中断绑定 CPU 8，且未落隔离核"
    fi

    # EtherCAT 网卡 IRQ → CPU 0
    local ecat_irqs; ecat_irqs="$(grep -iE "igc|${NIC_ECAT}" /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' ')"
    if [[ -z "$ecat_irqs" ]]; then
        warn "无 ${NIC_ECAT}/igc 中断 (网口未 UP/未插线?)"
    else
        local ecat_ok=1
        for n in $ecat_irqs; do
            local alist; alist="$(cat /proc/irq/$n/effective_affinity_list 2>/dev/null || cat /proc/irq/$n/smp_affinity_list 2>/dev/null)"
            if cpu_in_list 0 "$alist"; then
                : # OK on CPU 0
            else
                fail "EtherCAT IRQ $n 未绑 CPU 0 (当前 $alist)"
                ecat_ok=0
            fi
        done
        [[ $ecat_ok -eq 1 ]] && pass "${NIC_ECAT} 中断绑定 CPU 0 (SOEM 主动轮询，硬中断落管家核)"
    fi
}

# ---------- 4. ethtool 网卡调优 ----------
check_ethtool() {
    sect "ethtool 网卡调优 (交叉评估保险项)"

    if ! command -v ethtool >/dev/null 2>&1; then
        warn "ethtool 未安装，跳过"; return
    fi
    if [[ ! -d "/sys/class/net/${NIC_ECAT}" ]]; then
        warn "网卡 ${NIC_ECAT} 不存在，跳过"; return
    fi

    local feat; feat="$(ethtool -k "$NIC_ECAT" 2>/dev/null)"
    local ok_all=1
    for f in gro gso tso lro rxvlan txvlan; do
        if echo "$feat" | grep -iq "^$f: off"; then :; else ok_all=0; fi
    done
    [[ $ok_all -eq 1 ]] && pass "${NIC_ECAT} 硬件卸载已关闭 (gro/gso/tso/lro/rxvlan/txvlan off)" \
        || warn "${NIC_ECAT} 部分卸载未关 (网口 DOWN 时 ethtool -k 可能失败)"

    local coal; coal="$(ethtool -c "$NIC_ECAT" 2>/dev/null)"
    if echo "$coal" | grep -q 'rx-usecs: 0'; then
        pass "${NIC_ECAT} rx-usecs=0 (中断聚合已关)"
    else
        warn "${NIC_ECAT} 中断聚合未关 (rx-usecs 非 0，可能网口未 UP)"
    fi

    # autoneg 仅提示，不计入 PASS/FAIL——config 脚本里 autoneg off 默认关闭，
    # 需现场确认链路稳定后才手动开启（ENABLE_AUTONEG_OFF），这里只如实报告当前状态
    local pause; pause="$(ethtool "$NIC_ECAT" 2>/dev/null | grep -i 'auto-negotiation')"
    if [[ -n "$pause" ]]; then
        echo "      [INFO] ${NIC_ECAT} 当前: $(echo "$pause" | sed 's/^[[:space:]]*//')"
        echo "      [INFO] autoneg off 默认未启用，需现场验证 WKC 稳定后再决定是否开启"
    fi
}

# ---------- 5. systemd / udev ----------
check_services() {
    sect "systemd 服务 + udev (md §4.7)"

    # set_rt_env.sh 存在 + 可执行
    if [[ -x "$RT_ENV_SCRIPT" ]]; then
        pass "set_rt_env.sh 存在且可执行 ($RT_ENV_SCRIPT)"
    else
        fail "set_rt_env.sh 缺失或不可执行: $RT_ENV_SCRIPT"
    fi

    # f1-rt-setup.service enabled
    local st; st="$(systemctl is-enabled "$RT_SETUP_SVC" 2>/dev/null)"
    [[ "$st" == "enabled" ]] && pass "$RT_SETUP_SVC enabled" \
        || fail "$RT_SETUP_SVC 未 enabled ($st)"

    # THP service enabled
    local st2; st2="$(systemctl is-enabled "$THP_SVC" 2>/dev/null)"
    [[ "$st2" == "enabled" ]] && pass "$THP_SVC enabled" \
        || fail "$THP_SVC 未 enabled ($st2)"

    # udev governor 规则
    [[ -f "$UDEV_GOVERNOR" ]] && pass "udev governor 规则存在" \
        || fail "udev governor 规则缺失: $UDEV_GOVERNOR"

    # f1-motion-control.service（应用服务，可能未装）
    if [[ -f /etc/systemd/system/f1-motion-control.service ]]; then
        pass "f1-motion-control.service 存在 (应用部署后 enable)"
    else
        warn "f1-motion-control.service 不存在 (应用未部署，正常)"
    fi
}

# ---------- main ----------
main() {
    echo "F1 minipc 实时环境验证 ($(date))"
    echo "内核: $(uname -r)  | CPU: $(nproc) 线程"

    check_kernel
    check_os
    check_irq
    check_ethtool
    check_services

    echo
    echo "========================================="
    printf "结果: %s PASS, %s WARN, %s FAIL\n" \
        "$(c_grn "$PASS")" "$(c_ylw "$WARN")" "$(c_red "$FAIL")"
    echo "========================================="
    if [[ $FAIL -gt 0 ]]; then
        echo "存在 FAIL 项，请根据上述提示修复。"
        exit 1
    elif [[ $WARN -gt 0 ]]; then
        echo "存在 WARN 项（多为重启前未生效或设备未插），请确认。"
        exit 0
    else
        echo "ALL CHECKS PASSED"
        exit 0
    fi
}

main "$@"