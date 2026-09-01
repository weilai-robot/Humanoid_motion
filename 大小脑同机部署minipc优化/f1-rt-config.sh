#!/usr/bin/env bash
# =============================================================================
# F1 minipc 实时环境一键配置 (i7-12800H, Ubuntu 22.04 + PREEMPT_RT)
#
# 适用：大脑(navigation)+小脑(motion_control)同机部署，SOEM EtherCAT 主站
# 前提：检测到 PREEMPT_RT 内核已装则跳过；未装则经 Ubuntu Pro 安装 (参考 realtime-config.sh)
#
# 本版整合来源：
#   - f1-rt-config.sh（工程健壮性主干：幂等/备份/原子写入/preflight/回滚）
#   - f1-rt-config-gemini.sh（吸收其唯一优点：irqaffinity= 内核参数，开机最早期生效）
#   - 人工复核：修正两版共同继承自原文档的 smp_affinity 十六进制 bug
#              （echo 256 想绑 CPU8，实际因十六进制解析绑到 CPU 1,2,4,6,9，
#               污染了隔离核 4/6——已修正为 echo 100）
#   - 人工复核：明确排除 mce=off（执行器安全系统不应关闭硬件故障上报）
#   - 参考移植：realtime-config.sh 的 step_ubuntu_pro_rt 逻辑（Ubuntu Pro 安装 RT 内核），
#               使本脚本从"只校验"升级为"检测+安装"；token 存 /etc/f1/pro_token (0600 root:root)，
#               读文件→交互输入→回写，不依赖环境变量，可安全推 GitHub
#
# 工程模式：
#   - 幂等：marker block + drop-in 片段，可重复运行
#   - 时间戳备份：改原文件前先备份到 /var/backups/f1-rt
#   - 原子替换：mktemp + install，避免半写状态
#   - systemd oneshot 封装运行时设置，THP 禁用做成独立早期服务（双保险）
#
# 交叉评估结论：
#   - 采纳：sched_rt_runtime_us=-1 / cpufreq/boost=0(替代 no_turbo) / default_smp_affinity
#           兜底 / busy_poll 剔除 / SOEM NIC IRQ 绑管家核(md 策略) / ethtool 保险
#           / irqaffinity=（GRUB 层，早于任何 systemd 服务生效，来自 gemini 版的合理项）
#   - 排除：mce=off(非周期性,丢硬件告警，执行器安全系统不应关) / nosmt(砍大脑超线程)
#           / idle=poll(热失控) / rcu_nocb_poll(管家核 RCU 常驻干扰大脑)
#           / nice=-20(FIFO 无意义,污染大脑 CFS)
#
# 已知风险点（未默认开启，需现场验证后再打开）：
#   - ethtool -A <if> autoneg off：若对端/交换机未固定速率双工，可能导致链路协商失败。
#     默认 ENABLE_AUTONEG_OFF=false，仅在链路已用 autoneg on 稳定验证过 WKC 后再手动改 true。
# =============================================================================

set -Eeuo pipefail

# ---------- 常量（F1 固定值，换机器改这里）----------
readonly TAG="f1-rt"
readonly TS="$(date +%Y%m%d-%H%M%S)"
readonly BACKUP_DIR="/var/backups/${TAG}"
readonly MARK_BEGIN="# BEGIN ${TAG} (managed - do not edit by hand)"
readonly MARK_END="# END ${TAG}"

# drop-in 片段（幂等，可整文件替换）
readonly GRUB_DROPIN="/etc/default/grub.d/99-${TAG}.cfg"
readonly SYSCTL_DROPIN="/etc/sysctl.d/99-${TAG}.conf"

# 可执行运行时脚本 + systemd 单元 + udev 规则
readonly RT_ENV_SCRIPT="/opt/f1/scripts/run/set_rt_env.sh"          # 遵循 md §4.7.3 路径
readonly RT_SETUP_SVC="f1-rt-setup.service"                         # md §4.7.2
readonly THP_SVC="f1-rt-disable-thp.service"
readonly UDEV_GOVERNOR="/etc/udev/rules.d/99-performance-governor.rules"

# F1 拓扑固定值
readonly ISOL_CPUS="4,5,6,7"      # P-core 2,3 含 HT sibling (md §1.3 实测)
readonly HK_CPUS="0-3,8-19"       # housekeeping：全部非隔离核（i7-12800H 20 线程排除 4-7）
readonly NIC_ECAT="enp2s0"        # Intel igc 板载 → EtherCAT (md §1.1)

# 现场验证后再打开：链路已确认 autoneg on 且 WKC 稳定，再改为 true 并重跑脚本
readonly ENABLE_AUTONEG_OFF="false"

# Ubuntu Pro token 文件：脚本读取此文件获取 token (0600 root:root)；
# 文件不存在则交互输入，attach 成功后自动回写。获取 token：https://ubuntu.com/pro（个人版免费，最多 5 台）
readonly PRO_TOKEN_FILE="/etc/f1/pro_token"

# ---------- ui helpers ----------
c_red() { printf '\033[31m%s\033[0m\n' "$*"; }
c_grn() { printf '\033[32m%s\033[0m\n' "$*"; }
c_ylw() { printf '\033[33m%s\033[0m\n' "$*"; }
c_cyn() { printf '\033[36m%s\033[0m\n' "$*"; }

info() { c_cyn "[INFO]  $*"; }
ok()   { c_grn "[ OK ]  $*"; }
warn() { c_ylw "[WARN]  $*"; }
err()  { c_red "[ERR ]  $*" >&2; }
die()  { err "$*"; exit 1; }

# 交互辅助（移植自 realtime-config.sh，用于 Ubuntu Pro token 输入）
ask() {
    # ask "prompt" "default"  ->  echoes value
    local prompt="$1" default="${2-}" ans
    if [[ -n "$default" ]]; then
        read -r -p "$(c_ylw "? ${prompt} [${default}]: ")" ans </dev/tty || true
        echo "${ans:-$default}"
    else
        read -r -p "$(c_ylw "? ${prompt}: ")" ans </dev/tty || true
        echo "$ans"
    fi
}

ask_yn() {
    # ask_yn "prompt" "y|n"  -> returns 0 for yes, 1 for no
    local prompt="$1" default="${2:-n}" ans
    local hint="[y/N]"; [[ "$default" == "y" ]] && hint="[Y/n]"
    read -r -p "$(c_ylw "? ${prompt} ${hint}: ")" ans </dev/tty || true
    ans="${ans:-$default}"
    [[ "${ans,,}" == "y" || "${ans,,}" == "yes" ]]
}

# ---------- file helpers ----------
backup_file() {
    local f="$1"
    [[ -f "$f" ]] || return 0
    mkdir -p "$BACKUP_DIR"
    local dest="${BACKUP_DIR}/$(echo "$f" | tr '/' '_').${TS}.bak"
    if [[ ! -f "$dest" ]]; then
        cp -a "$f" "$dest"
        info "Backed up $f -> $dest"
    fi
}

# 写配置片段文件：包 marker block，适合 drop-in / service / udev（非可执行）
write_managed_file() {
    # write_managed_file <path> <mode> <content>
    local path="$1" mode="$2" content="$3"
    mkdir -p "$(dirname "$path")"
    # 若已存在但无本 TAG marker，先备份原文件
    if [[ -f "$path" ]] && ! grep -qF "$MARK_BEGIN" "$path"; then
        backup_file "$path"
    fi
    local tmp; tmp="$(mktemp)"
    {
        printf '%s\n' "$MARK_BEGIN"
        printf '# Generated: %s\n' "$TS"
        printf '%s\n' "$content"
        printf '%s\n' "$MARK_END"
    } >"$tmp"
    install -m "$mode" -o root -g root "$tmp" "$path"
    rm -f "$tmp"
}

# 写可执行脚本：shebang 必须第一行（marker header 会破坏 execve，故只加注释行）
write_exec_script() {
    # write_exec_script <path> <mode> <content>   (content 须以 #! 开头)
    local path="$1" mode="$2" content="$3"
    mkdir -p "$(dirname "$path")"
    [[ -f "$path" ]] && backup_file "$path"
    local tmp; tmp="$(mktemp)"
    printf '%s\n' "$content" >"$tmp"
    install -m "$mode" -o root -g root "$tmp" "$path"
    rm -f "$tmp"
}

# ---------- pre-flight ----------
preflight() {
    [[ $EUID -eq 0 ]] || die "必须以 root 运行 (use sudo)."

    local ver_id; ver_id="$(. /etc/os-release && echo "$VERSION_ID")"
    [[ "$ver_id" == "22.04" ]] || die "检测到 Ubuntu $ver_id，本脚本仅面向 22.04 (隔离核/CPU 拓扑按 22.04 + i7-12800H 写死)，已中止."

    [[ "$(uname -m)" == "x86_64" ]] || die "本脚本面向 x86_64."

    if systemd-detect-virt --quiet; then
        die "检测到虚拟化 ($(systemd-detect-virt))，硬实时在 VM 中无意义."
    fi

    # 注：PREEMPT_RT 内核的检测/安装已迁至 step_ubuntu_pro_rt（main 中 preflight 之后执行），
    #     使本脚本从"只校验"升级为"检测+安装"；VM 校验仍在此处硬阻断（生产硬实时在 VM 无意义）

    # CPU 型号/拓扑三重校验：本脚本隔离核 ISOL_CPUS=4,5,6,7 按 i7-12800H 拓扑写死，
    # 型号/核数不同则隔离策略不同，任一不符直接中止（不留 warn 绕过）
    local model; model="$(grep -m1 '^model name' /proc/cpuinfo | sed 's/.*: //')"
    [[ "$model" == *"i7-12800H"* ]] || die "目标主控不是 i7-12800H (检测到: $model)。隔离核 ISOL_CPUS=4,5,6,7 按 i7-12800H 拓扑写死，型号不同则优化策略不同，已中止."

    local ncpu; ncpu="$(nproc)"
    [[ "$ncpu" -eq 20 ]] || die "逻辑线程数 $ncpu ≠ 20 (i7-12800H 应为 6P+8E=14核/20线程)，已中止."

    local ncores; ncores="$(lscpu -p=CORE 2>/dev/null | grep -v '^#' | sort -u | wc -l)"
    [[ "$ncores" -eq 14 ]] || die "物理核数 $ncores ≠ 14 (i7-12800H 应为 6P+8E=14核)，已中止."

    # 运行时依赖
    command -v python3 >/dev/null || apt-get install -y python3
    command -v ethtool  >/dev/null || apt-get install -y ethtool

    mkdir -p "$BACKUP_DIR"
    ok "Pre-flight 检查通过."
}

# ---------- steps ----------

step_ubuntu_pro_rt() {
    info "Step: 检测/安装 PREEMPT_RT 内核 (Ubuntu Pro realtime-kernel)"

    # ① 已运行 RT 内核 → 跳过安装。
    #    但检测订阅状态：detach 不会卸载已装的 RT 内核包，内核照跑、实时控制不受影响，
    #    仅失去 RT 内核官方更新补丁——给 warn 提醒，不阻断。
    if uname -v | grep -qi "PREEMPT RT" || uname -r | grep -qi "rt"; then
        ok "系统已运行 RT 内核 ($(uname -r))，跳过安装."
        if ! pro status --format=json 2>/dev/null | grep -q '"attached": *true'; then
            warn "RT 内核在跑、实时控制正常，但 Ubuntu Pro 订阅未 attached：失去 RT 内核官方更新补丁。如需保持订阅激活，提供 token 重跑本脚本会重新 attach。"
        fi
        return 0
    fi

    warn "未检测到 RT 内核——所有后续配置都必须基于 RT 内核生效，现尝试经 Ubuntu Pro 安装."

    # 1. 确保 ubuntu-advantage-tools 已装
    if ! command -v pro >/dev/null 2>&1; then
        apt-get update -qq
        apt-get install -y ubuntu-advantage-tools
    fi

    # 2. Pro attach
    #    ② 已 attached 跳过；否则读 /etc/f1/pro_token，无文件/为空则交互输入并回写
    if pro status --format=json 2>/dev/null | grep -q '"attached": *true'; then
        ok "Ubuntu Pro 已 attached."
    else
        warn "Ubuntu Pro 未 attached."
        local tok="" tok_from_file=0
        if [[ -f "$PRO_TOKEN_FILE" ]]; then
            tok="$(cat "$PRO_TOKEN_FILE" 2>/dev/null)"
            [[ -n "$tok" ]] && tok_from_file=1
        fi
        if [[ -z "$tok" ]]; then
            if ask_yn "现在 attach Ubuntu Pro？（需要 token，个人版免费：https://ubuntu.com/pro）" "y"; then
                tok="$(ask "粘贴你的 Ubuntu Pro token" "")"
            else
                warn "跳过 RT 内核安装。后续手动: sudo pro attach <token> && sudo pro enable realtime-kernel"
                return 0
            fi
        fi
        [[ -n "$tok" ]] || { warn "token 为空，跳过 RT 内核安装."; return 0; }
        # attach；失败按 stderr 关键字给针对性提示
        local err_out
        if ! err_out="$(pro attach "$tok" 2>&1)"; then
            if echo "$err_out" | grep -qi "quota\|exceeded\|limit reached\|maximum"; then
                warn "attach 失败：该 token 绑定设备数已达上限（Ubuntu Pro 个人版最多 5 台）。在 Ubuntu Pro web 端释放一台设备后重试。"
            else
                warn "attach 失败：$err_out"
            fi
            warn "跳过 RT 内核安装（其余配置仍会写入，重启进 RT 内核后生效）."
            return 0
        fi
        ok "Ubuntu Pro 已 attach."
        # 仅交互输入的 token 回写文件；从文件读的已存在，不重复写
        if [[ "$tok_from_file" -eq 0 ]]; then
            mkdir -p "$(dirname "$PRO_TOKEN_FILE")"
            local tmp_tok; tmp_tok="$(mktemp)"
            printf '%s\n' "$tok" >"$tmp_tok"
            install -m 0600 -o root -g root "$tmp_tok" "$PRO_TOKEN_FILE"
            rm -f "$tmp_tok"
            info "token 已存入 $PRO_TOKEN_FILE (0600 root:root)，下次重跑免输入."
        fi
    fi

    # 3. enable realtime-kernel
    #    ③ 已 enabled 跳过；否则直接装（不再 ask_yn）
    if pro status --format=json 2>/dev/null | grep -q '"name": *"realtime-kernel"[^}]*"status": *"enabled"'; then
        ok "realtime-kernel 已 enabled."
    else
        pro enable realtime-kernel --assume-yes || warn "pro enable realtime-kernel 报错，RT 内核未装上."
    fi

    warn "若刚安装了 RT 内核，必须重启进 RT 内核后 GRUB/sysctl 才真正生效."
}

step_grub_cmdline() {
    info "Step: GRUB 内核命令行 (drop-in)"
    # md §2.7 的 6 项 + 交叉评估新增项 = 12 项
    # 新增 irqaffinity=${HK_CPUS}（吸收自 gemini 版）：
    #   在内核最早期生效，早于任何 systemd 服务/运行时脚本，
    #   避免开机早期（网络协商、systemd 早期阶段）新注册中断落到隔离核上，
    #   与运行时动态计算的 default_smp_affinity 形成"早期兜底 + 精确兜底"两层。
    # 排除：mce=off(非周期性,丢硬件告警，执行器安全系统不应关)
    #       / nosmt(砍大脑超线程) / idle=poll(热失控)
    #       / rcu_nocb_poll(管家核 RCU 常驻) / nosoftlockup skew_tick(弱增强,默认不加)
    local params=(
        "isolcpus=${ISOL_CPUS}"
        "nohz_full=${ISOL_CPUS}"
        "rcu_nocbs=${ISOL_CPUS}"
        "irqaffinity=${HK_CPUS}"
        "intel_pstate=disable"
        "processor.max_cstate=0"
        "intel_idle.max_cstate=0"
        "watchdog_thresh=60"
        "audit=0"
        "tsc=reliable"
        "clocksource=tsc"
        "transparent_hugepage=never"
    )
    local line="GRUB_CMDLINE_LINUX_DEFAULT=\"\${GRUB_CMDLINE_LINUX_DEFAULT} ${params[*]}\""
    write_managed_file "$GRUB_DROPIN" 0644 "$line"
    ok "写入 $GRUB_DROPIN (12 个参数，已排除 mce/nosmt/idle=poll)"
}

step_sysctl() {
    info "Step: sysctl 调优 (drop-in)"
    local content
    content="$(cat <<'EOF'
# --- RT 调度节流解除 ---
# isolcpus 隔离核上 RT 任务可 100% 占用，禁用 5% 强制让出 (防周期碰撞)
kernel.sched_rt_runtime_us = -1

# --- 定时器/NUMA 静音 ---
# 配合 nohz_full 阻止定时器跨核游走 (零开销保险)
kernel.timer_migration = 0
# 单 NUMA 节点，关闭自动 NUMA 负载均衡 (零开销保险)
kernel.numa_balancing = 0

# --- VM 统计间隔拉长 ---
# 减少 vmstat 唤醒频率
vm.stat_interval = 10

# 注：vm.swappiness 不设——服务层 set_rt_env.sh 已 swapoff，且 fstab swap 已注释
# 注：net.core.busy_poll/busy_read 不设——SOEM 用 raw socket，opt-in 无意义
EOF
)"
    write_managed_file "$SYSCTL_DROPIN" 0644 "$content"
    sysctl --system >/dev/null 2>&1 || warn "sysctl --system 有警告，重启后生效."
    ok "写入 $SYSCTL_DROPIN"
}

step_disable_thp() {
    info "Step: 禁用透明大页 (systemd oneshot，basic.target 前生效)"
    local unit="/etc/systemd/system/${THP_SVC}"
    write_managed_file "$unit" 0644 "$(cat <<'EOF'
[Unit]
Description=Disable Transparent Huge Pages (F1 RT)
DefaultDependencies=no
After=sysinit.target local-fs.target
Before=basic.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo never > /sys/kernel/mm/transparent_hugepage/enabled; echo never > /sys/kernel/mm/transparent_hugepage/defrag'
RemainAfterExit=yes

[Install]
WantedBy=basic.target
EOF
)"
    systemctl daemon-reload
    systemctl enable --now "$THP_SVC" >/dev/null 2>&1 || warn "$THP_SVC 启动失败，重启后生效."
    ok "$THP_SVC enabled (与 GRUB transparent_hugepage=never 双保险)."
}

step_swap_off() {
    info "Step: 关闭 Swap (md §3.2)"
    if [[ "$(swapon --noheadings --show 2>/dev/null | wc -l)" -gt 0 ]]; then
        swapoff -a || warn "swapoff -a 报错."
    fi
    if grep -Ev '^\s*#' /etc/fstab | grep -qw swap; then
        backup_file /etc/fstab
        sed -i -E "s|^([^#].*\sswap\s.*)\$|# [${TAG} ${TS} disabled swap] \1|" /etc/fstab
        ok "已注释 /etc/fstab 中的 swap 行."
    else
        ok "/etc/fstab 无 swap 行."
    fi
}

step_disable_irqbalance() {
    info "Step: 停用并 mask irqbalance (md §3.5，诊断报告增量)"
    if systemctl list-unit-files 2>/dev/null | grep -q '^irqbalance\.service'; then
        systemctl disable --now irqbalance.service 2>/dev/null || true
        systemctl mask irqbalance.service 2>/dev/null || true
        ok "irqbalance 已停用 + mask (防止绕过 isolcpus 重路由 IRQ)."
    else
        ok "irqbalance 未安装."
    fi
}

step_set_rt_env_script() {
    info "Step: 生成运行时环境脚本 set_rt_env.sh (md §4.7.3 增强版)"
    # 在 md 原版基础上新增：cpufreq/boost=0 / default_smp_affinity 兜底 / ethtool 调优
    # 修正 md 的 echo 256 十六进制歧义 bug → echo 100 (0x100 = CPU 8)
    # 说明：GRUB 层已有 irqaffinity=${HK_CPUS} 在最早期生效，此处 default_smp_affinity
    # 是"运行时精确兜底"——按实际在线 CPU 动态计算，覆盖 GRUB 静态区间之外的边缘情况
    # （例如 CPU 拓扑与预期不符时仍能正确排除隔离核）。
    write_exec_script "$RT_ENV_SCRIPT" 0755 "$(cat <<SET_RT_EOF
#!/bin/bash
# F1 RT environment setup — 开机经 f1-rt-setup.service 调用
# 幂等，可重复执行。增强版（md §4.7.3 + 交叉评估新增项）
set -u

ISOL="${ISOL_CPUS}"
ENABLE_AUTONEG_OFF="${ENABLE_AUTONEG_OFF}"

# 1. governor = performance（全核，md §3.1）
for g in /sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_governor; do
    [ -w "\$g" ] && echo performance > "\$g" 2>/dev/null || true
done

# 2. 禁用睿频（ACPI 通用路径；因 intel_pstate=disable，no_turbo 节点不存在）
[ -w /sys/devices/system/cpu/cpufreq/boost ] && echo 0 > /sys/devices/system/cpu/cpufreq/boost 2>/dev/null || true

# 3. default IRQ 亲和性精确兜底 = 所有非隔离核（防止热插拔设备 IRQ 污染实时核）
#    GRUB 的 irqaffinity= 已在最早期生效，这里按实际在线 CPU 动态计算，双重保险
DEF_MASK=\$(python3 - "\$ISOL" <<'PY' 2>/dev/null || echo fff0f
import sys, os
isol = set(int(x) for x in sys.argv[1].split(','))
mask = 0
for p in os.listdir('/sys/devices/system/cpu'):
    if p.startswith('cpu') and p[3:].isdigit():
        c = int(p[3:])
        if c not in isol:
            mask |= (1 << c)
print(f'{mask:x}')
PY
)
echo "\$DEF_MASK" > /proc/irq/default_smp_affinity 2>/dev/null || true

# 4. NVMe 队列中断 → CPU 0 (housekeeping)，移出实时核 (md §3.4)
#    smp_affinity 按十六进制解析：0x1 = CPU 0
for n in \$(grep nvme /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' '); do
    echo 1 > /proc/irq/\$n/smp_affinity 2>/dev/null || true
done

# 5. xHCI / r8152 (USB 网卡含 Livox) → CPU 8 (导航核) (md §3.3)
#    !!! 注意：smp_affinity 按十六进制解析，CPU 8 = bit8 = 0x100 !!!
#    原文档 "echo 256" 会被解析为 0x256=598=CPU{1,2,4,6,9}，直接污染隔离核 4/6，
#    已确认并修正为 echo 100（十六进制 0x100 才是 CPU 8）。
for n in \$(grep -iE 'xhci|r8152' /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' '); do
    echo 100 > /proc/irq/\$n/smp_affinity 2>/dev/null || true
done

# 6. EtherCAT 网卡 (igc/enp2s0) → CPU 0 (SOEM 主动轮询，硬中断落管家核不抢 RT 核)
for n in \$(grep -iE 'igc|${NIC_ECAT}' /proc/interrupts 2>/dev/null | cut -d: -f1 | tr -d ' '); do
    echo 1 > /proc/irq/\$n/smp_affinity 2>/dev/null || true
done

# 7. ethtool 网卡调优：关中断聚合 + 硬件卸载 (交叉评估保险项)
if command -v ethtool >/dev/null 2>&1; then
    IF=${NIC_ECAT}
    ethtool -G \$IF rx 128 tx 128 2>/dev/null || true
    ethtool -K \$IF gro off gso off tso off lro off rxvlan off txvlan off 2>/dev/null || true
    ethtool -C \$IF adaptive-rx off adaptive-tx off rx-usecs 0 tx-usecs 0 2>/dev/null || true
    ethtool -A \$IF rx off tx off 2>/dev/null || true
    # autoneg off 有链路协商失败风险，未在现场验证前默认不开启（见脚本头部说明）
    if [ "\$ENABLE_AUTONEG_OFF" = "true" ]; then
        ethtool -A \$IF autoneg off 2>/dev/null || true
    fi
    ip link set \$IF txqueuelen 100 2>/dev/null || true
fi

# 8. 兜底：确认 swap 已关
if [ -n "\$(swapon --show 2>/dev/null)" ]; then
    echo "WARN: swap 仍激活，执行 swapoff -a"
    swapoff -a 2>/dev/null || true
fi

echo "F1 RT environment setup done."
SET_RT_EOF
)"
    ok "写入 $RT_ENV_SCRIPT (0755)"
}

step_rt_setup_service() {
    info "Step: 生成 f1-rt-setup.service (md §4.7.2 oneshot)"
    local unit="/etc/systemd/system/${RT_SETUP_SVC}"
    write_managed_file "$unit" 0644 "$(cat <<EOF
[Unit]
Description=F1 Realtime Environment Setup (governor + IRQ affinity + ethtool)
After=network-online.target
DefaultDependencies=no
Before=f1-motion-control.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=${RT_ENV_SCRIPT}

[Install]
WantedBy=multi-user.target
EOF
)"
    systemctl daemon-reload
    systemctl enable "$RT_SETUP_SVC" >/dev/null 2>&1 || warn "enable $RT_SETUP_SVC 失败，检查 service 文件."
    ok "$RT_SETUP_SVC enabled (Before=f1-motion-control.service)."
}

step_udev_governor() {
    info "Step: udev 规则持久化 governor (md §4.7.4)"
    write_managed_file "$UDEV_GOVERNOR" 0644 "$(cat <<'EOF'
ACTION=="add", SUBSYSTEM=="cpu", KERNEL=="cpu[0-9]*", \
  RUN+="/bin/sh -c 'echo performance > /sys/devices/system/cpu/%k/cpufreq/scaling_governor'"
EOF
)"
    ok "写入 $UDEV_GOVERNOR (与 set_rt_env.sh 互为兜底)."
}

step_update_grub() {
    info "Step: update-grub"
    if command -v update-grub >/dev/null 2>&1; then
        update-grub
        ok "GRUB 已更新."
    else
        warn "update-grub 不可用，手动执行: sudo update-grub"
    fi
}

# ---------- summary ----------
print_summary() {
    echo
    c_cyn "============ F1 RT 配置完成 ============"
    cat <<EOF
备份目录         : ${BACKUP_DIR}
GRUB drop-in     : ${GRUB_DROPIN}
sysctl drop-in   : ${SYSCTL_DROPIN}
运行时脚本       : ${RT_ENV_SCRIPT}
systemd 单元     : ${THP_SVC}, ${RT_SETUP_SVC}
udev 规则        : ${UDEV_GOVERNOR}

本版相对两份原始草稿的关键修正:
  1) 修正 smp_affinity 十六进制 bug: xHCI/r8152 绑核由 "echo 256" 改为 "echo 100"
     (原写法会把 CPU8 亲和性解析成 CPU 1/2/4/6/9，污染隔离核 4/6)
  2) GRUB 增加 irqaffinity=${HK_CPUS}，比仅靠运行时兜底更早生效
  3) 排除 mce=off (执行器安全系统不应关闭硬件故障上报)
  4) ethtool autoneg off 默认关闭，需现场验证链路后手动开启 (ENABLE_AUTONEG_OFF)
  5) RT 内核从"只校验"升级为"检测+安装" (step_ubuntu_pro_rt，参考 realtime-config.sh)；
     未装则经 Ubuntu Pro 安装，token 存 /etc/f1/pro_token (0600)，不依赖环境变量

下一步:
  1) 若 step_ubuntu_pro_rt 新装了 RT 内核，必须重启进 RT 内核 (uname -r 应含 -realtime)；
     已是 RT 内核则重启使 GRUB 参数生效: sudo reboot
  2) 重启后验证: sudo ./verify-rt-config.sh
  3) 现场核对 EtherCAT 链路 WKC 稳定后，再考虑是否开启 autoneg off

回滚:
  - rm ${GRUB_DROPIN} ${SYSCTL_DROPIN}
  - systemctl disable --now ${THP_SVC} ${RT_SETUP_SVC}
  - rm /etc/systemd/system/${THP_SVC} /etc/systemd/system/${RT_SETUP_SVC}
  - rm ${UDEV_GOVERNOR} ${RT_ENV_SCRIPT}
  - 从 ${BACKUP_DIR} 恢复 fstab
  - systemctl unmask irqbalance && systemctl enable --now irqbalance
  - sudo update-grub && sudo reboot
=========================================
EOF
}

# ---------- main ----------
main() {
    preflight
    step_ubuntu_pro_rt

    step_grub_cmdline
    step_sysctl
    step_disable_thp
    step_swap_off
    step_disable_irqbalance
    step_set_rt_env_script
    step_rt_setup_service
    step_udev_governor
    step_update_grub

    print_summary
}

main "$@"