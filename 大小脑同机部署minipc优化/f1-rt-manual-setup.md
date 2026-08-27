# F1 minipc RT 手动安装步骤（i7-12800H, Ubuntu 22.04 + PREEMPT_RT）

> 只做 5 件事：隔离核三件套 + 停 irqbalance + default_smp_affinity 排除隔离核 + 网卡 IRQ 绑 CPU0 + xHCI/r8152 绑 CPU8。
> 每一步都给**验证命令**，敲完自己看结果对不对。全程可回滚。

---

## 前置确认（先看清楚再动手）

```bash
# 1. 确认内核（最好是 RT 内核；不是也能继续，但硬实时效果打折）
uname -r
#   期望含 -realtime 或 rt 字样

# 2. 确认 CPU 拓扑：20 线程、隔离 4-7
nproc                              # → 20
lscpu -p=CORE | grep -v '^#' | sort -u | wc -l   # → 14 物理核

# 3. 确认 EtherCAT 网卡：enp2s0（机器唯一网口；igc 是其内核驱动名）
ip -br link                        # 应只见 enp2s0

# 4. 记录当前中断分布（绑完能对比）
grep -iE 'igc|enp2s0|xhci|r8152' /proc/interrupts
#   把每行开头的 IRQ 号记下来（冒号左边那列，如 41:）
```

---

## Step 1：GRUB 隔离核三件套（isolcpus / nohz_full / rcu_nocbs）

**目的**：开机即把 CPU4-7 从普通调度中隔离出来，只给小脑 RT 任务用。

```bash
# 写一个 drop-in 文件（不动 /etc/default/grub 原文件，干净可回滚）
echo 'GRUB_CMDLINE_LINUX_DEFAULT="${GRUB_CMDLINE_LINUX_DEFAULT} isolcpus=4,5,6,7 nohz_full=4,5,6,7 rcu_nocbs=4,5,6,7"' \
  | sudo tee /etc/default/grub.d/99-f1-isol.cfg

# 重新生成 grub.cfg
sudo update-grub
```

**验证**（重启前看文件内容，重启后看内核实际参数）：
```bash
cat /etc/default/grub.d/99-f1-isol.cfg          # 确认写进去了
sudo reboot
# 重启后：
cat /proc/cmdline | tr ' ' '\n' | grep -E 'isolcpus|nohz_full|rcu_nocbs'   # 三个参数都应在
cat /sys/devices/system/cpu/isolated            # → 4-7（隔离已生效）
```

**回滚**：`sudo rm /etc/default/grub.d/99-f1-isol.cfg && sudo update-grub && sudo reboot`

---

## Step 2：停用并 mask irqbalance

**目的**：irqbalance 会把高频中断动态搬到"看起来空闲"的隔离核上——必须停掉并 mask，否则 Step 3/4/5 手动设的亲和性会被它覆盖。

```bash
sudo systemctl disable --now irqbalance
sudo systemctl mask irqbalance
```

**验证**：
```bash
systemctl is-enabled irqbalance    # → masked
systemctl is-active irqbalance     # → inactive
```

**回滚**：`sudo systemctl unmask irqbalance && sudo systemctl enable --now irqbalance`

---

## Step 3：default_smp_affinity 排除隔离核（立即生效）

**目的**：设中断默认亲和性 = 除隔离核外的所有核。这样**之后新注册的中断**默认不落 CPU4-7。

> i7-12800H 共 20 线程，隔离 4-7，其余 0-3 + 8-19。
> 掩码按**十六进制**：CPU0-3 = bit0-3 = 0xF，CPU8-19 = bit8-19，排除 bit4-7。
> 结果 = `fff0f`（验证：bit4,5,6,7 为 0，其余为 1）。

```bash
echo fff0f | sudo tee /proc/irq/default_smp_affinity
```

**验证**：
```bash
cat /proc/irq/default_smp_affinity
#   期望: 00000000,00000000,000fff0f
#   （bit4-7 那一段为 0，表示隔离核被排除）
```

> ⚠️ **坑**：这一步只对**之后新注册**的中断生效。开机时已经注册的网卡中断不受影响，所以要靠 Step 4/5 单独绑。重启后此值也会重置，需 Step 6 持久化。

---

## Step 4：EtherCAT 网卡 IRQ 绑 CPU0

**目的**：把 SOEM 主站网卡（enp2s0）的硬中断固定到管家核 CPU0，不落到隔离核。

```bash
# 先看 IRQ 号（可能有多条 MSI-X 队列，都要绑）
grep -iE 'igc|enp2s0' /proc/interrupts

# 一次性把 enp2s0 网卡的所有中断绑到 CPU0
for n in $(grep -iE 'igc|enp2s0' /proc/interrupts | cut -d: -f1 | tr -d ' '); do
  echo 1 | sudo tee /proc/irq/$n/smp_affinity > /dev/null
  echo "IRQ $n → CPU0 (0x1)"
done
```

**验证**：
```bash
for n in $(grep -iE 'igc|enp2s0' /proc/interrupts | cut -d: -f1 | tr -d ' '); do
  echo -n "IRQ $n: "; cat /proc/irq/$n/smp_affinity
done
#   每行末尾应是 00000001（bit0 = CPU0）
```

> ⚠️ **十六进制坑**：`echo 1` 写入的是十六进制 0x1 = bit0 = CPU0，**不是**十进制1。下一条同理会踩这个坑，务必看 Step 5。

---

## Step 5：xHCI / r8152 IRQ 绑 CPU8

**目的**：USB 控制器（含 Livox 雷达的 r8152 网卡）硬中断固定到 CPU8（管家/导航核）。

```bash
# 先看 IRQ 号
grep -iE 'xhci|r8152' /proc/interrupts

# 绑到 CPU8
for n in $(grep -iE 'xhci|r8152' /proc/interrupts | cut -d: -f1 | tr -d ' '); do
  echo 100 | sudo tee /proc/irq/$n/smp_affinity > /dev/null
  echo "IRQ $n → CPU8 (0x100)"
done
```

**验证**：

```bash
for n in $(grep -iE 'xhci|r8152' /proc/interrupts | cut -d: -f1 | tr -d ' '); do
  echo -n "IRQ $n: "; cat /proc/irq/$n/smp_affinity
done
#   每行应是 ...,00000100（0x100 = bit8 = CPU8）
```

> ⚠️⚠️ **最关键的坑**：这里写的是 `echo 100`，意思是**十六进制 0x100 = bit8 = CPU8**。
> 千万**不要**写成 `echo 256`（"CPU8 是 2^8=256"的直觉是错的）——
> 内核按十六进制解析，`256` 会被当成 `0x256 = 598 = bit1,2,4,6,9`，
> **直接污染你的隔离核 CPU4 和 CPU6**。这是原版文档的已知 bug，务必用 `echo 100`。

---

## Step 6：持久化 Step 3-5（重启后自动重设）

**目的**：`default_smp_affinity` 和各网卡 `smp_affinity` 重启都会重置，必须开机重设。下面手写一个极简脚本 + 一个 oneshot service（你自己创建、自己 enable，全程可见）。

```bash
# 1) 创建脚本目录
sudo mkdir -p /opt/f1/scripts/run
```

```bash
# 2) 写开机脚本（用 cat 写整段，避免逐行 echo 出错）
sudo tee /opt/f1/scripts/run/rt_irq_affinity.sh > /dev/null <<'EOF'
#!/bin/bash
# F1 RT IRQ 隔离持久化 — 开机重设（重启后 affinity 会重置）
set -u

# default IRQ 亲和性：排除隔离核 4-7（CPU0-3,8-19 = 0xfff0f）
echo fff0f > /proc/irq/default_smp_affinity

# EtherCAT 网卡 enp2s0 (驱动 igc) → CPU 0（0x1）
for n in $(grep -iE 'igc|enp2s0' /proc/interrupts | cut -d: -f1 | tr -d ' '); do
  echo 1 > /proc/irq/$n/smp_affinity
done

# xHCI / r8152 (USB 网卡含 Livox) → CPU 8（0x100）
for n in $(grep -iE 'xhci|r8152' /proc/interrupts | cut -d: -f1 | tr -d ' '); do
  echo 100 > /proc/irq/$n/smp_affinity
done
EOF

sudo chmod 0755 /opt/f1/scripts/run/rt_irq_affinity.sh
```

```bash
# 3) 写 systemd service（After=network 确保网卡中断已注册；Before=小脑服务确保先就绪）
sudo tee /etc/systemd/system/f1-rt-irq.service > /dev/null <<'EOF'
[Unit]
Description=F1 RT IRQ affinity setup
DefaultDependencies=no
After=network-online.target
Before=f1-motion-control.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/opt/f1/scripts/run/rt_irq_affinity.sh

[Install]
WantedBy=multi-user.target
EOF

# 4) 启用
sudo systemctl daemon-reload
sudo systemctl enable f1-rt-irq.service
```

> ⚠️ `Before=f1-motion-control.service` 是为了在小脑启动前把中断绑好。**请核对你机器上小脑服务的真实名称**，名字不对就改这一行（名字不对不会报错，但失去时序保证）。

**验证**：
```bash
# 手动跑一次脚本，确认无报错
sudo /opt/f1/scripts/run/rt_irq_affinity.sh && echo "脚本执行 OK"
# 确认 service 已 enabled
systemctl is-enabled f1-rt-irq.service    # → enabled
```

**回滚**：
```bash
sudo systemctl disable --now f1-rt-irq.service
sudo rm /etc/systemd/system/f1-rt-irq.service /opt/f1/scripts/run/rt_irq_affinity.sh
sudo systemctl daemon-reload
```

---

## Step 7：重启 + 全面验证

```bash
sudo reboot
```

重启后逐条核对（每条都应满足）：

| 验证项 | 命令 | 期望 |
|--------|------|------|
| 隔离生效 | `cat /sys/devices/system/cpu/isolated` | `4-7` |
| GRUB 参数 | `cat /proc/cmdline \| tr ' ' '\n' \| grep isolcpus` | 含 isolcpus=4,5,6,7 |
| irqbalance | `systemctl is-enabled irqbalance` | `masked` |
| default 亲和性 | `cat /proc/irq/default_smp_affinity` | `...000fff0f`（排除 4-7） |
| 网卡中断绑 CPU0 | `cat /proc/irq/<igc的IRQ号>/smp_affinity` | 末尾 `00000001` |
| xHCI 绑 CPU8 | `cat /proc/irq/<xhci的IRQ号>/smp_affinity` | 含 `00000100` |
| service 跑过 | `systemctl status f1-rt-irq.service` | active (exited) |

> 额外确认隔离核"被打扰程度"：重启跑一会儿后，看隔离核的中断计数是否几乎不增长——
> ```bash
> cat /proc/interrupts | head -1   # 表头，CPU4-7 在第 5-8 列
> # 隔离核对应列的数字应远小于 CPU0/8，说明中断没往隔离核落
> ```

---

## 总回滚（全部还原）

```bash
sudo systemctl disable --now f1-rt-irq.service 2>/dev/null
sudo rm -f /etc/systemd/system/f1-rt-irq.service /opt/f1/scripts/run/rt_irq_affinity.sh /etc/default/grub.d/99-f1-isol.cfg
sudo systemctl unmask irqbalance && sudo systemctl enable --now irqbalance
sudo systemctl daemon-reload
sudo update-grub
sudo reboot
```

---

## 两个已知坑（务必知道）

1. **smp_affinity 写入按十六进制解析**：CPU0=`echo 1`(0x1)、CPU8=`echo 100`(0x100)。**绝不能**用 `echo 256` 绑 CPU8——会被当 0x256 污染隔离核 4/6。
2. **default_smp_affinity 只对新注册中断生效**：已注册的网卡中断要单独绑（Step 4/5），不能只设 default 就指望全部生效。

---