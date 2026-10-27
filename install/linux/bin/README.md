# install/linux/bin — 文件清单（谁是谁）

> 本目录随构建整体拷贝到部署机 build 目录（`cp -r` 语义，含子目录），运行时
> cwd=build 目录。启动体系分三层：**入口薄壳 → 主编排 → 检查仪器**；谁是哪个，
> 见下表。

## 入口脚本（现场敲的就是这些）

| 文件 | 场景 | 顶层 cfg | 运行身份 |
|------|------|----------|---------|
| `run.sh` | 手柄遥控（习惯 `sudo ./run.sh`） | `cfg/x1_cfg.yaml` | root |
| `run_real_nav.sh` | 大脑联合导航（`./run_real_nav.sh`） | `cfg/x1_cfg_real_nav.yaml` | robot |
| `run_sim.sh` | 仿真（无显示时自动起 Xvfb） | `cfg/x1_cfg_sim.yaml` | 当前用户 |
| `run_identifier.sh` | 辨识模式 | `cfg/x1_cfg_identifier.yaml` | 当前用户 |
| `run_with_recording.sh` | 手柄 + 录包 | `cfg/x1_cfg.yaml` | 当前用户 |

`run.sh` / `run_real_nav.sh` 是 8 行薄壳，全部逻辑在 `run_main.sh`；
后三个为旧版独立脚本（未收编，逻辑各自一份，改动需同步多处）。

## 主编排

| 文件 | 干什么 |
|------|--------|
| `run_main.sh` | 预检三路（程序域 / RT 资格 / 环境门禁）→ 颈部 → setcap → 拉起 aimrt_main → 启动后 rt_verify → 守场回收。`--check-only` 只预检不启动（为 systemd ExecStartPre 预留）。三路预检**一律硬挡**，无逃生阀 |

## checks/ — 检查仪器（高内聚低耦合，均可单独运行）

| 文件 | 是谁 | 什么时候跑 |
|------|------|-----------|
| `checks/rt_perm_check.sh` | RT 资格：当前用户能否设置实时调度（root / cap_sys_nice / rtprio 限额三判据） | 每次启动（预检一环）；可单独敲 |
| `checks/rt_gate_check.sh` | 环境门禁：读 `/run/f1-rt-verify/status` 凭证两步判定（存在性 + RESULT 以 PASS 开头）+ 网口 UP / bind CPU 在线两项活体检查 | 每次启动（预检一环）；门禁被拦时手敲它复现判定链 |
| `checks/rt_verify.sh` | 启动后线程级验证：两条命脉线程的 FF/优先级/绑核实证（抓假实时） | 每次启动后 |
| `checks/cfg_helper.py` | yaml 读取小工具（get / module-cfg / onnx-models） | 被上述共用 |
| `checks/f1-rt-verify.service` | 验收员 verify-rt-config.sh 每次开机自动跑并产出 `/run/f1-rt-verify/` 凭证（report.txt 全文 + status 机器读），供启动门禁读取（安装法见下） | 每次开机 |

仪器接口均为参数式（`--need-prio` / `--status` / `--nic` / `--bind-cpus` 等），
编排者（run_main.sh）从 cfg 算参数传入；仪器不写死机器参数，换 285H 等不同 CPU
拓扑只改 cfg。

## 颈部 / 环境 / 录包 / 仿真工具（顶层，旧脚本引用着，勿挪动）

| 文件 | 干什么 |
|------|--------|
| `neck_hold.py` / `htd35h_controller.py` | 颈部双舵机使能与下使能（串口） |
| `ros2_source.sh` / `ros2_setup.sh` | ROS2 环境自动探测与 source |
| `record.sh` | 录包（run_with_recording.sh 后台拉起） |
| `sim_keyboard_ctrl.py` / `auto_walk_ctrl.py` | 仿真键盘 / 自动行走速度发布 |
| `rt_walk_verify.py` | 直线行走偏航验证（与 RT 线程检查无关） |
| `cfg/` | 各模式顶层与模块配置 |

## 排障直跑（绕过启动器，唯一合法的"无预检"路径）

```bash
cd build
./aimrt_main --cfg_file_path=./cfg/x1_cfg.yaml     # 或 real_nav cfg
```

不经预检、无任何拦截 —— 仅排障用；正因有这条自然出口，预检不设逃生阀。

## f1-rt-verify.service 安装（每台部署机 root 一次）

```bash
sudo mkdir -p /opt/f1/scripts
sudo cp <verify-rt-config.sh 路径>/verify-rt-config.sh /opt/f1/scripts/  # 脚本由部署/运维侧提供
sudo cp checks/f1-rt-verify.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable f1-rt-verify.service
# 查看开机自检报告: journalctl -u f1-rt-verify.service -b
# 查看门禁凭证:       cat /run/f1-rt-verify/status
# 手动补跑（凭证随之刷新）: sudo systemctl start f1-rt-verify.service
```

安装后每次开机：本单元自动全量验收并写 `/run/f1-rt-verify/` 凭证（tmpfs，
每次开机清空重写）→ run_main.sh 的环境门禁（rt_gate_check.sh）读该凭证，
非 PASS 即拦截启动。**swap 决策为"保留"的机器**：在 unit 的 ExecStart 末尾加
`--expect-swap-on`（status 写 PASS_WITH_EXPECTED 仍放行；swap 关闭后删掉该参数）。
