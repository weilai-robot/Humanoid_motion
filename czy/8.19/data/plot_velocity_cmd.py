#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""绘制 8.19 真机测试的 5 个 walk_diag CSV 的速度指令曲线图。

输出: velocity_cmd_curves/ 目录
  - <原文件名>.png           单文件速度曲线（cmd_x / smoothed_speed / cycle_time）
  - all_overview.png          五组数据总览
"""
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Noto Sans CJK SC", "AR PL UKai CN", "WenQuanYi Micro Hei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

SRC_DIR = "/home/robot/czy/F1_github/F1/motion_control/czy/8.19"
OUT_DIR = os.path.join(SRC_DIR, "data", "velocity_cmd_curves")

FILES = [
    ("walk_diag_20260819_150724.csv", "① LT刹车: 0.4恒速→8s二次减速", None),
    ("walk_diag_20260819_150956.csv", "② 0.6 m/s 恒速", None),
    ("walk_diag_20260819_151452.csv", "③ 斜坡加减速(快)", 5.9),
    ("walk_diag_20260819_151632.csv", "④ 三角波+小幅往复", None),
    ("walk_diag_20260819_151821.csv", "⑤ 慢加速→快速减速", 6.7),
]

# 理论 cycle_time 映射（0.35 + ratio*0.35），用于叠加参考
CYCLE_MIN, CYCLE_MAX, SPEED_MAX = 0.35, 0.70, 0.6


def theo_cycle(s):
    s = min(max(s, 0.0), SPEED_MAX)
    return CYCLE_MIN + (s / SPEED_MAX) * (CYCLE_MAX - CYCLE_MIN)


def load(fn):
    rows = list(csv.DictReader(open(os.path.join(SRC_DIR, fn))))
    t0 = int(rows[0]["timestamp_ns"])
    t = [(int(r["timestamp_ns"]) - t0) / 1e9 for r in rows]
    cmdx = [float(r["cmd_linear_x"]) for r in rows]
    sm = [float(r["smoothed_speed"]) for r in rows]
    cyc = [float(r["cycle_time"]) for r in rows]
    pit = [math.degrees(float(r["base_euler_y"])) for r in rows]
    rol = [math.degrees(float(r["base_euler_x"])) for r in rows]
    return t, cmdx, sm, cyc, pit, rol


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    summaries = []

    for fn, title, t_fall in FILES:
        t, cmdx, sm, cyc, pit, rol = load(fn)
        base = os.path.splitext(fn)[0]

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 7), sharex=True,
                                       gridspec_kw={"height_ratios": [2, 1.2]})
        # ── 上图: 速度指令 ──
        ax1.plot(t, cmdx, lw=1.2, color="#1f77b4", label="cmd_linear_x (指令)")
        ax1.plot(t, sm, lw=1.2, color="#d62728", ls="--", label="smoothed_speed (EMA)")
        ax1.axhline(0.05, color="gray", ls=":", lw=1, label="站立阈值 0.05")
        ax1.axhline(SPEED_MAX, color="#7f7f7f", ls=":", lw=1, label="cycle_speed_max 0.6")
        ax1.set_ylabel("速度 (m/s)")
        ax1.set_title(title + f"    ({fn}, {t[-1]:.1f}s)")
        ax1.legend(loc="upper right", fontsize=9)
        ax1.grid(alpha=0.3)

        # 失稳标记
        if t_fall is not None:
            ax1.axvline(t_fall, color="red", lw=2, alpha=0.6)
            ax1.text(t_fall + 0.15, 0.52, f"失稳 t={t_fall:.1f}s", color="red", fontsize=10)
            # 倒地后的区间涂灰
            ax1.axvspan(t_fall, t[-1], color="gray", alpha=0.12)

        # ── 下图: cycle_time + 姿态 ──
        ax2.plot(t, cyc, lw=1.2, color="#2ca02c", label="cycle_time (实际)")
        ax2.plot(t, [theo_cycle(s) for s in sm], lw=1.0, color="#2ca02c", ls=":",
                 alpha=0.6, label="cycle_time (理论映射)")
        ax2.set_ylabel("步态周期 (s)", color="#2ca02c")
        ax2.tick_params(axis="y", labelcolor="#2ca02c")
        ax2.set_xlabel("时间 (s)")
        ax2.grid(alpha=0.3)

        ax2b = ax2.twinx()
        ax2b.plot(t, pit, lw=0.8, color="#ff7f0e", alpha=0.7, label="pitch")
        ax2b.plot(t, rol, lw=0.8, color="#9467bd", alpha=0.7, label="roll")
        ax2b.axhline(10, color="orange", ls=":", lw=0.8)
        ax2b.axhline(-10, color="orange", ls=":", lw=0.8)
        ax2b.set_ylabel("姿态角 (°)")

        # 合并图例
        h1, l1 = ax2.get_legend_handles_labels()
        h2, l2 = ax2b.get_legend_handles_labels()
        ax2.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=8)

        fig.tight_layout()
        fig.savefig(os.path.join(OUT_DIR, base + ".png"), dpi=120)
        plt.close(fig)

        summaries.append((base, title, t, cmdx, sm, cyc, t_fall))
        print(f"saved: {base}.png")

    # ── 总览图 ──
    fig, axes = plt.subplots(5, 1, figsize=(11, 14), sharex=False)
    for ax, (base, title, t, cmdx, sm, cyc, t_fall) in zip(axes, summaries):
        ax.plot(t, cmdx, lw=1.0, color="#1f77b4", label="cmd_x")
        ax.plot(t, sm, lw=1.0, color="#d62728", ls="--", label="EMA")
        ax2 = ax.twinx()
        ax2.plot(t, cyc, lw=0.8, color="#2ca02c", alpha=0.7, label="cycle")
        ax2.set_ylabel("cycle", color="#2ca02c")
        ax2.set_ylim(0.3, 0.75)
        if t_fall is not None:
            ax.axvline(t_fall, color="red", lw=1.5, alpha=0.6)
            ax.axvspan(t_fall, t[-1], color="gray", alpha=0.12)
        ax.set_title(title, fontsize=10)
        ax.set_ylabel("m/s", fontsize=8)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=7)
    axes[-1].set_xlabel("时间 (s)")
    fig.suptitle("8.19 真机测试 - 速度指令曲线总览 (红=EMA平滑, 绿=步态周期, 红=失稳)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(os.path.join(OUT_DIR, "all_overview.png"), dpi=110)
    plt.close(fig)
    print("saved: all_overview.png")


if __name__ == "__main__":
    main()
