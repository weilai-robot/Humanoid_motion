#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颈部双舵机保持锁定 — 小脑启动前置步骤
=====================================

背景:
  F1 颈部装有两个 HTD-35H 总线舵机 (ID1=头部左右, ID2=点头, 限位650)，
  由小脑 Intel 主机经 CH340 串口 -> STM32 扩展板控制。小脑/大脑运行期间
  这两个舵机不参与控制，但必须保持不动（避免头部晃动 / Mid-360 外参漂移）。

用法:
  python3 neck_hold.py                          # 默认: /dev/ttyUSB0, 回中位后锁定(home)
  python3 neck_hold.py --mode lock              # 当前位置直接锁定(零运动)
  python3 neck_hold.py --port /dev/neck_servo   # 使用 udev 别名

默认 home 的原因: Mid-360 激光雷达安装在头部, 头部必须回到确定性中位(500,500)
雷达外参才与标定一致; 原位锁定会沿用上次遗留姿态, 若头部被转歪将导致
SLAM/导航系统性漂移。运动发生在小脑启动前(机器人未站立), 物理安全。

退出码:
  0 = 成功锁定
  2 = 串口打开失败 / 舵机无应答
  3 = 依赖缺失 (python3-serial)

注意:
  - 锁定状态保存在舵机内部，本脚本退出 / 串口关闭后依然保持；
    刻意不做退出释放（与 vendor demo 相反）。
  - 同一串口同时只能被一个进程打开。
"""
import argparse
import sys
import time

try:
    import serial  # noqa: F401  (提前给出可读的缺依赖提示)
except ImportError:
    print("[neck_hold][ERROR] 缺少 pyserial，请安装: sudo apt install python3-serial")
    sys.exit(3)

from htd35h_controller import HTD35HController

# 舵机 ID 与中位（与 servo_control_new 仓库《使用说明》一致）
NECK_IDS = [1, 2]          # ID1=头部左右(0-1000), ID2=点头(0-650)


def main():
    ap = argparse.ArgumentParser(
        description="颈部双舵机锁定: 小脑/大脑运行期间保持头部不动")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="STM32 扩展板串口 (默认 /dev/ttyUSB0, 建议 udev 别名 /dev/neck_servo)")
    ap.add_argument("--mode", choices=["lock", "home"], default="home",
                    help="home=回中位后锁定(默认, Mid-360在头上需外参确定性); "
                         "lock=当前位置直接锁定(零运动, 应急用)")
    ap.add_argument("--pose1", type=int, default=500,
                    help="home 模式下 ID1(头部左右) 目标脉冲, 默认500; 若雷达建图时"
                         "头部姿态非中位, 用此参数对齐标定姿态")
    ap.add_argument("--pose2", type=int, default=500,
                    help="home 模式下 ID2(点头) 目标脉冲, 默认500; 注意 ID2 限位650")
    args = ap.parse_args()

    try:
        ctrl = HTD35HController(port=args.port, baudrate=115200)
    except Exception as e:
        print(f"[neck_hold][ERROR] 串口 {args.port} 打开失败: {e}")
        print("  检查: 设备存在? 权限(dialout 组/udev 规则)? 是否被其他进程占用?")
        sys.exit(2)

    try:
        # 1) 连通性检查: 读两个舵机当前位置
        pos = ctrl.read_all_positions(NECK_IDS)
        if not pos:
            print("[neck_hold][ERROR] 两个舵机均无应答 (串口通但总线不通)")
            print("  检查: 舵机供电? STM32 扩展板到舵机的总线接线?")
            sys.exit(2)
        for sid in NECK_IDS:
            if sid not in pos:
                print(f"[neck_hold][WARN] 舵机 ID{sid} 无应答，仍会对其发送锁定指令")
        print(f"[neck_hold] 连接成功, 当前位置: {pos}")

        # 2) 可选: 回中位 (set_position 到位后自动保持); 目标脉冲可配, 对齐雷达标定姿态
        if args.mode == "home":
            pose = {NECK_IDS[0]: args.pose1, NECK_IDS[1]: args.pose2}
            print(f"[neck_hold] 回中位 {pose}, 约 1s ...")
            ctrl.set_position(1.0, list(pose.items()))
            time.sleep(1.2)

        # 3) 扭矩锁定 (SubCmd 0x0B): 在当前位置主动保持, 受扰自动回位
        ctrl.enable_torque_all(NECK_IDS, True)
        time.sleep(0.3)

        # 4) 回读确认
        after = ctrl.read_all_positions(NECK_IDS)
        print(f"[neck_hold] 锁定完成, 锁定后位置: {after}")
        # 注意: 此处刻意不释放扭矩 —— 退出后头部保持锁定
    finally:
        ctrl.close()  # 仅关串口; 锁定状态在舵机内部, 不受影响


if __name__ == "__main__":
    main()
