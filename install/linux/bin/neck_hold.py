#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
颈部双舵机扭矩使能 — 小脑启动前置步骤
=====================================

背景:
  F1 颈部装有两个 HTD-35H 总线舵机 (ID1=头部左右, ID2=点头, 限位650)，
  由小脑 Intel 主机经 CH340 串口 -> STM32 扩展板控制。

用法:
  python3 neck_hold.py                          # 默认: /dev/ttyUSB0
  python3 neck_hold.py --port /dev/neck_servo   # 使用 udev 别名

退出码:
  0 = 成功
  2 = 串口打开失败
  3 = 依赖缺失 (python3-serial)

注意:
  - 仅执行扭矩使能(SubCmd 0x0B), 不回中位、不读位置、不做其他操作。
  - 使能状态保存在舵机内部，本脚本退出 / 串口关闭后依然保持；
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

# 舵机 ID（与 servo_control_new 仓库《使用说明》一致）
NECK_IDS = [1, 2]          # ID1=头部左右(0-1000), ID2=点头(0-650)


def main():
    ap = argparse.ArgumentParser(
        description="颈部双舵机扭矩使能: 保持头部不动")
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="STM32 扩展板串口 (默认 /dev/ttyUSB0, 建议 udev 别名 /dev/neck_servo)")
    args = ap.parse_args()

    try:
        ctrl = HTD35HController(port=args.port, baudrate=115200)
    except Exception as e:
        print(f"[neck_hold][ERROR] 串口 {args.port} 打开失败: {e}")
        print("  检查: 设备存在? 权限(dialout 组/udev 规则)? 是否被其他进程占用?")
        sys.exit(2)

    try:
        # 扭矩使能 (SubCmd 0x0B): 在当前位置主动保持, 受扰自动回位
        ctrl.enable_torque_all(NECK_IDS, True)
        time.sleep(0.3)
        print(f"[neck_hold] 扭矩使能完成, 舵机: {NECK_IDS}")
        # 注意: 此处刻意不释放扭矩 —— 退出后头部保持锁定
    finally:
        ctrl.close()  # 仅关串口; 使能状态在舵机内部, 不受影响


if __name__ == "__main__":
    main()
