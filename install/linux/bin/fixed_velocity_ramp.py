#!/usr/bin/env python3
"""
固定速度斜坡脚本——仿真/真机中复刻"按下 X/RT 前进，20 秒后自动停止"的简洁场景。

行为：
  启动后（按 Enter）：持续发布 x=0.4 m/s（y=0, wz=0）
  20 秒后：x 降为 0（y=0, wz=0），保持发布直到退出

用法：
  1. 启动仿真（run_sim.sh），先让机器人进入 walk_leg：
     ros2 topic pub /walk_mode std_msgs/msg/Float32 '{data: 0.0}' -t 2
  2. 运行本脚本：python3 fixed_velocity_ramp.py
  3. 按 Enter 开始发速度；20s 后自动归零；Ctrl+C 退出
"""

import time
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ── 参数 ──
VX = 0.4             # 前进速度 m/s
VY = 0.0             # 侧向速度 m/s（固定 0，无横移）
DURATION_FORWARD = 20.0  # 前进时长 s，之后 x 归 0
PUB_RATE = 20        # 发布频率 Hz
MODE_RETRY = 1.0     # 切 walk 模式重复发布时长 s
WALK_SETTLE = 2.0    # 切到 walk 后等待步态稳定时长 s（避免刚切模式就发速度）
# ─────────────

class FixedVelocityRamp(Node):
    def __init__(self):
        super().__init__("fixed_velocity_ramp")
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)
        self.walk_pub = self.create_publisher(Float32, "/walk_mode", 1)
        self.first_run = True    # 首次 tick 先切 walk 模式
        self.t0 = 0.0
        self.running = False
        self.create_timer(1.0 / PUB_RATE, self.tick)
        self.get_logger().info(
            f"固定速度斜坡脚本已启动。\n"
            f"  启动即自动：发 /walk_mode 切到 walk（若当前非 walk）→ "
            f"等 {WALK_SETTLE}s 稳定 → 发 x={VX} m/s 持续 {DURATION_FORWARD}s → x=0\n"
            f"  y 始终为 {VY}（无侧向）。Ctrl+C 退出"
        )

    def _send_walk(self):
        # 持续发 /walk_mode MODE_RETRY 秒，防状态机/节流器丢事件
        msg = Float32()
        msg.data = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < MODE_RETRY:
            self.walk_pub.publish(msg)
            time.sleep(1.0 / PUB_RATE)
        self.get_logger().info("已发送 /walk_mode，等待步态稳定 ...")
        time.sleep(WALK_SETTLE)

    def _publish(self, vx):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = VY
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

    def start(self):
        if self.running:
            return
        self.running = True
        self.first_run = True
        self.get_logger().info(f"== 启动：先切 walk 模式，随后 x={VX} m/s，{DURATION_FORWARD}s 后归零 ==")

    def tick(self):
        if not self.running:
            return
        if self.first_run:
            # 首次：先切到 walk 模式并等稳定，再开始发速度
            self.first_run = False
            self._send_walk()
            self.t0 = time.monotonic()
            self.get_logger().info(f"进入 walk，开始发 x={VX} m/s ...")
            return
        elapsed = time.monotonic() - self.t0
        if elapsed < DURATION_FORWARD:
            self._publish(VX)
        else:
            self._publish(0.0)   # 20s 后 x=0

def main():
    rclpy.init()
    node = FixedVelocityRamp()
    try:
        node.start()   # 启动即开始，无需按 Enter
        spin_thread = __import__("threading").Thread(
            target=lambda: rclpy.spin(node), daemon=True)
        spin_thread.start()
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()