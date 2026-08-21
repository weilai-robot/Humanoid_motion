#!/usr/bin/env python3
"""
LT 刹车模拟脚本（无手柄版）——精确复刻 C++ JoyStickModule 的 LT 刹车逻辑

复刻自 motion_control/module/joy_stick_module/src/joy_stick_module.cc L190-249：
  - 触发：按下 LT（或按 Enter），捕获上一帧实际发布的速度
  - 减速：scale = (1 - t/duration)^2 二次凹函数，时长默认 8s（joy_x1.yaml lt_brake.duration）
  - 输出：linear.x 按曲线衰减；linear.y/angular.z 全部置 0（刹车中不转向）
  - 结束：发 /stand_mode 回站立

用法：
  1. 启动仿真（run_sim.sh），并先让机器人在走（rt_walk_verify.py 或手动发速度）
  2. 另开终端：python3 lt_brake_sim.py
  3. 按 Enter 触发刹车（脚本会捕获当前正在发布的速度作为初始速度）
  4. 观察 8 秒内速度按二次曲线归零、然后回 stand
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# ── 参数（与 joy_x1.yaml lt_brake 一致）──
BRAKE_DURATION = 8.0     # 刹车总时长 s
PUB_RATE = 20            # 发布频率 Hz（与手柄主循环一致）
MODE_RETRY = 1.0         # /stand_mode 重复发布时长 s（防丢）
# ──────────────────────────────────────────


class LtBrakeSim(Node):
    def __init__(self):
        super().__init__("lt_brake_sim")
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)
        self.stand_pub = self.create_publisher(Float32, "/stand_mode", 1)

        # 刹车前捕获的速度（按下 Enter 时读取话题上的最新值）
        self.vel_sub = self.create_subscription(Twist, "/cmd_vel_limiter", self.on_vel, 1)
        self.latest_vx = 0.0
        self.latest_vy = 0.0
        self.latest_wz = 0.0

        self.brake_active = False
        self.brake_t0 = 0.0
        self.init_vx = 0.0
        self.init_vy = 0.0
        self.init_wz = 0.0
        self.log_counter = 0

        self.create_timer(1.0 / PUB_RATE, self.tick)
        self.get_logger().info(
            "LT 刹车模拟脚本已启动。\n"
            "  请先让机器人在走（rt_walk_verify.py 或手动发 /cmd_vel_limiter）\n"
            "  按 Enter 触发刹车（8s 二次凹减速 → 回 stand），Ctrl+C 退出"
        )

    def on_vel(self, msg):
        # 记录刹车前最后一条速度（复刻 C++ 的 last_vel_x_/y_/wz_）
        self.latest_vx = msg.linear.x
        self.latest_vy = msg.linear.y
        self.latest_wz = msg.angular.z

    def trigger_brake(self):
        """按下 LT：捕获当前速度，启动二次减速"""
        if self.brake_active:
            return
        self.brake_active = True
        self.brake_t0 = time.monotonic()
        self.init_vx = self.latest_vx   # 捕获上一帧实际速度
        self.init_vy = self.latest_vy
        self.init_wz = self.latest_wz
        self.get_logger().info(
            f"[LT] 刹车 STARTED, 初始速度 vx={self.init_vx:.3f} vy={self.init_vy:.3f} wz={self.init_wz:.3f}, "
            f"时长 {BRAKE_DURATION}s (二次凹减速)"
        )

    def tick(self):
        if not self.brake_active:
            return

        elapsed = time.monotonic() - self.brake_t0
        linear_t = min(elapsed / BRAKE_DURATION, 1.0)   # 先 clamp 到 [0,1] 再平方，避免超时后反弹
        scale = (1.0 - linear_t) ** 2                   # 复刻 C++ L217

        if scale <= 0.0:
            self.brake_active = False
            # 刹停 → 回 stand（复刻 C++ L218-225）
            self.get_logger().info("[LT] 刹车 COMPLETE, back to stand")
            smsg = Float32()
            smsg.data = 0.0
            t0 = time.monotonic()
            while time.monotonic() - t0 < MODE_RETRY:
                self.stand_pub.publish(smsg)
                time.sleep(0.05)
            # 补发一条零速度确保停稳
            msg = Twist()
            self.vel_pub.publish(msg)
            return

        # 二次减速输出（复刻 C++ L228-239）：仅衰减 vx，vy/wz 置 0 防转向
        msg = Twist()
        msg.linear.x = self.init_vx * scale
        msg.linear.y = 0.0
        msg.angular.z = 0.0
        self.vel_pub.publish(msg)

        self.log_counter += 1
        if self.log_counter % 20 == 0:   # 每 1s 打一次
            self.get_logger().info(f"[LT] t={elapsed:.1f}s scale={scale:.2f} vx={msg.linear.x:.3f}")


def main():
    rclpy.init()
    node = LtBrakeSim()

    import threading
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            input()
            node.trigger_brake()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
