#!/usr/bin/env python3
"""
自动行走脚本 - 无手柄时按时间线控制 MuJoCo 仿真中的机器人
与 C++ JoyStickModule 的 LT 自动行走逻辑一致（stand 保持 5s）

用法：
  在 run_sim.sh 启动后，另开一个终端执行：
    python3 auto_walk_ctrl.py

时间线（t = 脚本启动后的秒数）:
  ┌──────────────────────────────────────────────────────────────┐
  │  0 ~ 1 s    : 发布 /stand_mode（确保站立）                    │
  │  0 ~ 5 s    : 速度 0（stand 保持）                            │
  │  5 ~ 6 s    : 发布 /walk_mode 进入 walk_leg                   │
  │  5 ~ 8 s    : 直接 0.4 m/s（无加速段）                        │
  │  8 ~ 12 s   : 匀减速 0.4 → 0 m/s（0.1 m/s²，共 4 s）          │
  │ 12 ~ 13 s   : 发布 /stand_mode 回到 stand                     │
  └──────────────────────────────────────────────────────────────┘

依赖：
  source /opt/ros/<distro>/setup.bash   # 或已 source ROS2 环境
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

# ── 时间线参数（与 C++ LT 自动行走一致）──────────────────
DELAY_WALK = 5.0       # 进入 walk_leg 前的 stand 保持时长 s
VEL_MAX = 0.4          # 最大线速度 m/s
VEL_RAMP = 0.1         # 减速变化率 m/s²（无加速段）
VEL_DIRECTION = 1.0    # 运动方向：+1 前进，-1 后退
HOLD_TIME = 3.0        # 保持最大速度的时长 s
PUB_RATE = 20          # 速度发布频率 Hz
MODE_RETRY_TIME = 1.0  # 模式话题重复发布时长 s（防丢）
# ──────────────────────────────────────────────────────────


class AutoWalkCtrl(Node):
    def __init__(self):
        super().__init__("auto_walk_ctrl")

        self.stand_pub = self.create_publisher(Float32, "/stand_mode", 1)
        self.walk_pub = self.create_publisher(Float32, "/walk_mode", 1)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)

        self.dec_time = VEL_MAX / VEL_RAMP  # 减速所需时长 = 4.0 s
        self.hold_end = DELAY_WALK + HOLD_TIME  # 8.0 s
        self.dec_end = self.hold_end + self.dec_time  # 12.0 s

        self.t0 = time.monotonic()
        self.walk_pub_until = -1.0
        self.stand_back_until = -1.0
        self.last_log_t = 0.0
        self.done = False

        self.create_timer(1.0 / PUB_RATE, self.tick)
        self.get_logger().info(
            f"自动行走脚本已启动: stand保持{DELAY_WALK}s, "
            f"直接{VEL_MAX}m/s, 保持{HOLD_TIME}s, "
            f"减速{self.dec_time}s({VEL_RAMP}m/s²), 回stand"
        )

    def target_velocity(self, t):
        """按时间线返回目标线速度 m/s（直接满速，无加速段）"""
        if t < DELAY_WALK:
            return 0.0
        if t < self.hold_end:
            return VEL_DIRECTION * VEL_MAX
        if t < self.dec_end:
            return VEL_DIRECTION * (VEL_MAX - VEL_RAMP * (t - self.hold_end))
        return 0.0

    def tick(self):
        t = time.monotonic() - self.t0

        # 0 ~ 1s：发布 /stand_mode 确保站立
        if t < MODE_RETRY_TIME:
            self._pub_float(self.stand_pub)

        # 5s 后发布 /walk_mode 进入 walk_leg（重复 1s 防丢）
        if self.walk_pub_until < 0.0 and t >= DELAY_WALK:
            self.walk_pub_until = t + MODE_RETRY_TIME
            self.get_logger().info(f"t={t:.2f}s -> 发布 /walk_mode 进入 walk_leg")
        if self.walk_pub_until > 0.0 and t < self.walk_pub_until:
            self._pub_float(self.walk_pub)

        v = self.target_velocity(t)

        # 减速结束后发布 /stand_mode 回到 stand（重复 1s）
        if t >= self.dec_end and t < self.dec_end + MODE_RETRY_TIME:
            self._pub_float(self.stand_pub)

        # 发布当前速度（20 Hz）
        msg = Twist()
        msg.linear.x = v
        self.vel_pub.publish(msg)

        # 每秒打印一次进度
        if t - self.last_log_t >= 1.0:
            self.last_log_t = t
            self.get_logger().info(f"t={t:6.2f}s  linear.x={v:+.3f} m/s")

        # 时间线完成后自动退出
        if t >= self.dec_end + MODE_RETRY_TIME:
            self.done = True

    def _pub_float(self, pub):
        msg = Float32()
        msg.data = 0.0
        pub.publish(msg)


def main():
    rclpy.init()
    node = AutoWalkCtrl()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.done:
            node.get_logger().info("时间线执行完毕，已回到 stand，脚本退出")
    except KeyboardInterrupt:
        pass
    finally:
        zero = Twist()
        node.vel_pub.publish(zero)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
