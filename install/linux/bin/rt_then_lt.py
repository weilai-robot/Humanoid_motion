#!/usr/bin/env python3
"""
RT→LT 联合模拟脚本——仿真中复刻"先按 RT 直线行走，过一段时间再按 LT 刹车"的完整场景

流程（按 Enter 启动，全程自动）：
  t=0     : 切 walk_leg（重复发 1s）→ 等 1s 稳定 → 锁存目标航向
  t=2s 起 : RT 偏航闭环（0.4 m/s + yaw PID），持续 RT_DURATION 秒
  t=2+RT_DURATION: 自动触发 LT——捕获当前速度，8s 二次凹减速 ((1-t/8)^2)
  刹停    : 发 /stand_mode 回站立，结束

复刻自：
  - RT: install/linux/bin/rt_walk_verify.py（偏航 PID 闭环）
  - LT: install/linux/bin/lt_brake_sim.py（二次凹刹车）
用法：
  1. 启动仿真（run_sim.sh）
  2. python3 rt_then_lt.py
  3. 按 Enter 开始完整序列，观察 RT 直行 → LT 刹停 → 回 stand
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ── RT 参数（与 rt_walk_verify.py 一致；如需对齐真机请改成 yaml 的 1.2/0.5/0.3）──
LINEAR_X = 0.4          # 前进速度 m/s
YAW_KP = 0.8
YAW_KI = 0.3
YAW_KD = 0.2
MAX_ANGULAR_Z = 0.5
I_LIMIT = 0.3
# ── LT 参数（与 lt_brake_sim.py / joy_x1.yaml lt_brake 一致）──
BRAKE_DURATION = 8.0    # 刹车总时长 s
# ── 场景时间线 ──
RT_DURATION = 15.0      # RT 直线行走持续时长 s（改这里控制"过一段时间再按 LT"）
MODE_RETRY = 1.0        # 模式切换重复发布时长 s
WALK_SETTLE = 1.0       # 进入 walk_leg 后等待稳定 s
PUB_RATE = 20           # 闭环发布频率 Hz
# ──────────────────────────────────────────


def quat_to_yaw(qw, qx, qy, qz):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RtThenLt(Node):
    def __init__(self):
        super().__init__("rt_then_lt")
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.on_imu, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)
        self.walk_pub = self.create_publisher(Float32, "/walk_mode", 1)
        self.stand_pub = self.create_publisher(Float32, "/stand_mode", 1)

        self.lock = threading.Lock()
        self.latest_yaw = 0.0
        self.latest_gyro_z = 0.0
        self.imu_received = False

        # 状态机
        self.phase = "idle"       # idle / rt / lt / done
        self.yaw_target = 0.0
        self.yaw_integral = 0.0
        self.phase_t0 = 0.0
        self.phase_started = False
        self.lt_init_vx = 0.0
        self.log_counter = 0

        self.create_timer(1.0 / PUB_RATE, self.tick)
        self.get_logger().info(
            f"RT→LT 联合模拟已启动。\n"
            f"  按 Enter 开始：RT 直行 {RT_DURATION}s → LT 刹车 {BRAKE_DURATION}s → 回 stand\n"
            f"  Ctrl+C 退出"
        )

    def on_imu(self, msg):
        with self.lock:
            self.latest_yaw = quat_to_yaw(
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            self.latest_gyro_z = msg.angular_velocity.z
            self.imu_received = True

    def start(self):
        if self.phase != "idle":
            return
        if not self.imu_received:
            self.get_logger().warn("IMU 尚未就绪，等待数据...")
            return
        self.phase = "rt"
        self.phase_t0 = time.monotonic()
        self.phase_started = False
        self.get_logger().info("======== 序列开始：切 walk_leg ========")

    # ── RT 阶段 ──
    def phase_rt(self, t):
        if t < MODE_RETRY:
            # 切 walk_leg（重复发防丢）
            msg = Float32()
            msg.data = 0.0
            self.walk_pub.publish(msg)
        elif not self.phase_started:
            if t < MODE_RETRY + WALK_SETTLE:
                return  # 等步态稳定
            # 锁存目标航向，开始闭环
            with self.lock:
                self.yaw_target = self.latest_yaw
            self.yaw_integral = 0.0
            self.phase_started = True
            self.get_logger().info(
                f"======== RT 直行开始（{RT_DURATION}s）========\n"
                f"  目标航向 yaw0 = {self.yaw_target:.4f} rad, vx = {LINEAR_X} m/s"
            )
            return
        else:
            self.rt_control()
            if t - (MODE_RETRY + WALK_SETTLE) >= RT_DURATION:
                # RT 结束 → 触发 LT
                self.phase = "lt"
                self.phase_t0 = time.monotonic()
                self.lt_init_vx = LINEAR_X   # RT 一直发 0.4
                self.get_logger().info("======== RT 结束，触发 LT 刹车 ========")

    def rt_control(self):
        with self.lock:
            yaw_current, gyro_z = self.latest_yaw, self.latest_gyro_z
        yaw_err = normalize_angle(self.yaw_target - yaw_current)
        dt = 1.0 / PUB_RATE
        self.yaw_integral += yaw_err * dt
        self.yaw_integral = max(-I_LIMIT / YAW_KI, min(I_LIMIT / YAW_KI, self.yaw_integral))
        cmd_wz = YAW_KP * yaw_err + YAW_KD * (-gyro_z) + YAW_KI * self.yaw_integral
        cmd_wz = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, cmd_wz))

        msg = Twist()
        msg.linear.x = LINEAR_X
        msg.angular.z = cmd_wz
        self.vel_pub.publish(msg)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"[RT] yaw_err={math.degrees(yaw_err):+6.1f}° cmd_wz={cmd_wz:+.3f} rad/s")

    # ── LT 阶段 ──
    def phase_lt(self, t):
        linear_t = min(t / BRAKE_DURATION, 1.0)   # 先 clamp 到 [0,1] 再平方，避免 t>duration 后反弹
        scale = (1.0 - linear_t) ** 2
        if scale <= 0.0:
            # 刹停 → 回 stand
            self.phase = "done"
            smsg = Float32()
            smsg.data = 0.0
            for _ in range(int(PUB_RATE * MODE_RETRY)):
                self.stand_pub.publish(smsg)
                time.sleep(1.0 / PUB_RATE)
            zero = Twist()
            self.vel_pub.publish(zero)
            self.get_logger().info("======== LT 刹停完成，已回 stand，序列结束 ========")
            return

        msg = Twist()
        msg.linear.x = self.lt_init_vx * scale
        msg.angular.z = 0.0   # 刹车中不转向（复刻 LT）
        self.vel_pub.publish(msg)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(f"[LT] t={t:.1f}s scale={scale:.2f} vx={msg.linear.x:.3f}")

    def tick(self):
        if self.phase in ("idle", "done"):
            return
        t = time.monotonic() - self.phase_t0
        if self.phase == "rt":
            self.phase_rt(t)
        elif self.phase == "lt":
            self.phase_lt(t)


def main():
    rclpy.init()
    node = RtThenLt()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            input()
            node.start()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
