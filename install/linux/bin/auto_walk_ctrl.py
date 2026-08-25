#!/usr/bin/env python3
"""
RT→LT 自动模拟脚本（无手柄版）——复刻手柄"按 RT 直行 10s，再按 LT 刹车"的完整流程

复刻自 C++ JoyStickModule（joy_stick_module.cc）:
  RT (L251-359): 开启时锁存目标航向 → 持续 1s 重发 /walk_mode → 20Hz 偏航 PID 闭环
                 vx=0.4, vy=0, wz=Kp·err+Ki·∫+Kd·(-gyro_z)，限幅 ±0.6
  LT (L190-249): 捕获当前速度 → 8s 二次凹减速 (1-t/8)²，vy/wz 置 0
                 → 刹停后发 /stand_mode 回站立

PID 参数取自 joy_x1.yaml rt_auto_walk（Kp=1.2 / Ki=0.5 / Kd=0.3 / max_wz=0.6）。

时间线（按 Enter 启动，t 为脚本启动后秒数）:
  ┌────────────────────────────────────────────────────────────┐
  │ 0 ~ 1 s    : 持续发 /walk_mode（切 walk_leg，防丢重发）     │
  │ 1 ~ 2 s    : 等待步态稳定（RT_SETTLE）                      │
  │ 2 s        : 锁存当前 yaw 为目标航向，RT 闭环开始           │
  │ 2 ~ 12 s   : RT 直线行走（PID 纠偏，RT_DURATION=10s）       │
  │ 12 s       : 触发 LT —— 捕获当前速度                        │
  │ 12 ~ 16 s  : LT 二次凹减速 (1-t/4)²，wz 恒 0                │
  │ 16 s       : 刹停 → 发 /stand_mode 回 stand，结束           │
  └────────────────────────────────────────────────────────────┘

用法：
  1. 启动仿真（run_sim.sh），机器人处于 stand
  2. 另开终端：python3 auto_walk_ctrl.py
  3. 按 Enter 开始；观察 RT 直行 → LT 刹停 → 回 stand
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ── RT 参数（与 joy_x1.yaml rt_auto_walk 一致）──
LINEAR_X = 0.4         # 前进速度 m/s
LINEAR_Y = 0.0         # 侧向补偿 m/s（当前 yaml 为 0）
YAW_KP = 1.2           # 偏航 P 增益
YAW_KI = 0.5           # 偏航 I 增益
YAW_KD = 0.3           # 偏航 D 增益
MAX_ANGULAR_Z = 0.6    # 最大转弯角速度 rad/s
I_LIMIT = 0.5          # 积分项限幅 rad/s
MODE_RETRY = 1.0       # /walk_mode 重复发布时长 s（复刻 RT_MODE_RETRY）
WALK_SETTLE = 1.0      # 进入 walk_leg 后等待稳定 s
# ── LT 参数（测试用 4s 刹车；真机 yaml lt_brake.duration 为 8.0）──
BRAKE_DURATION = 4.0   # 刹车总时长 s
# ── 场景时间线 ──
RT_DURATION = 10.0     # RT 直线行走持续时长 s（"按 RT 10 秒后按 LT"）
PUB_RATE = 20          # 发布频率 Hz（与手柄主循环 freq 一致）


def quat_to_yaw(qw, qx, qy, qz):
    """四元数 → yaw，与 rotation_tools.h QuatToXyz 一致"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class AutoRtLt(Node):
    def __init__(self):
        super().__init__("auto_walk_ctrl")
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.on_imu, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)
        self.walk_pub = self.create_publisher(Float32, "/walk_mode", 1)
        self.stand_pub = self.create_publisher(Float32, "/stand_mode", 1)

        self.lock = threading.Lock()
        self.latest_yaw = 0.0
        self.latest_gyro_z = 0.0
        self.imu_received = False

        self.phase = "idle"        # idle / rt / lt / done
        self.phase_t0 = 0.0
        self.yaw_target = 0.0
        self.yaw_integral = 0.0
        self.rt_started = False
        self.lt_init_vx = 0.0
        self.log_counter = 0

        self.create_timer(1.0 / PUB_RATE, self.tick)
        self.get_logger().info(
            "RT→LT 自动模拟脚本已启动（等 IMU 数据）。\n"
            f"  按 Enter 开始：RT 直行 {RT_DURATION}s → LT 刹车 {BRAKE_DURATION}s → 回 stand\n"
            "  Ctrl+C 退出"
        )

    def on_imu(self, msg):
        with self.lock:
            self.latest_yaw = quat_to_yaw(
                msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z)
            self.latest_gyro_z = msg.angular_velocity.z
            self.imu_received = True

    # ── RT 阶段（复刻 C++ L251-359）──
    def phase_rt(self, t):
        if t < MODE_RETRY:
            # 启动阶段：持续重发 /walk_mode 切 walk_leg
            msg = Float32()
            msg.data = 0.0
            self.walk_pub.publish(msg)
            return
        if not self.rt_started:
            if t < MODE_RETRY + WALK_SETTLE:
                return  # 等步态稳定
            # 锁存当前 yaw 为目标航向，清积分（复刻 RT 开启）
            with self.lock:
                self.yaw_target = self.latest_yaw
            self.yaw_integral = 0.0
            self.rt_started = True
            self.get_logger().info(
                f"[RT] 直行开始（{RT_DURATION}s）: yaw0={self.yaw_target:.4f} rad "
                f"({math.degrees(self.yaw_target):.1f} deg), vx={LINEAR_X} m/s")
            return

        # 偏航 PID 闭环（复刻 C++ L308-349）
        with self.lock:
            yaw_current, gyro_z = self.latest_yaw, self.latest_gyro_z
        yaw_err = normalize_angle(self.yaw_target - yaw_current)
        dt = 1.0 / PUB_RATE
        self.yaw_integral += yaw_err * dt
        self.yaw_integral = max(-I_LIMIT / YAW_KI, min(I_LIMIT / YAW_KI, self.yaw_integral))
        cmd_wz = YAW_KP * yaw_err + YAW_KI * self.yaw_integral + YAW_KD * (-gyro_z)
        cmd_wz = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, cmd_wz))

        msg = Twist()
        msg.linear.x = LINEAR_X
        msg.linear.y = LINEAR_Y
        msg.angular.z = cmd_wz
        self.vel_pub.publish(msg)

        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"[RT] yaw_err={math.degrees(yaw_err):+6.1f} deg  integral={self.yaw_integral:+.3f}  cmd_wz={cmd_wz:+.3f}")

        # RT 时长到 → 触发 LT（捕获当前速度，复刻 C++ L195-207）
        if t - (MODE_RETRY + WALK_SETTLE) >= RT_DURATION:
            self.lt_init_vx = LINEAR_X
            self.phase = "lt"
            self.phase_t0 = time.monotonic()
            self.get_logger().info(f"[LT] 刹车 STARTED, 初始 vx={self.lt_init_vx:.3f}, 时长 {BRAKE_DURATION}s")

    # ── LT 阶段（复刻 C++ L211-249）──
    def phase_lt(self, t):
        linear_t = min(t / BRAKE_DURATION, 1.0)   # 先 clamp 再平方，避免超时反弹
        scale = (1.0 - linear_t) ** 2
        if scale <= 0.0:
            # 刹停 → 回 stand（复刻 C++ L218-225）
            self.phase = "done"
            smsg = Float32()
            smsg.data = 0.0
            for _ in range(int(PUB_RATE * MODE_RETRY)):
                self.stand_pub.publish(smsg)
                time.sleep(1.0 / PUB_RATE)
            zero = Twist()
            self.vel_pub.publish(zero)
            self.get_logger().info("[LT] 刹停 COMPLETE, 已回 stand，序列结束")
            return

        # 二次凹减速：仅衰减 vx，vy/wz 置 0 防转向（复刻 C++ L228-238）
        msg = Twist()
        msg.linear.x = self.lt_init_vx * scale
        msg.angular.z = 0.0
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
    node = AutoRtLt()
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # 等待 IMU
    while not node.imu_received and rclpy.ok():
        time.sleep(0.1)

    try:
        while rclpy.ok():
            input()
            if node.phase == "idle":
                node.phase = "rt"
                node.phase_t0 = time.monotonic()
                node.rt_started = False
                node.get_logger().info("======== 序列开始：切 walk_leg ========")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
