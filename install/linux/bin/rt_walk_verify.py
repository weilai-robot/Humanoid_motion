#!/usr/bin/env python3
"""
RT 直线行走验证脚本（无手柄版）

复刻 C++ JoyStickModule 中 RT 偏航闭环的 PD 控制逻辑，
通过 ROS2 订阅 /imu/data，发布 /cmd_vel_limiter，用于仿真验证。

用法：
  1. 启动仿真（run_sim.sh）
  2. 按 z → s 进入 stand
  3. 另开终端运行：python3 rt_walk_verify.py
  4. 按 Enter 开始直线行走（自动切 walk_leg + IMU 纠偏）
  5. 再按 Enter 停止（自动回 stand）
  6. Ctrl+C 退出

验证要点：
  - 观察 robot 是否走直线（偏航被纠正回来）
  - 日志输出 yaw_err 和 cmd_wz，确认 PD 在工作
  - 如果振荡，调小 yaw_kp 或调大 yaw_kd
"""

import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ── PID 控制参数（与 joy_x1.yaml 中 rt_auto_walk 一致）──
LINEAR_X = 0.4         # 前进速度 m/s
YAW_KP = 0.8           # 偏航 P 增益
YAW_KI = 0.3           # 偏航 I 增益（消除稳态误差）
YAW_KD = 0.2           # 偏航 D 增益（阻尼）
MAX_ANGULAR_Z = 0.5    # 最大转弯角速度 rad/s
I_LIMIT = 0.3          # 积分项限幅 rad/s（防积分饱和）
PUB_RATE = 20          # 闭环发布频率 Hz
MODE_RETRY = 1.0       # 模式切换重复发布时长 s
WALK_SETTLE = 1.0      # 进入 walk_leg 后等待稳定 s
# ──────────────────────────────────────────────────────────


def quat_to_yaw(qw, qx, qy, qz):
    """四元数 → 偏航角 yaw（rad），与 rotation_tools.h QuatToXyz 一致"""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    """角度归一化到 [-π, π]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RtWalkVerify(Node):
    def __init__(self):
        super().__init__("rt_walk_verify")

        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.on_imu, 10)
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)
        self.walk_pub = self.create_publisher(Float32, "/walk_mode", 1)
        self.stand_pub = self.create_publisher(Float32, "/stand_mode", 1)

        self.latest_yaw = 0.0
        self.latest_gyro_z = 0.0
        self.imu_received = False
        self.lock = threading.Lock()

        self.walk_active = False
        self.mode_switching = False       # 模式切换中，暂停闭环
        self.yaw_target = 0.0
        self.yaw_integral = 0.0           # 积分项累积
        self.log_counter = 0

        self.create_timer(1.0 / PUB_RATE, self.control_tick)
        self.get_logger().info(
            "RT 直线行走验证脚本已启动，等待 IMU 数据...\n"
            "  按 Enter 开始/停止行走，Ctrl+C 退出"
        )

    def on_imu(self, msg):
        with self.lock:
            self.latest_yaw = quat_to_yaw(
                msg.orientation.w, msg.orientation.x,
                msg.orientation.y, msg.orientation.z
            )
            self.latest_gyro_z = msg.angular_velocity.z
            self.imu_received = True

    def start_walk(self):
        """启动行走：切 walk_leg → 等稳定 → 记录目标航向 → 开始闭环"""
        self.mode_switching = True

        # 1. 发布 /walk_mode 持续 MODE_RETRY 秒
        self.get_logger().info("切换到 walk_leg ...")
        msg = Float32()
        msg.data = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < MODE_RETRY:
            self.walk_pub.publish(msg)
            time.sleep(0.05)

        # 2. 等待步态稳定
        self.get_logger().info(f"等待 {WALK_SETTLE}s 步态稳定 ...")
        time.sleep(WALK_SETTLE)

        # 3. 记录当前 yaw 作为目标航向
        with self.lock:
            self.yaw_target = self.latest_yaw
        self.yaw_integral = 0.0  # 重置积分项

        self.walk_active = True
        self.mode_switching = False
        self.get_logger().info(
            f"======== 行走开始 ========\n"
            f"  目标航向 yaw0 = {self.yaw_target:.4f} rad ({math.degrees(self.yaw_target):.1f} deg)\n"
            f"  前进速度 = {LINEAR_X} m/s\n"
            f"  PD 参数: Kp={YAW_KP}, Kd={YAW_KD}"
        )

    def stop_walk(self):
        """停止行走：发零速度 → 切 stand"""
        self.walk_active = False
        self.mode_switching = True

        # 1. 发布零速度
        msg = Twist()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.5:
            self.vel_pub.publish(msg)
            time.sleep(0.05)

        # 2. 发布 /stand_mode
        self.get_logger().info("切换到 stand ...")
        smsg = Float32()
        smsg.data = 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < MODE_RETRY:
            self.stand_pub.publish(smsg)
            time.sleep(0.05)

        self.mode_switching = False
        self.get_logger().info("======== 行走停止，已回 stand ========")

    def control_tick(self):
        if not self.walk_active or self.mode_switching:
            return

        with self.lock:
            yaw_current = self.latest_yaw
            gyro_z = self.latest_gyro_z

        # 偏航误差
        yaw_err = normalize_angle(self.yaw_target - yaw_current)

        # 积分项累积（带限幅防饱和）
        dt = 1.0 / PUB_RATE
        self.yaw_integral += yaw_err * dt
        self.yaw_integral = max(-I_LIMIT / YAW_KI, min(I_LIMIT / YAW_KI, self.yaw_integral))

        # PID 控制
        cmd_wz = YAW_KP * yaw_err + YAW_KD * (-gyro_z) + YAW_KI * self.yaw_integral
        cmd_wz = max(-MAX_ANGULAR_Z, min(MAX_ANGULAR_Z, cmd_wz))

        # 发布速度
        msg = Twist()
        msg.linear.x = LINEAR_X
        msg.angular.z = cmd_wz
        self.vel_pub.publish(msg)

        # 每 20 帧（约 1 秒）打印一次
        self.log_counter += 1
        if self.log_counter % 20 == 0:
            self.get_logger().info(
                f"yaw_err={yaw_err:+.4f} rad ({math.degrees(yaw_err):+6.1f} deg)  "
                f"gyro_z={gyro_z:+.4f}  integral={self.yaw_integral:+.4f}  cmd_wz={cmd_wz:+.4f} rad/s"
            )


def main():
    rclpy.init()
    node = RtWalkVerify()

    # ROS2 spin 在独立线程
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # 等待 IMU 数据
    while not node.imu_received and rclpy.ok():
        time.sleep(0.1)
    if node.imu_received:
        node.get_logger().info("IMU 数据已收到，可以开始验证！")

    # 主线程：等待 Enter 键控制开始/停止
    try:
        while rclpy.ok():
            input()
            if not node.walk_active:
                node.start_walk()
            else:
                node.stop_walk()
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        if node.walk_active:
            node.stop_walk()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
