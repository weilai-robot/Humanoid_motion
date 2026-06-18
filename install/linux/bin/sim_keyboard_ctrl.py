#!/usr/bin/env python3
"""
键盘模拟手柄控制器 - 用于在无手柄的 Docker/仿真环境中控制机器人

用法：
  在 run_sim.sh 启动后，另开一个终端执行：
    python3 sim_keyboard_ctrl.py

依赖：
  pip3 install rclpy  (通常随 ROS2 一起安装)
  source /opt/ros/<distro>/setup.bash

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  状态机流转（必须按顺序触发）:
    idle → [z] → zero → [s] → stand → [w] → walk_leg

  按键映射:
  ┌─────────────────────────────────────────────────┐
  │  i    → /idle_mode   (任意状态 → idle)          │
  │  z    → /zero_mode   (idle/keep/stand → zero)   │
  │  s    → /stand_mode  (zero → stand)             │
  │  w    → /walk_mode   (stand → walk_leg)         │
  │  a    → /walk_mode2  (stand → walk_leg_arm)     │
  │  k    → /keep_mode   (idle → keep)              │
  │  p    → /plan_mode   (stand/keep/walk → +plan)  │
  ├─────────────────────────────────────────────────┤
  │  速度控制 (发布到 /cmd_vel_limiter):            │
  │  ↑ / W(大写) → 前进  linear.x +0.1             │
  │  ↓ / S(大写) → 后退  linear.x -0.1             │
  │  ← / A(大写) → 左移  linear.y +0.1             │
  │  → / D(大写) → 右移  linear.y -0.1             │
  │  Q(大写)     → 左转  angular.z +0.1            │
  │  E(大写)     → 右转  angular.z -0.1            │
  │  空格         → 停止  (所有速度清零)             │
  ├─────────────────────────────────────────────────┤
  │  Ctrl+C / q  → 退出                            │
  └─────────────────────────────────────────────────┘
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""

import sys
import tty
import termios
import select
import threading
import time

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float32
    from geometry_msgs.msg import Twist
except ImportError:
    print("[ERROR] 未找到 rclpy，请先 source ROS2 环境：")
    print("  source /opt/ros/<distro>/setup.bash")
    sys.exit(1)


# ── 状态机描述（与 rl_x1_sim.yaml 保持一致）──────────────────────────
STATE_GRAPH = {
    "idle":         {"trigger": "/idle_mode",  "from": ["keep", "stand", "zero", "walk_leg", "walk_leg_arm", "stand_&_plan", "keep_&_plan", "walk_leg_&_plan"]},
    "keep":         {"trigger": "/keep_mode",  "from": ["idle", "stand", "zero", "walk_leg", "walk_leg_arm"]},
    "zero":         {"trigger": "/zero_mode",  "from": ["idle", "keep", "stand", "stand_&_plan"]},
    "stand":        {"trigger": "/stand_mode", "from": ["zero", "stand_&_plan", "keep_&_plan", "walk_leg_&_plan", "walk_leg", "walk_leg_arm"]},
    "walk_leg":     {"trigger": "/walk_mode",  "from": ["stand", "stand_&_plan", "walk_leg_&_plan", "walk_leg_arm"]},
    "walk_leg_arm": {"trigger": "/walk_mode2", "from": ["stand", "stand_&_plan", "walk_leg_&_plan", "walk_leg"]},
    "plan":         {"trigger": "/plan_mode",  "from": ["stand", "keep", "walk_leg"]},
}

KEY_TO_MODE = {
    'i': "idle",
    'z': "zero",
    's': "stand",
    'w': "walk_leg",
    'a': "walk_leg_arm",
    'k': "keep",
    'p': "plan",
}

VEL_STEP = 0.1
VEL_LIMIT = {"linear_x": 0.5, "linear_y": 0.3, "angular_z": 0.5}


class KeyboardController(Node):
    def __init__(self):
        super().__init__("sim_keyboard_ctrl")

        # 模式话题发布器
        self.mode_pubs = {}
        for mode_key, info in STATE_GRAPH.items():
            topic = info["trigger"]
            if topic not in self.mode_pubs:
                self.mode_pubs[topic] = self.create_publisher(Float32, topic, 1)

        # 速度话题发布器
        self.vel_pub = self.create_publisher(Twist, "/cmd_vel_limiter", 10)

        self.current_vel = Twist()
        self.current_state = "idle"  # 假设初始状态为 idle
        self.last_mode_time = 0.0

        # 定时发布当前速度（20Hz）
        self.vel_timer = self.create_timer(0.05, self._publish_vel)

        self.get_logger().info("键盘控制器已启动，请查看终端操作说明。")

    def _publish_vel(self):
        self.vel_pub.publish(self.current_vel)

    def trigger_mode(self, mode_key: str):
        """发布模式切换话题（带 1s 节流）"""
        now = time.time()
        if now - self.last_mode_time < 1.1:
            print(f"  [警告] 模式切换冷却中，请 1 秒后再试")
            return

        info = STATE_GRAPH.get(mode_key)
        if info is None:
            return

        topic = info["trigger"]
        pub = self.mode_pubs.get(topic)
        if pub:
            msg = Float32()
            msg.data = 0.0
            pub.publish(msg)
            self.last_mode_time = now
            print(f"  → 发布 {topic}  (当前假定状态: {self.current_state})")

    def update_velocity(self, axis: str, delta: float):
        """更新速度并做限幅"""
        if axis == "linear_x":
            self.current_vel.linear.x = max(-VEL_LIMIT["linear_x"],
                                            min(VEL_LIMIT["linear_x"],
                                                self.current_vel.linear.x + delta))
        elif axis == "linear_y":
            self.current_vel.linear.y = max(-VEL_LIMIT["linear_y"],
                                            min(VEL_LIMIT["linear_y"],
                                                self.current_vel.linear.y + delta))
        elif axis == "angular_z":
            self.current_vel.angular.z = max(-VEL_LIMIT["angular_z"],
                                             min(VEL_LIMIT["angular_z"],
                                                 self.current_vel.angular.z + delta))
        self._print_vel()

    def stop(self):
        self.current_vel = Twist()
        print("  → 速度清零")

    def _print_vel(self):
        v = self.current_vel
        print(f"  → 速度: linear=[{v.linear.x:.2f}, {v.linear.y:.2f}, {v.linear.z:.2f}]  "
              f"angular=[{v.angular.x:.2f}, {v.angular.y:.2f}, {v.angular.z:.2f}]")


def get_key(timeout=0.1):
    """非阻塞读取单个按键（支持方向键转义序列）"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            ch = sys.stdin.read(1)
            # 方向键：ESC [ A/B/C/D
            if ch == '\x1b':
                rlist2, _, _ = select.select([sys.stdin], [], [], 0.05)
                if rlist2:
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        rlist3, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if rlist3:
                            ch3 = sys.stdin.read(1)
                            return 'ARROW_' + {'A': 'UP', 'B': 'DOWN',
                                               'C': 'RIGHT', 'D': 'LEFT'}.get(ch3, '?')
            return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None


def print_help():
    print("""
━━━━━━━━━━━━━━━━ 机器人键盘控制器 ━━━━━━━━━━━━━━━━
  模式切换（状态机流转：idle→zero→stand→walk）:
    i  →  idle      z  →  zero     s  →  stand
    w  →  walk_leg  a  →  walk_leg_arm
    k  →  keep      p  →  plan

  速度控制（需先进入 walk 模式）:
    ↑/W → 前进     ↓/S → 后退
    ←/A → 左移     →/D → 右移
    Q   → 左转     E   → 右转
    空格 → 急停

  q 或 Ctrl+C → 退出
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
""")


def spin_node(node):
    rclpy.spin(node)


def main():
    rclpy.init()
    node = KeyboardController()

    # 在后台线程运行 ROS2 spin
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    print_help()

    try:
        while rclpy.ok():
            key = get_key(timeout=0.1)
            if key is None:
                continue

            # 退出
            if key in ('\x03', 'q'):  # Ctrl+C 或 q
                print("\n[退出] 键盘控制器已关闭")
                break

            # 模式切换
            elif key in KEY_TO_MODE:
                node.trigger_mode(KEY_TO_MODE[key])

            # 速度控制 - 大写字母或方向键
            elif key == 'ARROW_UP'   or key == 'W': node.update_velocity("linear_x",  VEL_STEP)
            elif key == 'ARROW_DOWN' or key == 'S': node.update_velocity("linear_x", -VEL_STEP)
            elif key == 'ARROW_LEFT' or key == 'A': node.update_velocity("linear_y",  VEL_STEP)
            elif key == 'ARROW_RIGHT'or key == 'D': node.update_velocity("linear_y", -VEL_STEP)
            elif key == 'Q':                         node.update_velocity("angular_z",  VEL_STEP)
            elif key == 'E':                         node.update_velocity("angular_z", -VEL_STEP)
            elif key == ' ':                         node.stop()

            # 帮助
            elif key == 'h':
                print_help()

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
