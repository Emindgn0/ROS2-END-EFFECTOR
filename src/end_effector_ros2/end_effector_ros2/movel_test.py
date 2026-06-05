#!/usr/bin/env python3
"""
movel_test.py — H2515 Hareket Testi (Gazebo + Gerçek Robot)
============================================================
Kullanım:
  ros2 run end_effector_ros2 movel_test             # Gazebo modu (varsayılan)
  ros2 run end_effector_ros2 movel_test --real      # Gerçek robot / DRCF emülatörü

Gazebo modu:
  CartesI/O üzerinden 6DOF IK test eder.
  Gerekli: ros2 launch end_effector_ros2 gazebo.launch.py spawn_car:=true
           ros2 launch end_effector_ros2 end_effector.launch.py use_gazebo:=true

Gerçek robot modu:
  DSR_ROBOT2 ile movej/movel gönderir.
  Gerekli: Docker Desktop WSL2 entegrasyonu +
           ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py model:=h2515
"""

import rclpy
from rclpy.node import Node
import sys
import time
import math

ROBOT_ID    = 'dsr01'
ROBOT_MODEL = 'h2515'

# B-pillar test noktaları — robot base frame, METRE cinsinden
# (x_m, y_m, z_m)
_SQ2_INV = 1.0 / math.sqrt(2.0)
TEST_WAYPOINTS_M = [
    (-0.362, -0.060, 1.00),   # WP1: B-pillar hover yüksekliği
    (-0.362, -0.060, 0.94),   # WP2: 60mm aşağı (yaklaşım)
    (-0.362, -0.060, 1.00),   # WP3: hover'a geri çekil
]

# Aynı noktalar DSR_ROBOT2 için mm + oryantasyon
TEST_WAYPOINTS_MM = [
    (-362, -60, 1000, 0, 180, 0),
    (-362, -60,  940, 0, 180, 0),
    (-362, -60, 1000, 0, 180, 0),
]

HOME_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ── Gazebo Modu ───────────────────────────────────────────────────────────────
class MovelTestGazebo(Node):
    """CartesI/O + gazebo_bridge 6DOF IK üzerinden hareket testi."""

    def __init__(self):
        super().__init__('movel_test')

        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import String
        from sensor_msgs.msg import JointState

        self._PoseStamped = PoseStamped
        self._joint_pos   = None

        self.pub_ik  = self.create_publisher(PoseStamped, '/cartesian_interface/arm/reference', 10)
        self.pub_log = self.create_publisher(String, '/end_effector/log', 10)
        self.create_subscription(JointState, '/gz/joint_states', self._cb_joints, 10)

    def _cb_joints(self, msg):
        from sensor_msgs.msg import JointState
        self._joint_pos = dict(zip(msg.name, msg.position))

    def move_to(self, x: float, y: float, z: float, label: str = ''):
        """Pozisyon hedefi gönder (3DOF IK — Gazebo modu için güvenli)."""
        pose = self._PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        # Sıfır quaternion → gazebo_bridge 3DOF pozisyon IK kullanır
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 0.0

        tag = f'[{label}] ' if label else ''
        self.get_logger().info(f'{tag}→ x={x:.3f} y={y:.3f} z={z:.3f}m')
        self.pub_ik.publish(pose)

        # Robot hareketinin yerleşmesi için bekle
        time.sleep(2.5)

        # Son joint pozisyonlarını logla
        if self._joint_pos:
            j_deg = {k: round(math.degrees(v), 1) for k, v in self._joint_pos.items()
                     if k.startswith('joint_')}
            self.get_logger().info(f'  Joints: {j_deg}')

    def go_home(self):
        """Gazebo'da home sinyali gönder."""
        from std_msgs.msg import Bool
        pub = self.create_publisher(Bool, '/end_effector/go_home', 10)
        time.sleep(0.3)
        pub.publish(Bool(data=True))
        self.get_logger().info('HOME → /end_effector/go_home')
        time.sleep(2.0)

    def run(self):
        self.get_logger().info('=== movel_test [GAZEBO] başlıyor ===')
        self.get_logger().info(
            'B-pillar oryantasyonu: R_y(−90°) → tool Z → −X\n'
            f'Test noktaları: {len(TEST_WAYPOINTS_M)} waypoint'
        )

        # Home'a git
        self.go_home()

        # B-pillar waypoint'leri test et
        for i, (x, y, z) in enumerate(TEST_WAYPOINTS_M):
            if not rclpy.ok():
                break
            self.move_to(x, y, z, label=f'WP{i+1}')
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('=== movel_test [GAZEBO] tamamlandı ===')
        self.go_home()


# ── Gerçek Robot Modu ─────────────────────────────────────────────────────────
def run_real():
    rclpy.init()
    node = rclpy.create_node('movel_test', namespace=ROBOT_ID)

    try:
        import DR_init
        DR_init.__dsr__id    = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node  = node

        from DSR_ROBOT2 import (
            movej, movel, set_velx, set_accx, set_robot_mode,
            posj, posx, ROBOT_MODE_AUTONOMOUS,
        )
    except ImportError as e:
        node.get_logger().error(
            f'DSR_ROBOT2 yüklenemedi: {e}\n'
            '  Önce DRCF emülatörünü başlatın:\n'
            '  1. Docker Desktop → Settings → WSL Integration → Ubuntu etkinleştir\n'
            '  2. ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py model:=h2515'
        )
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    set_velx(50, 30)
    set_accx(100, 60)

    node.get_logger().info('=== movel_test [GERÇEK ROBOT] başlıyor ===')

    home = posj(*HOME_JOINTS_DEG)
    node.get_logger().info(f'HOME: movej {HOME_JOINTS_DEG}')
    ret = movej(home, vel=30, acc=60)
    if ret != 0:
        node.get_logger().error(f'movej hata kodu: {ret}')
    time.sleep(1.0)

    for i, wp in enumerate(TEST_WAYPOINTS_MM):
        x, y, z, rx, ry, rz = wp
        node.get_logger().info(f'WP{i+1}: movel ({x},{y},{z}) ori=({rx},{ry},{rz})')
        pos = posx(x, y, z, rx, ry, rz)
        ret = movel(pos, vel=[50, 30], acc=[100, 60])
        if ret != 0:
            node.get_logger().error(f'movel WP{i+1} hata: {ret}')
        time.sleep(0.5)

    node.get_logger().info("HOME'a dönülüyor...")
    movej(home, vel=30, acc=60)
    time.sleep(1.0)

    node.get_logger().info('=== movel_test [GERÇEK ROBOT] tamamlandı ===')
    node.destroy_node()
    rclpy.shutdown()


# ── Main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    use_real = '--real' in (sys.argv or [])

    if use_real:
        run_real()
    else:
        rclpy.init(args=args)
        node = MovelTestGazebo()
        try:
            node.run()
        except KeyboardInterrupt:
            pass
        finally:
            try:
                node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
