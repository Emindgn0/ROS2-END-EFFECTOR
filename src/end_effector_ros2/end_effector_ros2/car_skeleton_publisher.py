#!/usr/bin/env python3
"""
car_skeleton_publisher.py — Gazebo Araba Modeli RViz Görselleştirici
=====================================================================
Gazebo'da spawn edilen araba modelinin STL mesh'ini ve zımpara hedeflerini
RViz'de MarkerArray olarak gösterir.
RViz'e eklemek için: Add → By topic → /car_skeleton/markers → MarkerArray
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Pose
import math

# ── Frame ve mesh ─────────────────────────────────────────────────────────────
FRAME_ID = 'base_link'
# Gazebo'da araba spawn pozisyonu: x=-0.3, y=-1.0, z=0.2, yaw=pi/2
# (dsr_gazebo.launch.py'den alındı)
CAR_X   = -0.3
CAR_Y   = -1.0
CAR_Z   =  0.2
CAR_YAW =  math.pi / 2      # 90° yaw
CAR_SCALE = 0.008            # SDF'deki scale ile aynı (mm→m)

MESH_URI = 'package://end_effector_ros2/models/arac_sase/meshes/arac_sase.stl'

# Zımpara hedef noktaları (movel_test waypoints, robot base frame)
GRIND_TARGETS = [
    (-0.362, -0.060, 1.000),   # WP1 hover
    (-0.362, -0.060, 0.940),   # WP2 temas
]

def _rgba(r, g, b, a=1.0):
    c = ColorRGBA(); c.r = r; c.g = g; c.b = b; c.a = a; return c

def _yaw_to_quat(yaw):
    """Yaw açısını quaternion'a çevir (roll=pitch=0)."""
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw / 2)
    q.w = math.cos(yaw / 2)
    return q


class CarSkeletonPublisher(Node):

    def __init__(self):
        super().__init__('car_skeleton')
        self.pub = self.create_publisher(MarkerArray, '/car_skeleton/markers', 10)
        self.create_timer(0.2, self._publish)
        self.get_logger().info(
            f'Araba mesh yayınlanıyor → /car_skeleton/markers\n'
            f'  Mesh : {MESH_URI}\n'
            f'  Pozisyon: x={CAR_X} y={CAR_Y} z={CAR_Z} yaw={math.degrees(CAR_YAW):.0f}°\n'
            f'  RViz Fixed Frame: {FRAME_ID}'
        )


    def _publish(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ── Araba STL mesh ───────────────────────────────────────────────────
        mesh = Marker()
        mesh.header.frame_id = FRAME_ID
        mesh.header.stamp = now
        mesh.ns = 'car_mesh'
        mesh.id = 0
        mesh.type = Marker.MESH_RESOURCE
        mesh.action = Marker.ADD
        mesh.pose.position.x = CAR_X
        mesh.pose.position.y = CAR_Y
        mesh.pose.position.z = CAR_Z
        mesh.pose.orientation = _yaw_to_quat(CAR_YAW)
        mesh.scale.x = CAR_SCALE
        mesh.scale.y = CAR_SCALE
        mesh.scale.z = CAR_SCALE
        mesh.color = _rgba(0.75, 0.75, 0.80, 0.80)
        mesh.mesh_resource = MESH_URI
        mesh.mesh_use_embedded_materials = False
        mesh.lifetime.sec = 0
        ma.markers.append(mesh)

        # ── Zımpara hedef noktaları ──────────────────────────────────────────
        for i, (tx, ty, tz) in enumerate(GRIND_TARGETS):
            sphere = Marker()
            sphere.header.frame_id = FRAME_ID
            sphere.header.stamp = now
            sphere.ns = 'grind_targets'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = tx
            sphere.pose.position.y = ty
            sphere.pose.position.z = tz
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
            sphere.color = _rgba(1.0, 0.1, 0.1, 1.0)
            sphere.lifetime.sec = 0
            ma.markers.append(sphere)

            label = Marker()
            label.header.frame_id = FRAME_ID
            label.header.stamp = now
            label.ns = 'grind_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = tx + 0.05
            label.pose.position.y = ty
            label.pose.position.z = tz + 0.05
            label.pose.orientation.w = 1.0
            label.scale.z = 0.04
            label.color = _rgba(1.0, 1.0, 1.0, 1.0)
            label.text = f'WP{i+1} ({tz:.2f}m)'
            label.lifetime.sec = 0
            ma.markers.append(label)

        self.pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CarSkeletonPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
