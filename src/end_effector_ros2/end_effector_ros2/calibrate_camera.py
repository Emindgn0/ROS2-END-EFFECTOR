#!/usr/bin/env python3
"""
calibrate_camera.py — El-Göz (Eye-in-Hand) Kalibrasyon Node'u
==============================================================
Kullanım:
  ros2 run end_effector_ros2 calibrate_camera

Ne yapar:
  1. Robot hover yüksekliğinde N pozisyona hareket eder
  2. Her pozisyonda /end_effector/detections verisinden piksel koordinatı okur
  3. Robot XY + piksel XY çiftlerinden lineer ölçek (sx, sy) hesaplar
  4. Sonucu ~/.ros/camera_robot_calibration.json olarak kaydeder

Ön koşullar:
  - Gazebo veya gerçek robot çalışır durumda olmalı
  - vision_node çalışıyor olmalı (YOLO tespiti açık)
  - Kalibrasyon hedefi (bilinen işaretçi veya çapak) sahneye yerleştirilmeli

Kalibrasyon dosyası formatı:
  {"method": "linear", "sx": <float>, "sy": <float>}
  veya
  {"method": "homography", "H": [[3x3 float listesi]]}
"""

import rclpy
from rclpy.node import Node
import json
import math
import os
import time
import threading

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

CAL_PATH     = os.path.expanduser('~/.ros/camera_robot_calibration.json')
FRAME_CX     = 640
FRAME_CY     = 360
HOVER_Z      = 1.0       # m — kalibrasyon yüksekliği
WAIT_SETTLE  = 1.5       # s  — robot yerleşmesi için bekleme
WAIT_DETECT  = 2.0       # s  — YOLO tespiti bekleme

# Kalibrasyon grid noktaları — WORK_POS_X/Y etrafında ±DELTA
BASE_X  = -0.40
BASE_Y  =  0.00
DELTAS  = [
    ( 0.00,  0.00),   # merkez
    ( 0.05,  0.00),   # +5cm X
    (-0.05,  0.00),   # −5cm X
    ( 0.00,  0.05),   # +5cm Y
    ( 0.00, -0.05),   # −5cm Y
    ( 0.05,  0.05),   # köşe
    (-0.05, -0.05),   # köşe
]


class CalibrateCamera(Node):

    def __init__(self):
        super().__init__('calibrate_camera')

        self._last_burrs  = []
        self._lock        = threading.Lock()
        self._data_points = []   # [(robot_dx_m, robot_dy_m, px_dx, px_dy), ...]

        self.pub_cartesio = self.create_publisher(
            PoseStamped, '/cartesian_interface/arm/reference', 10
        )
        self.create_subscription(
            String, '/end_effector/detections', self._cb_detections, 10
        )

        self.get_logger().info(
            'calibrate_camera başlatıldı\n'
            f'  Kalibrasyon grid: {len(DELTAS)} nokta\n'
            f'  Hover yüksekliği: {HOVER_Z}m\n'
            f'  Çıktı: {CAL_PATH}'
        )

        # Kalibrasyon döngüsünü ayrı thread'de başlat
        threading.Thread(target=self._calibrate_loop, daemon=True).start()

    def _cb_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._last_burrs = data.get('burrs', [])
        except Exception:
            pass

    def _move_to(self, x: float, y: float, z: float):
        """CartesI/O'ya hedef gönder — B-pillar oryantasyonu ile."""
        _SQ2_INV = 1.0 / math.sqrt(2.0)
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = -_SQ2_INV
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = _SQ2_INV
        self.pub_cartesio.publish(pose)

    def _get_best_detection(self):
        """En yüksek confidence'lı tespiti döndür veya None."""
        with self._lock:
            burrs = list(self._last_burrs)
        if not burrs:
            return None
        return max(burrs, key=lambda b: b.get('conf', 0))

    def _calibrate_loop(self):
        self.get_logger().info('Kalibrasyona başlanıyor...')
        time.sleep(2.0)  # node'ların hazır olması için bekle

        # Merkeze git ve referans piksel konumunu al
        self._move_to(BASE_X, BASE_Y, HOVER_Z)
        time.sleep(WAIT_SETTLE + WAIT_DETECT)

        ref = self._get_best_detection()
        if ref is None:
            self.get_logger().error(
                'HATA: Merkez pozisyonda tespit yok!\n'
                '  Kalibrasyon hedefinin kamera alanında olduğundan emin olun.'
            )
            return

        ref_px = ref['x']
        ref_py = ref['y']
        self.get_logger().info(f'Referans piksel: ({ref_px}, {ref_py})')

        # Her grid noktasına git
        for i, (dx_m, dy_m) in enumerate(DELTAS[1:], start=1):
            target_x = BASE_X + dx_m
            target_y = BASE_Y + dy_m

            self.get_logger().info(f'Nokta {i}/{len(DELTAS)-1}: '
                                   f'Δ=({dx_m*100:.0f}cm, {dy_m*100:.0f}cm)')
            self._move_to(target_x, target_y, HOVER_Z)
            time.sleep(WAIT_SETTLE + WAIT_DETECT)

            det = self._get_best_detection()
            if det is None:
                self.get_logger().warn(f'Nokta {i}: Tespit yok — atlanıyor')
                continue

            px_dx = det['x'] - ref_px   # piksel değişimi
            px_dy = det['y'] - ref_py

            if abs(px_dx) < 2 and abs(px_dy) < 2:
                self.get_logger().warn(f'Nokta {i}: Piksel değişimi çok küçük — atlanıyor')
                continue

            self._data_points.append((dx_m, dy_m, px_dx, px_dy))
            self.get_logger().info(
                f'  Robot Δ=({dx_m*100:.1f}cm,{dy_m*100:.1f}cm) '
                f'Piksel Δ=({px_dx:.1f},{px_dy:.1f})'
            )

        # Merkeze geri dön
        self._move_to(BASE_X, BASE_Y, HOVER_Z)
        time.sleep(WAIT_SETTLE)

        if len(self._data_points) < 2:
            self.get_logger().error(
                f'Yetersiz veri: {len(self._data_points)} nokta (en az 2 gerekli).'
            )
            return

        self._compute_and_save()

    def _compute_and_save(self):
        """Lineer ölçek katsayılarını en küçük kareler ile hesapla ve kaydet."""
        if not NUMPY_AVAILABLE:
            self._compute_simple()
            return

        data = self._data_points
        # Robot ΔX = sx * Piksel ΔX  →  sx = mean(robot_dx / px_dx)
        sx_vals = [d[0] / d[2] for d in data if abs(d[2]) > 1]
        sy_vals = [d[1] / d[3] for d in data if abs(d[3]) > 1]

        if not sx_vals or not sy_vals:
            self.get_logger().error('Katsayı hesaplamak için yeterli veri yok.')
            return

        sx = float(np.mean(sx_vals))
        sy = float(np.mean(sy_vals))

        self.get_logger().info(
            f'\nKalibrasyon sonucu:\n'
            f'  sx = {sx:.6f} m/px  ({sx*100:.4f} cm/px)\n'
            f'  sy = {sy:.6f} m/px  ({sy*100:.4f} cm/px)\n'
            f'  Nokta sayısı: {len(data)}'
        )

        # Homografi desteği: ≥4 nokta varsa
        if NUMPY_AVAILABLE and len(data) >= 4:
            self._save_homography(data, sx, sy)
        else:
            self._save_linear(sx, sy)

    def _compute_simple(self):
        """numpy olmadan basit ortalama."""
        data = self._data_points
        sx_vals = [d[0] / d[2] for d in data if abs(d[2]) > 1]
        sy_vals = [d[1] / d[3] for d in data if abs(d[3]) > 1]
        sx = sum(sx_vals) / len(sx_vals) if sx_vals else -0.01
        sy = sum(sy_vals) / len(sy_vals) if sy_vals else -0.01
        self._save_linear(sx, sy)

    def _save_linear(self, sx: float, sy: float):
        cal = {'method': 'linear', 'sx': sx, 'sy': sy}
        os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)
        with open(CAL_PATH, 'w') as f:
            json.dump(cal, f, indent=2)
        self.get_logger().info(f'Lineer kalibrasyon kaydedildi: {CAL_PATH}')

    def _save_homography(self, data, sx_fallback: float, sy_fallback: float):
        """
        Piksel → robot dönüşümü için 3×3 homografi matrisi hesapla.
        src: piksel delta noktaları, dst: robot delta noktaları (metre).
        """
        src = np.array([[d[2], d[3]] for d in data], dtype=np.float32)
        dst = np.array([[d[0], d[1]] for d in data], dtype=np.float32)

        H, mask = cv2_find_homography(src, dst)
        if H is None:
            self.get_logger().warn('Homografi hesaplanamadı — lineer kullanılıyor')
            self._save_linear(sx_fallback, sy_fallback)
            return

        cal = {'method': 'homography', 'H': H.tolist(),
               'sx_fallback': sx_fallback, 'sy_fallback': sy_fallback}
        os.makedirs(os.path.dirname(CAL_PATH), exist_ok=True)
        with open(CAL_PATH, 'w') as f:
            json.dump(cal, f, indent=2)
        self.get_logger().info(f'Homografi kalibrasyonu kaydedildi: {CAL_PATH}')


def cv2_find_homography(src, dst):
    """cv2 varsa homografi hesapla, yoksa None döndür."""
    try:
        import cv2
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 3.0)
        return H, mask
    except Exception:
        return None, None


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateCamera()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
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
