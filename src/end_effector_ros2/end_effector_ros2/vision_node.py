#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import numpy as np
import json
import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

CONF_THRESHOLD  = 0.40
CAMERA_RETRY    = 3.0   # saniye — yeniden deneme aralığı
IOU_NMS_THR     = 0.45  # tile birleştirme NMS eşiği


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.package_share_dir = get_package_share_directory('end_effector_ros2')

        # Parametreler
        self.declare_parameter('model_name',     'YOLO26s.pt')
        self.declare_parameter('camera_index',   0)
        self.declare_parameter('stream_fps',     30)
        self.declare_parameter('use_gazebo_cam', False)
        self.declare_parameter('use_tiling',     False)
        self.declare_parameter('tile_size',      640)
        self.declare_parameter('tile_overlap',   0.2)

        self.model_name     = self.get_parameter('model_name').value
        self.camera_index   = self.get_parameter('camera_index').value
        self.stream_fps     = self.get_parameter('stream_fps').value
        self.use_gazebo_cam = self.get_parameter('use_gazebo_cam').value
        self.use_tiling     = self.get_parameter('use_tiling').value
        self.tile_size      = self.get_parameter('tile_size').value
        self.tile_overlap   = self.get_parameter('tile_overlap').value

        self.bridge        = CvBridge()
        self.model         = None
        self.cap           = None
        self.camera_active = True
        self._last_retry   = 0.0
        self._running      = True

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher'lar
        self.pub_raw        = self.create_publisher(Image,  '/end_effector/camera/image_raw',       img_qos)
        self.pub_annotated  = self.create_publisher(Image,  '/end_effector/camera/image_annotated', img_qos)
        self.pub_detections = self.create_publisher(String, '/end_effector/detections',             10)
        self.pub_cam_status = self.create_publisher(Bool,   '/end_effector/camera_status',          10)

        # Subscriber'lar
        self.create_subscription(Bool,   '/end_effector/camera_enable', self._cb_cam_enable, 10)
        self.create_subscription(String, '/end_effector/set_model',     self._cb_set_model,  10)
        self.create_subscription(Bool,   '/end_effector/shutdown',      self._cb_shutdown,   10)

        if self.use_gazebo_cam:
            self.create_subscription(Image, '/camera/image_raw', self._cb_gazebo_image, img_qos)
            self.get_logger().info('Gazebo kamera modu aktif.')
        else:
            period = 1.0 / self.stream_fps
            self.create_timer(period, self._process_frame)
            self.get_logger().info(
                f'USB kamera — index:{self.camera_index} fps:{self.stream_fps} '
                f'tiling:{self.use_tiling} tile_size:{self.tile_size}'
            )

        self._load_model(self.model_name)

    # ── Model ─────────────────────────────────────────────────────────────────
    def _load_model(self, model_name: str):
        if not YOLO_AVAILABLE:
            self.get_logger().warn('ultralytics kurulu değil — tespit devre dışı')
            return

        full_path = os.path.join(self.package_share_dir, 'weights', model_name)

        if not os.path.exists(full_path):
            self.get_logger().warn(f'Model bulunamadı: {full_path}')
            weights_dir = os.path.join(self.package_share_dir, 'weights')
            if os.path.isdir(weights_dir):
                pts = [f for f in os.listdir(weights_dir) if f.endswith('.pt')]
                if pts:
                    full_path = os.path.join(weights_dir, pts[0])
                    self.get_logger().info(f'Alternatif model: {pts[0]}')
                else:
                    self.get_logger().error('weights/ klasöründe .pt yok!')
                    return
            else:
                self.get_logger().error(f'weights/ yok: {weights_dir}')
                return

        try:
            self.model = YOLO(full_path)
            self.model_name = model_name
            self.get_logger().info(f'Model yüklendi: {full_path}')
        except Exception as e:
            self.get_logger().error(f'Model yükleme hatası: {e}')
            self.model = None

    def _cb_set_model(self, msg: String):
        self.get_logger().info(f'Model değiştiriliyor: {msg.data}')
        self._load_model(msg.data)

    # ── Kamera ────────────────────────────────────────────────────────────────
    def _open_camera(self) -> bool:
        if self.cap and self.cap.isOpened():
            return True
        now = time.time()
        if now - self._last_retry < CAMERA_RETRY:
            return False
        self._last_retry = now
        self.get_logger().info(f'Kamera açılıyor: index={self.camera_index}')
        cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.cap = cap
            self.pub_cam_status.publish(Bool(data=True))
            self.get_logger().info('Kamera açıldı ✓')
            return True
        self.get_logger().warn(f'Kamera açılamadı (index={self.camera_index})')
        self.pub_cam_status.publish(Bool(data=False))
        return False

    def _cb_cam_enable(self, msg: Bool):
        self.camera_active = msg.data
        if not msg.data:
            if self.cap:
                self.cap.release()
                self.cap = None
            self.pub_cam_status.publish(Bool(data=False))
            self.get_logger().info('Kamera kapatıldı')

    def _process_frame(self):
        if not self.camera_active:
            return
        if not self._open_camera():
            return
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Frame okunamadı — kamera yeniden açılacak')
            self.cap.release()
            self.cap = None
            return
        self._run_detection(frame)

    def _cb_gazebo_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._run_detection(frame)
        except Exception as e:
            self.get_logger().error(f'Gazebo görüntü hatası: {e}')

    # ── Tespit ────────────────────────────────────────────────────────────────
    def _run_detection(self, frame: np.ndarray):
        try:
            ros_raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.pub_raw.publish(ros_raw)
        except Exception:
            pass

        h, w, _ = frame.shape
        cx_mid, cy_mid = w // 2, h // 2
        annotated = frame.copy()

        if self.model is not None:
            try:
                if self.use_tiling:
                    boxes = self._detect_tiled(frame)
                else:
                    boxes = self._detect_full(frame)
            except Exception as e:
                self.get_logger().error(f'YOLO hatası: {e}')
                boxes = []
        else:
            boxes = []

        burrs = []
        for x1, y1, x2, y2, conf in boxes:
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            dist   = math.hypot(cx - cx_mid, cy - cy_mid)
            burrs.append({'x': cx, 'y': cy, 'conf': round(conf, 3), 'dist': round(dist, 1)})
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f'Capak {conf:.2f}', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.drawMarker(annotated, (cx_mid, cy_mid), (255, 0, 0), cv2.MARKER_CROSS, 30, 2)

        try:
            ros_ann = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            self.pub_annotated.publish(ros_ann)
        except Exception:
            pass

        self.pub_detections.publish(
            String(data=json.dumps({'burrs': burrs, 'count': len(burrs)}))
        )

    # ── Tam-görüntü YOLO ──────────────────────────────────────────────────────
    def _detect_full(self, frame: np.ndarray):
        """Tek çağrı ile tam görüntü üzerinde YOLO çalıştır."""
        results = self.model(
            np.ascontiguousarray(frame),
            conf=CONF_THRESHOLD,
            verbose=False
        )
        boxes = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf = float(box.conf[0])
                boxes.append((x1, y1, x2, y2, conf))
        return boxes

    # ── Tile Tabanlı YOLO ─────────────────────────────────────────────────────
    def _detect_tiled(self, frame: np.ndarray):
        """
        Görüntüyü örtüşen tile'lara böler, her tile'da YOLO çalıştırır,
        koordinatları tam-görüntü uzayına dönüştürür, NMS ile birleştirir.
        """
        h, w = frame.shape[:2]
        step = int(self.tile_size * (1.0 - self.tile_overlap))
        all_boxes = []

        xs = list(range(0, w, step))
        ys = list(range(0, h, step))

        for y0 in ys:
            for x0 in xs:
                x1 = min(x0 + self.tile_size, w)
                y1 = min(y0 + self.tile_size, h)
                tile = frame[y0:y1, x0:x1]

                results = self.model(
                    np.ascontiguousarray(tile),
                    conf=CONF_THRESHOLD,
                    verbose=False
                )
                for result in results:
                    for box in result.boxes:
                        bx1, by1, bx2, by2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        # tile koordinatlarını tam-görüntü koordinatlarına çevir
                        all_boxes.append((bx1 + x0, by1 + y0, bx2 + x0, by2 + y0, conf))

        return self._nms(all_boxes, IOU_NMS_THR)

    # ── NMS ───────────────────────────────────────────────────────────────────
    def _nms(self, boxes, iou_thr: float):
        """Confidence'a göre sıralı NMS — tile sınırlarındaki çift tespitleri temizler."""
        if not boxes:
            return []
        boxes_s = sorted(boxes, key=lambda b: b[4], reverse=True)
        keep = []
        while boxes_s:
            best = boxes_s.pop(0)
            keep.append(best)
            boxes_s = [b for b in boxes_s if self._iou(best, b) < iou_thr]
        return keep

    @staticmethod
    def _iou(a, b) -> float:
        """İki kutu arasındaki IoU (0-1)."""
        ax1, ay1, ax2, ay2 = a[:4]
        bx1, by1, bx2, by2 = b[:4]
        ix1 = max(ax1, bx1); iy1 = max(ay1, by1)
        ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
        inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
        if inter == 0:
            return 0.0
        area_a = (ax2 - ax1) * (ay2 - ay1)
        area_b = (bx2 - bx1) * (by2 - by1)
        return inter / (area_a + area_b - inter)

    def _cb_shutdown(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown sinyali — kapatılıyor')
            import os, signal
            os.kill(os.getpid(), signal.SIGINT)

    def destroy_node(self):
        self._running = False
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
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
