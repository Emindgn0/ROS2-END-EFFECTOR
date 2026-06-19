#!/usr/bin/env python3
"""
logic_node.py - Otonom Görev ve Yönlendirme Düğümü
====================================================
Subscribe:
  /end_effector/detections         (std_msgs/String - JSON)
  /end_effector/mission_start      (std_msgs/Bool)
  /end_effector/mission_stop       (std_msgs/Bool)
  /end_effector/emergency_stop     (std_msgs/Bool)
  /end_effector/load_cells         (std_msgs/String - JSON)

Publish:
  /end_effector/servo_command      (std_msgs/String - JSON)
  /end_effector/sander_only        (std_msgs/String - JSON)
  /end_effector/mission_status     (std_msgs/String - JSON)
  /end_effector/guidance           (std_msgs/String - JSON)
  /end_effector/log                (std_msgs/String)
  /cartesian_interface/arm/reference  (geometry_msgs/PoseStamped)
"""

import rclpy
from rclpy.node import Node
import threading
import time
import json
import math
import os

from std_msgs.msg import String, Bool

try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    from geometry_msgs.msg import PoseStamped
    CARTESIO_AVAILABLE = True
except ImportError:
    CARTESIO_AVAILABLE = False

# ── Kalibrasyon & Sabitler ────────────────────────────────────────────────────
FRAME_CX         = 640
FRAME_CY         = 360
TARGET_TOL_PX    = 20          # piksel — XY hizalama toleransı
SERVO_PAN_COEFF  = 12
SERVO_TILT_COEFF = 12
SERVO_CENTER     = 160
SERVO_MIN        = 0
SERVO_MAX        = 180

SCAN_DURATION    = 5.0         # saniye — tarama süresi
NAVIGATE_WAIT    = 2.5         # saniye — CartesI/O hareket bekleme

SANDER_ON        = 111
SANDER_OFF       = 222

SERVO_OPEN_POS   = 0.025   # m — kamera dışarıda (tarama modu)
SERVO_CLOSED_POS = 0.0     # m — kamera içeride (hareket / zımparalama)

# ── Force Control Sabitleri ───────────────────────────────────────────────────
FORCE_CONTACT_THRESHOLD  = 25.0   # N  — temas kuvveti hedefi
FORCE_SAFETY_LIMIT       = 50.0   # N  — güvenlik limiti (acil dur)

# CartesI/O Z parametreleri
Z_APPROACH_STEP     = 0.002   # m  — her adımda 2mm in
Z_APPROACH_INTERVAL = 0.1     # s  — adımlar arası bekleme
Z_HOVER_HEIGHT      = 1.0     # m  — hover yüksekliği (B pillar çalışma)
Z_MIN_HEIGHT        = 0.88    # m  — temas arama alt sınırı
Z_RETRACT_HEIGHT    = 1.10    # m  — görev sonrası güvenli geri çekilme

# Robot base frame'inde varsayılan XY çalışma pozisyonu (B-pillar)
WORK_POS_X  = -0.40   # m  — arabaya doğru (-X yönü)
WORK_POS_Y  =  0.00   # m  — araç Y merkezine yakın

# Kamera → robot koordinat dönüşüm (lineer fallback)
CAM_TO_ROBOT_X = -0.01   # m/piksel
CAM_TO_ROBOT_Y = -0.01   # m/piksel

# Empedans kontrolü
IMPEDANCE_GAIN     = 0.00005  # m/N
IMPEDANCE_DEADBAND = 2.0      # N

# DSR_ROBOT2 hız/ivme
DSR_VEL_LIN  = 50    # mm/s
DSR_ACC_LIN  = 100   # mm/s²
DSR_VEL_DEG  = 30    # deg/s
DSR_ACC_DEG  = 60    # deg/s²

# ── Oryantasyon sabitleri — quaternion (qx, qy, qz, qw) ──────────────────────
_SQ2_INV = 1.0 / math.sqrt(2.0)
# Tool Z aşağı (yatay yüzey için)
ORIENT_DOWN     = (0.0,  1.0,      0.0, 0.0)
# Tool Z → −X yönü: B-pillar'a dik yaklaşım (robot +X'ten bakar)
# R_y(−90°) → q = (0, −1/√2, 0, 1/√2)
ORIENT_B_PILLAR = (0.0, -_SQ2_INV, 0.0, _SQ2_INV)


class CameraCalibration:
    """
    Kamera piksel koordinatlarını robot ΔX/ΔY'ye (metre) dönüştürür.

    Desteklenen yöntemler:
      1. homography — 3×3 H matrisi (önerilen)
      2. linear     — sabit px→m katsayıları (fallback)

    Kalibrasyon dosyası: ~/.ros/camera_robot_calibration.json
    """

    CAL_PATH = os.path.expanduser('~/.ros/camera_robot_calibration.json')

    def __init__(self, frame_cx: int, frame_cy: int, logger):
        self._cx     = frame_cx
        self._cy     = frame_cy
        self._log    = logger
        self._H      = None
        self._sx     = CAM_TO_ROBOT_X
        self._sy     = CAM_TO_ROBOT_Y
        self._method = 'linear'
        self._load()

    def _load(self):
        if not os.path.exists(self.CAL_PATH):
            self._log.warn(
                f'Kalibrasyon dosyası yok: {self.CAL_PATH}\n'
                '  Lineer yaklaşım kullanılıyor — gerçek donanımda konum hatası oluşabilir!\n'
                '  El-göz kalibrasyonu: ros2 run end_effector_ros2 calibrate_camera'
            )
            return
        try:
            with open(self.CAL_PATH) as f:
                cal = json.load(f)
            method = cal.get('method', 'linear')
            if method == 'homography' and NUMPY_AVAILABLE and 'H' in cal:
                self._H      = np.array(cal['H'], dtype=float)
                self._method = 'homography'
                self._log.info(f'Homografi kalibrasyonu yüklendi: {self.CAL_PATH}')
            else:
                self._sx     = float(cal.get('sx', CAM_TO_ROBOT_X))
                self._sy     = float(cal.get('sy', CAM_TO_ROBOT_Y))
                self._method = 'linear'
                self._log.info(f'Lineer kalibrasyon: sx={self._sx:.4f} sy={self._sy:.4f}')
        except Exception as e:
            self._log.error(f'Kalibrasyon yüklenemedi: {e} — varsayılan kullanılıyor')

    def pixel_to_robot_delta(self, dx_px: float, dy_px: float):
        """Piksel ofsetini robot ΔX, ΔY (metre) olarak döndür."""
        if self._method == 'homography' and self._H is not None:
            pt  = np.array([dx_px, dy_px, 1.0], dtype=float)
            res = self._H @ pt
            res /= res[2]
            return float(res[0]), float(res[1])
        return dx_px * self._sx, dy_px * self._sy


class LogicNode(Node):

    def __init__(self):
        super().__init__('logic_node')

        # Parametreler
        self.declare_parameter('simulation',     False)
        self.declare_parameter('use_real_robot', False)
        self._sim_mode       = self.get_parameter('simulation').value
        self._use_real_robot = self.get_parameter('use_real_robot').value

        # Görev durumu
        self.is_scanning      = False
        self.mission_active   = False
        self.emergency        = False
        self.scan_start       = 0.0
        self._max_burr_count  = -1
        self._best_burrs      = []
        self._latest_burrs    = []
        self._lock            = threading.Lock()
        self.current_guidance = {'dir': 'STANDBY', 'dx': 0.0, 'dy': 0.0}

        # Kamera kalibrasyonu
        self._calibration = CameraCalibration(FRAME_CX, FRAME_CY, self.get_logger())

        # Force control durumu
        self._load_cells       = [0.0, 0.0, 0.0, 0.0]
        self._load_cells_stamp = 0.0
        self._contact_force    = 0.0
        self._contact_made     = False
        self._current_z        = Z_HOVER_HEIGHT
        self._current_x        = WORK_POS_X
        self._current_y        = WORK_POS_Y

        # Publisher'lar
        self.pub_servo   = self.create_publisher(String, '/end_effector/servo_command',  10)
        self.pub_sander  = self.create_publisher(String, '/end_effector/sander_only',    10)
        self.pub_status  = self.create_publisher(String, '/end_effector/mission_status', 10)
        self.pub_guide   = self.create_publisher(String, '/end_effector/guidance',       10)
        self.pub_log     = self.create_publisher(String, '/end_effector/log',            10)
        self.pub_home    = self.create_publisher(Bool,   '/end_effector/go_home',        10)

        if CARTESIO_AVAILABLE:
            self.pub_cartesio = self.create_publisher(
                PoseStamped, '/cartesian_interface/arm/reference', 10
            )
        else:
            self.pub_cartesio = None
            self.get_logger().warn('geometry_msgs yok — CartesI/O devre dışı')

        # Subscriber'lar
        self.create_subscription(String, '/end_effector/detections',
                                 self._cb_detections,    10)
        self.create_subscription(Bool,   '/end_effector/mission_start',
                                 self._cb_mission_start, 10)
        self.create_subscription(Bool,   '/end_effector/mission_stop',
                                 self._cb_mission_stop,  10)
        self.create_subscription(Bool,   '/end_effector/emergency_stop',
                                 self._cb_emergency,     10)
        self.create_subscription(Bool,   '/end_effector/shutdown',
                                 self._cb_shutdown,      10)
        self.create_subscription(String, '/end_effector/load_cells',
                                 self._cb_load_cells,    10)
        self.create_subscription(String, '/end_effector/set_mode',
                                 self._cb_set_mode,      10)

        self.create_timer(0.1, self._publish_status)

        # DSR_ROBOT2 hazırlık — DR_init.__dsr__node import'tan ÖNCE atanmalı
        # (DSR_ROBOT2.py import anında bu node ile servis client'ları oluşturur,
        # bkz. movel_test.py run_real())
        self._move_line_cli   = None
        self._set_mode_cli    = None
        self._robot_mode_set  = False
        if self._use_real_robot:
            try:
                from dsr_msgs2.srv import MoveLine, SetRobotMode
                self._move_line_cli = self.create_client(MoveLine,     '/dsr01/motion/move_line')
                self._set_mode_cli  = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')
                # set_robot_mode spin başladıktan sonra timer'dan çağrılır;
                # __init__ içinde spin_until_future_complete kullanmak
                # sonraki rclpy.spin() callback'lerini bozmaktadır.
                self._dsr_mode_timer = self.create_timer(0.5, self._dsr_set_robot_mode_once)
                self.get_logger().info('DSR_ROBOT2 hazır — gerçek robot modu')
            except Exception as e:
                self.get_logger().error(
                    f'use_real_robot=True ama DSR_ROBOT2 yüklenemedi: {type(e).__name__}: {e} '
                    'Gazebo moduna düşülüyor.'
                )
                self._use_real_robot = False

        if self._sim_mode:
            self.get_logger().warn('SİMÜLASYON MODU: sahte çapak + temas aktif')

        mode_str = 'GERÇEK ROBOT' if self._use_real_robot else 'GAZEBO/SİMÜLASYON'
        self.get_logger().info(f'LogicNode başlatıldı — mod: {mode_str}')

    # ── DSR robot modu başlatma (spin sonrası, timer callback'ten) ───────────
    def _dsr_set_robot_mode_once(self):
        if self._robot_mode_set or not self._use_real_robot:
            self.destroy_timer(self._dsr_mode_timer)
            return
        from dsr_msgs2.srv import SetRobotMode
        req = SetRobotMode.Request()
        req.robot_mode = 1  # ROBOT_MODE_AUTONOMOUS
        future = self._set_mode_cli.call_async(req)
        future.add_done_callback(self._dsr_mode_cb)
        # timer otomatik ateşlemeyi durdur (bir kez yeter)
        self.destroy_timer(self._dsr_mode_timer)

    def _dsr_mode_cb(self, future):
        try:
            future.result()
            self._robot_mode_set = True
            self.get_logger().info('[DSR] Robot AUTONOMOUS moda alındı')
        except Exception as e:
            self.get_logger().error(f'[DSR] set_robot_mode hatası: {e}')

    # ── Yardımcı ──────────────────────────────────────────────────────────────
    def _log(self, msg: str, level: str = 'INFO'):
        if level == 'ERROR':   self.get_logger().error(msg)
        elif level == 'WARN':  self.get_logger().warn(msg)
        else:                  self.get_logger().info(msg)
        self.pub_log.publish(String(data=f'[{level}] {msg}'))

    def _send_servo(self, s1: int, s2: int, sander: int):
        self.pub_servo.publish(String(data=json.dumps(
            {'s1': s1, 's2': s2, 'sander': sander}
        )))

    def _send_sander(self, state: int):
        self.pub_sander.publish(String(data=json.dumps({'sander': state})))

    def _go_home(self):
        self._log('HOME pozisyonuna dönülüyor...')
        self._send_sander(SANDER_OFF)
        self._set_camera_servo(SERVO_CLOSED_POS)
        time.sleep(0.3)
        self.pub_home.publish(Bool(data=True))

    def _set_camera_servo(self, pos: float):
        self.pub_servo.publish(String(data=json.dumps({
            'camera': pos,
            'sander': SANDER_OFF,
        })))

    # ── CartesI/O / Gerçek Robot Hareket Arayüzü ─────────────────────────────
    def _send_cartesio_target(self, x: float, y: float, z: float,
                               qx: float = 0.0,
                               qy: float = 0.0,
                               qz: float = 0.0,
                               qw: float = 0.0):
        """
        Gazebo IK'ya hedef gönder.
        Quaternion sıfır → 3DOF pozisyon IK (Gazebo modu için güvenli).
        Gerçek oryantasyon kontrolü DSR_ROBOT2 movel'in rx/ry/rz parametrelerinden gelir.
        """
        if self.pub_cartesio is None:
            return
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._current_x = x
        self._current_y = y
        self._current_z = z
        self.pub_cartesio.publish(pose)

    def _movel_real(self, x_m: float, y_m: float, z_m: float,
                    rx_deg: float = 0.0, ry_deg: float = 180.0, rz_deg: float = 0.0):
        """Gerçek robota move_line servisi — rclpy.spin çakışmasını önlemek için
        call_async + future.done() polling kullanır (spin_until_future_complete değil).
        Main executor future'ı tamamlar; Mission thread sadece poll eder."""
        from dsr_msgs2.srv import MoveLine
        req = MoveLine.Request()
        req.pos       = [x_m * 1000, y_m * 1000, z_m * 1000, rx_deg, ry_deg, rz_deg]
        req.vel       = [float(DSR_VEL_LIN), float(DSR_VEL_DEG)]
        req.acc       = [float(DSR_ACC_LIN), float(DSR_ACC_DEG)]
        req.time      = 0.0
        req.radius    = 0.0
        req.ref       = 0   # DR_BASE
        req.mode      = 0   # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type  = 0  # SYNC — controller bloklar, future motion bitince çözülür
        future = self._move_line_cli.call_async(req)
        deadline = time.time() + 60.0
        while not future.done() and not self.emergency and time.time() < deadline:
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError('movel timeout (60 s)')

    def _move_to(self, x: float, y: float, z: float):
        """
        Tek hareket arayüzü.
        use_real_robot=True  → DSR_ROBOT2 movel (bloklanarak bekler)
        use_real_robot=False → CartesI/O (Gazebo IK)
        """
        if self._use_real_robot:
            self._movel_real(x, y, z)
        else:
            self._send_cartesio_target(x, y, z)
            time.sleep(NAVIGATE_WAIT)

    # ── Kuvvet ────────────────────────────────────────────────────────────────
    def _get_contact_force(self) -> float:
        if time.time() - self._load_cells_stamp > 2.0:
            if self._sim_mode and getattr(self, '_contact_made', False):
                import math as _math
                return round(20.0 + 8.0 * _math.sin(time.time() * 0.7), 1)
            return 0.0
        with self._lock:
            vals = [v for v in self._load_cells if v > 0]
        return sum(vals)

    # ── Callback'ler ──────────────────────────────────────────────────────────
    def _cb_load_cells(self, msg: String):
        try:
            data = json.loads(msg.data)
            vals = data.get('values', [0.0]*4)
            with self._lock:
                self._load_cells = vals
            self._load_cells_stamp = time.time()
            self._contact_force = self._get_contact_force()
        except Exception as e:
            self.get_logger().error(f'Load cell parse: {e}')

    def _cb_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
            burrs = data.get('burrs', [])
            with self._lock:
                self._latest_burrs = burrs
            if self.is_scanning:
                with self._lock:
                    if len(burrs) >= self._max_burr_count:
                        self._max_burr_count = len(burrs)
                        self._best_burrs = list(burrs)
        except Exception as e:
            self.get_logger().error(f'Tespit parse: {e}')

    def _cb_mission_start(self, msg: Bool):
        if not msg.data: return
        if self.is_scanning or self.mission_active:
            self._log('Mission zaten aktif!', 'WARN'); return
        self.emergency = False
        self._log('Otonom görev başlatılıyor...')
        threading.Thread(
            target=self._mission_orchestrator,
            daemon=True, name='Mission'
        ).start()

    def _cb_mission_stop(self, msg: Bool):
        if msg.data: self._stop_mission('CANCELLED')

    def _cb_emergency(self, msg: Bool):
        if msg.data:
            self.emergency = True
            self._stop_mission('EMERGENCY')
            self._send_sander(SANDER_OFF)
            self._set_camera_servo(SERVO_CLOSED_POS)
            self._log('!!! ACİL DURDURMA !!!', 'ERROR')

    def _cb_set_mode(self, msg: String):
        new_sim = (msg.data == 'simulation')
        if new_sim == self._sim_mode:
            return
        self._sim_mode = new_sim
        label = 'SİMÜLASYON' if new_sim else 'GERÇEK DONANIM'
        self._log(f'Çalışma modu: {label}')
        if self.mission_active or self.is_scanning:
            self._stop_mission('MODE_CHANGED')

    def _cb_shutdown(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Shutdown sinyali — kapatılıyor')
            self._stop_mission('SHUTDOWN')
            import os, signal
            os.kill(os.getpid(), signal.SIGINT)

    def _stop_mission(self, reason: str = 'STOPPED'):
        self.mission_active = False
        self.is_scanning    = False
        self._contact_made  = False
        self.current_guidance = {'dir': reason, 'dx': 0.0, 'dy': 0.0}
        self._log(f'Görev durduruldu: {reason}', 'WARN')

    # ── Ana Görev Orkestrasyonu ───────────────────────────────────────────────
    def _mission_orchestrator(self):
        """
        A. SERVO AÇ  — kamera dışarı çıkar
        B. TARAMA    — 5s YOLO çapak tespiti
        C. SERVO KAP — kamera içeri girer
        D. [hedef başına] HOVER → Z İNİŞ → TEMAS → ZIMPARALA → GERİ ÇEK
        """

        # A. Kamera servoyu aç
        self._set_camera_servo(SERVO_OPEN_POS)
        self._log('Kamera açıldı — taranıyor...')
        time.sleep(0.5)

        # B. Tarama
        with self._lock:
            self._max_burr_count = -1
            self._best_burrs     = []

        self.is_scanning = True
        for remaining in range(int(SCAN_DURATION), 0, -1):
            if not self.is_scanning or self.emergency:
                self._set_camera_servo(SERVO_CLOSED_POS)
                return
            self.current_guidance = {
                'dir': f'TARANIYIOR... {remaining}s', 'dx': 0.0, 'dy': 0.0
            }
            time.sleep(1.0)
        self.is_scanning = False

        # C. Kamera servoyu kapat
        self._set_camera_servo(SERVO_CLOSED_POS)
        self._log('Kamera kapandı — hedefler işleniyor')
        time.sleep(0.3)

        with self._lock:
            captured = list(self._best_burrs)

        if not captured:
            if self._sim_mode:
                captured = [{'x': FRAME_CX, 'y': FRAME_CY, 'conf': 0.9, 'dist': 0.0}]
                self._log('SİMÜLASYON: Ekran merkezine sahte çapak eklendi', 'WARN')
            else:
                self._log('Hedef bulunamadı! Görev iptal.', 'ERROR')
                self.current_guidance = {'dir': 'HEDEF YOK', 'dx': 0.0, 'dy': 0.0}
                return

        self._log(f'Tarama tamamlandı — {len(captured)} çapak bulundu')
        targets = sorted(captured, key=lambda b: b.get('dist', 9999))
        total   = len(targets)
        self.mission_active = True

        for i, burr in enumerate(targets):
            if not self.mission_active or self.emergency:
                self._log('Görev durduruldu.', 'WARN'); break

            label = f'H{i+1}/{total}'
            self._log(f'{label}: çapak px=({burr["x"]},{burr["y"]}) conf={burr["conf"]:.2f}')

            # D1. HOVER — XY hizala, hover yüksekliğine git
            self._phase_hover(label, burr)

            # D2. Z İNİŞ — temas kuvvetine kadar
            ok = self._phase_z_descend(label)
            if not ok:
                self._retract(label)
                continue

            # D3. ZIMPARALA
            self._phase_grind(label, burr)

            # D4. GERİ ÇEK
            self._retract(label)

        self.mission_active = False
        self.current_guidance = {'dir': 'GÖREV TAMAMLANDI', 'dx': 0.0, 'dy': 0.0}
        self._log('=== OTONOM GÖREV TAMAMLANDI ===')
        self._go_home()

    # ── D1. Hover — XY hizala ─────────────────────────────────────────────────
    def _phase_hover(self, label: str, burr: dict):
        """
        Çapak piksel koordinatından robot XY hesaplar,
        hover yüksekliğine konumlanır.
        """
        dx_px = burr['x'] - FRAME_CX
        dy_px = burr['y'] - FRAME_CY
        dx_m, dy_m = self._calibration.pixel_to_robot_delta(dx_px, dy_px)

        target_x = WORK_POS_X + dx_m
        target_y = WORK_POS_Y + dy_m

        self._log(
            f'{label}: HOVER → x={target_x:.3f}m y={target_y:.3f}m '
            f'z={Z_HOVER_HEIGHT:.3f}m  '
            f'(Δx={dx_m*100:.1f}cm Δy={dy_m*100:.1f}cm)'
        )
        self.current_guidance = {
            'dir': f'{label} | HOVER',
            'dx': round(dx_m * 100, 1),
            'dy': round(dy_m * 100, 1),
        }

        self._move_to(target_x, target_y, Z_HOVER_HEIGHT)
        self._log(f'{label}: Hover tamamlandı')

    # ── D2. Z İniş — Force Controlled ────────────────────────────────────────
    def _phase_z_descend(self, label: str) -> bool:
        self._log(f'{label}: Z inişi başlıyor (hedef: {FORCE_CONTACT_THRESHOLD}N)...')
        self._contact_made = False
        z = self._current_z

        while z > Z_MIN_HEIGHT:
            if not self.mission_active or self.emergency:
                return False

            force = self._get_contact_force()

            if force >= FORCE_SAFETY_LIMIT:
                self._log(f'{label}: GÜVENLİK LİMİTİ! F={force:.1f}N — geri çekiliyor', 'ERROR')
                return False

            if force >= FORCE_CONTACT_THRESHOLD:
                self._contact_made = True
                self._log(f'{label}: TEMAS! F={force:.1f}N @ Z={z:.3f}m')
                self.current_guidance = {
                    'dir': f'{label} | TEMAS ✓ {force:.1f}N', 'dx': 0.0, 'dy': 0.0
                }
                return True

            z -= Z_APPROACH_STEP
            z  = max(z, Z_MIN_HEIGHT)
            self.current_guidance = {
                'dir': f'{label} | Z İNİŞ Z={z:.3f}m F={force:.1f}N',
                'dx': 0.0, 'dy': 0.0
            }

            if not self._use_real_robot:
                self._send_cartesio_target(self._current_x, self._current_y, z)
            else:
                self._movel_real(self._current_x, self._current_y, z)

            # Simülasyon: Z_MIN yakınında temas simüle et
            if self._sim_mode and z <= Z_MIN_HEIGHT + 0.02:
                time.sleep(0.3)
                force = self._get_contact_force()
                if force < FORCE_CONTACT_THRESHOLD:
                    force = FORCE_CONTACT_THRESHOLD
                self._contact_made = True
                self._log(f'{label}: SİM TEMAS! F={force:.1f}N @ Z={z:.3f}m')
                self.current_guidance = {
                    'dir': f'{label} | TEMAS ✓ {force:.1f}N', 'dx': 0.0, 'dy': 0.0
                }
                return True

            time.sleep(Z_APPROACH_INTERVAL)

        self._log(f'{label}: Min yüksekliğe ulaşıldı, temas yok!', 'WARN')
        return False

    # ── D3. Zımparalama ───────────────────────────────────────────────────────
    def _phase_grind(self, label: str, burr: dict):
        conf       = burr.get('conf', 0.5)
        grind_time = 2.0 + conf * 6.0

        self._log(f'{label}: ZIMPARALAMA başlıyor ({grind_time:.1f}s, conf={conf:.2f})')
        self.current_guidance = {
            'dir': f'{label} | ZİMPARALAMA {grind_time:.0f}s', 'dx': 0.0, 'dy': 0.0
        }
        self._send_sander(SANDER_ON)

        t_start = time.time()
        while (time.time() - t_start) < grind_time:
            if not self.mission_active or self.emergency:
                self._send_sander(SANDER_OFF)
                return

            force   = self._get_contact_force()
            elapsed = time.time() - t_start

            if force >= FORCE_SAFETY_LIMIT:
                self._log(f'{label}: Zımparalama sırasında güvenlik limiti!', 'ERROR')
                break

            if force < 5.0 and elapsed > 0.5:
                self._log(f'{label}: Düşük kuvvet F={force:.1f}N — temas kaybı?', 'WARN')

            # Empedans kontrolü (sadece Gazebo)
            if not self._use_real_robot and self.pub_cartesio is not None:
                force_err = FORCE_CONTACT_THRESHOLD - force
                if abs(force_err) > IMPEDANCE_DEADBAND:
                    z_adj = max(-0.001, min(0.001, force_err * IMPEDANCE_GAIN))
                    new_z = max(Z_MIN_HEIGHT, min(Z_HOVER_HEIGHT, self._current_z + z_adj))
                    if abs(new_z - self._current_z) > 5e-5:
                        self._send_cartesio_target(self._current_x, self._current_y, new_z)

            remaining = grind_time - elapsed
            self.current_guidance = {
                'dir': f'{label} | ZİMPARALAMA {remaining:.1f}s F={force:.1f}N',
                'dx': 0.0, 'dy': 0.0
            }
            time.sleep(0.1)

        self._send_sander(SANDER_OFF)
        self._log(f'{label}: Zımparalama tamamlandı')

    # ── D4. Geri Çekilme ──────────────────────────────────────────────────────
    def _retract(self, label: str):
        self._log(f'{label}: Geri çekiliyor (Z={Z_RETRACT_HEIGHT}m)')
        self.current_guidance = {
            'dir': f'{label} | GERİ ÇEKİLME', 'dx': 0.0, 'dy': 0.0
        }
        self._move_to(self._current_x, self._current_y, Z_RETRACT_HEIGHT)
        self._contact_made = False
        time.sleep(0.5)
        self._log(f'{label}: Geri çekilme tamamlandı')

    # ── Durum Yayını ──────────────────────────────────────────────────────────
    def _publish_status(self):
        with self._lock:
            burr_count = len(self._latest_burrs)
        force = self._get_contact_force()
        self.pub_status.publish(String(data=json.dumps({
            'is_scanning':    self.is_scanning,
            'mission_active': self.mission_active,
            'emergency':      self.emergency,
            'burr_count':     burr_count,
            'guidance':       self.current_guidance,
            'contact_force':  round(force, 2),
            'contact_made':   self._contact_made,
            'current_z':      round(self._current_z, 4),
            'use_real_robot': self._use_real_robot,
        })))
        self.pub_guide.publish(String(data=json.dumps(self.current_guidance)))


def main(args=None):
    rclpy.init(args=args)
    node = LogicNode()
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
