#!/usr/bin/env python3
"""
can_node.py - CAN Bus + Doosan DRFL + SOEM EtherCAT ROS2 Düğümü
=================================================================
Düzeltmeler:
  - CAN bağlantısı olmadan simülasyon load cell verisi YAYINLANMİYOR
  - can_status False iken load_cells topic'i susturuldu
  - Simülasyon modu SADECE simulation:=true parametresiyle açılır
  - DRFL/SOEM katmanları korundu (D4.2 §3.1.1)
"""

import rclpy
from rclpy.node import Node
import threading
import time
import json
import math

from std_msgs.msg import String, Bool, Float64

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

try:
    import DRFL
    DRFL_AVAILABLE = True
except ImportError:
    DRFL_AVAILABLE = False

try:
    import pysoem
    SOEM_AVAILABLE = True
except ImportError:
    SOEM_AVAILABLE = False

# ── Protokol Sabitleri ────────────────────────────────────────────────────────
PACKET_HEADER    = 0xAA
PACKET_LENGTH    = 10
LOAD_CELL_OFFSET = 80
LOAD_CELL_MIN    = -10.0
LISTEN_INTERVAL  = 0.01
SANDER_ON        = 111
SANDER_OFF       = 222

DOOSAN_IP               = '192.168.137.100'
DOOSAN_PORT             = 12345
DOOSAN_FLANGE_DO_SANDER = 1
ETHERCAT_ADAPTER        = 'eth0'
ETHERCAT_TIMEOUT        = 50_000  # µs

INIT_PACKET = bytearray([
    0xAA, 0x55, 0x12, 0x07,
    0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x1A,
])


# ── Yardımcı fonksiyonlar ─────────────────────────────────────────────────────
def _parse_load_cells(raw: bytes):
    for i in range(len(raw) - PACKET_LENGTH, -1, -1):
        if raw[i] == PACKET_HEADER and i + PACKET_LENGTH <= len(raw):
            return [
                max(LOAD_CELL_MIN, raw[i + 4] - LOAD_CELL_OFFSET),
                max(LOAD_CELL_MIN, raw[i + 5] - LOAD_CELL_OFFSET),
                max(LOAD_CELL_MIN, raw[i + 6] - LOAD_CELL_OFFSET),
                max(LOAD_CELL_MIN, raw[i + 7] - LOAD_CELL_OFFSET),
            ]
    return None


def _build_frame(s1: int, s2: int, sander: int) -> bytearray:
    return bytearray([
        0xAA, 0xC5, 0x03, 0x03, 0x00,
        s1 & 0xFF, s2 & 0xFF, sander & 0xFF,
        0x00, 0x55
    ])


# ── Doosan DRFL Katmanı ───────────────────────────────────────────────────────
class DrflLayer:
    """
    Doosan H2515 DRFL API sarmalayıcısı.
    D4.2 §3.1.1: Real-Time DRFL API — TCP/UDP üzerinden robot controller box
    """

    def __init__(self, ip, port, sim, logger):
        self.ip        = ip
        self.port      = port
        self.sim       = sim or not DRFL_AVAILABLE
        self.log       = logger
        self._robot    = None
        self.connected = False

    def connect(self) -> bool:
        if self.sim:
            self.connected = True
            self.log.info('[DRFL] Simülasyon modu aktif')
            return True
        try:
            self._robot = DRFL.RobotSystem()
            self._robot.connect(self.ip, self.port)
            self._robot.set_robot_mode(DRFL.ROBOT_MODE_AUTONOMOUS)
            self.connected = True
            self.log.info(f'[DRFL] Doosan bağlandı: {self.ip}:{self.port}')
            return True
        except Exception as e:
            self.log.error(f'[DRFL] Bağlantı hatası: {e}')
            return False

    def disconnect(self):
        if self._robot and not self.sim:
            try: self._robot.disconnect()
            except Exception: pass
        self.connected = False

    def move_joint(self, angles, speed=30.0, accel=60.0):
        if not self.connected: return False
        if self.sim:
            self.log.debug(f'[DRFL-SIM] moveJ: {angles}'); return True
        try:
            self._robot.moveJ(angles, speed, accel); return True
        except Exception as e:
            self.log.error(f'[DRFL] moveJ: {e}'); return False

    def set_digital_output(self, port, val):
        """Doosan flange 6+6 I/O — zımpara relay (D4.2 §1.5.1)"""
        if not self.connected: return False
        if self.sim:
            self.log.debug(f'[DRFL-SIM] DO[{port}]={val}'); return True
        try:
            self._robot.set_digital_output(port, 1 if val else 0); return True
        except Exception as e:
            self.log.error(f'[DRFL] DO: {e}'); return False

    def get_tcp_force(self):
        """Doosan dahili 6-eksen kuvvet sensörü (D4.2 §1.5, 0.2 N hassasiyet)"""
        if not self.connected: return [0.0] * 6
        if self.sim:
            import random
            base = 5.0 + 3.0 * math.sin(time.time() * 0.5)
            return [round(base + random.gauss(0, 0.2), 2) for _ in range(6)]
        try: return list(self._robot.get_tcp_force())
        except Exception: return [0.0] * 6

    def halt(self):
        """Mevcut hareketi iptal et — E-stop'tan önce güvenli durdurma adımı."""
        if self.sim:
            self.log.warn('[DRFL-SIM] HALT'); return
        if self._robot:
            try: self._robot.halt()
            except Exception as e: self.log.error(f'[DRFL] halt: {e}')

    def get_current_posj(self):
        """Anlık eklem açılarını döndür (6 eksen, rad) — D4.2 §3.1.1"""
        if not self.connected: return [0.0] * 6
        if self.sim: return [0.0] * 6
        try: return list(self._robot.get_current_posj())
        except Exception as e:
            self.log.error(f'[DRFL] get_current_posj: {e}'); return [0.0] * 6

    def movel(self, pose, speed=50.0, accel=100.0):
        """Kartezyen doğrusal hareket — D4.2 §3.1.1 moveL"""
        if not self.connected: return False
        if self.sim:
            self.log.debug(f'[DRFL-SIM] moveL: {pose}'); return True
        try:
            self._robot.moveL(pose, speed, accel); return True
        except Exception as e:
            self.log.error(f'[DRFL] moveL: {e}'); return False

    def get_external_torque(self):
        """Dış eklem torklarını döndür (6 eksen, Nm) — yük hücresi cross-check."""
        if not self.connected: return [0.0] * 6
        if self.sim: return [0.0] * 6
        try: return list(self._robot.get_external_torque())
        except Exception as e:
            self.log.error(f'[DRFL] get_external_torque: {e}'); return [0.0] * 6

    def emergency_stop(self):
        if not self.sim and self._robot:
            try: self._robot.emergency_stop()
            except Exception as e: self.log.error(f'[DRFL] e_stop: {e}')
        else:
            self.log.warn('[DRFL-SIM] ACİL DURDURMA')


# ── SOEM EtherCAT Katmanı ─────────────────────────────────────────────────────
class SoemLayer:
    """
    SOEM sarmalayıcısı — D4.2 §3.1.1: end-effector fieldbus
    """

    def __init__(self, adapter, sim, logger):
        self.adapter   = adapter
        self.sim       = sim or not SOEM_AVAILABLE
        self.log       = logger
        self._master   = None
        self.connected = False

    def connect(self):
        if self.sim:
            self.connected = True
            self.log.info(f'[SOEM] Simülasyon — adapter: {self.adapter}')
            return True
        try:
            self._master = pysoem.Master()
            self._master.open(self.adapter)
            if self._master.config_init() > 0:
                self._master.config_map()
                self._master.config_dc()
                self._master.state = pysoem.SAFE_OP_STATE
                self._master.write_state()
                self._master.state = pysoem.OP_STATE
                self._master.write_state()
                self.connected = True
                self.log.info(
                    f'[SOEM] EtherCAT hazır: {self.adapter}, '
                    f'{len(self._master.slaves)} slave'
                )
                return True
            self.log.warn('[SOEM] Slave bulunamadı')
            return False
        except Exception as e:
            self.log.error(f'[SOEM] Bağlantı: {e}')
            return False

    def disconnect(self):
        if self._master and not self.sim:
            try:
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()
            except Exception: pass
        self.connected = False

    def send_servo(self, s1, s2, sander):
        if not self.connected: return False
        if self.sim:
            self.log.debug(f'[SOEM-SIM] S1:{s1} S2:{s2} sander:{sander}')
            return True
        try:
            # Gerçek implementasyon: PDO yaz
            # self._master.slaves[0].output = _build_frame(s1, s2, sander)
            # self._master.send_processdata()
            # self._master.receive_processdata(ETHERCAT_TIMEOUT)
            return True
        except Exception as e:
            self.log.error(f'[SOEM] send_servo: {e}'); return False

    def read_load_cells(self):
        if not self.connected or self.sim: return None
        try:
            # Gerçek implementasyon: PDO oku
            # self._master.send_processdata()
            # self._master.receive_processdata(ETHERCAT_TIMEOUT)
            # return _parse_load_cells(self._master.slaves[0].input)
            return None
        except Exception: return None


# ── Ana Düğüm ─────────────────────────────────────────────────────────────────
class CANNode(Node):

    def __init__(self):
        super().__init__('can_node')

        # Parametreler
        self.declare_parameter('port',             '/dev/ttyUSB0')
        self.declare_parameter('baudrate',         2000000)
        self.declare_parameter('simulation',       False)   # SADECE True ise sim
        self.declare_parameter('publish_rate',     10.0)
        self.declare_parameter('use_drfl',         True)
        self.declare_parameter('use_soem',         False)
        self.declare_parameter('doosan_ip',        DOOSAN_IP)
        self.declare_parameter('doosan_port',      DOOSAN_PORT)
        self.declare_parameter('ethercat_adapter', ETHERCAT_ADAPTER)

        self.port         = self.get_parameter('port').value
        self.baudrate     = self.get_parameter('baudrate').value
        self.simulation   = self.get_parameter('simulation').value
        self.publish_rate = self.get_parameter('publish_rate').value
        use_drfl          = self.get_parameter('use_drfl').value
        use_soem          = self.get_parameter('use_soem').value
        doosan_ip         = self.get_parameter('doosan_ip').value
        doosan_port       = self.get_parameter('doosan_port').value
        ethercat_adapter  = self.get_parameter('ethercat_adapter').value

        # Durum
        self._ser          = None
        self._lock         = threading.Lock()
        self._running      = True
        self._sim_running  = False   # _sim_load_cell_loop aktif mi?
        self.load_cells    = [0.0, 0.0, 0.0, 0.0]
        self.last_s1       = 160
        self.last_s2       = 160
        self.last_sander   = SANDER_OFF
        self._can_active   = False   # Gerçek CAN bağlantısı var mı?
        self._last_heartbeat = time.time()  # watchdog için son komut zamanı

        # DRFL ve SOEM katmanları
        self.drfl = DrflLayer(
            ip=doosan_ip, port=doosan_port,
            sim=self.simulation, logger=self.get_logger()
        ) if use_drfl else None

        self.soem = SoemLayer(
            adapter=ethercat_adapter,
            sim=self.simulation, logger=self.get_logger()
        ) if use_soem else None

        # Publisher'lar
        self.pub_lc      = self.create_publisher(String,  '/end_effector/load_cells',      10)
        self.pub_status  = self.create_publisher(Bool,    '/end_effector/can_status',       10)
        self.pub_servo   = self.create_publisher(String,  '/end_effector/servo_state',      10)
        self.pub_gz_s1   = self.create_publisher(Float64, '/end_effector/gazebo/joint_s1',  10)
        self.pub_gz_s2   = self.create_publisher(Float64, '/end_effector/gazebo/joint_s2',  10)
        self.pub_drfl_st = self.create_publisher(String,  '/end_effector/drfl_status',      10)

        # Subscriber'lar
        self.create_subscription(String, '/end_effector/servo_command',
                                 self._cb_servo_cmd,   10)
        self.create_subscription(String, '/end_effector/sander_only',
                                 self._cb_sander_only, 10)
        self.create_subscription(Bool,   '/end_effector/emergency_stop',
                                 self._cb_emergency,   10)
        self.create_subscription(Bool,   '/end_effector/shutdown',
                                 self._cb_shutdown,    10)
        self.create_subscription(String, '/end_effector/set_mode',
                                 self._cb_set_mode,    10)

        # Timer'lar
        self.create_timer(1.0 / self.publish_rate, self._publish_state)
        self.create_timer(0.5, self._publish_drfl_status)
        self.create_timer(5.0, self._watchdog_check)

        # Başlat
        self._startup()

    # ── Başlangıç ─────────────────────────────────────────────────────────────
    def _startup(self):
        if self.simulation:
            # Açıkça simulation:=true verilmişse sim modunda başla
            self.get_logger().info('🟡 SİMÜLASYON MODU (parametre ile etkinleştirildi)')
            self._can_active = False  # Sim modunda CAN bağlı DEĞİL
            self.pub_status.publish(Bool(data=False))
            self._sim_running = True
            threading.Thread(target=self._sim_load_cell_loop,
                             daemon=True, name='SimLC').start()
        else:
            # Gerçek mod: CAN bağlantısı dene, başarısız olursa veri YOK
            self.get_logger().info('CAN bağlantısı deneniyor...')
            self.pub_status.publish(Bool(data=False))   # Başlangıçta False

            if SERIAL_AVAILABLE:
                threading.Thread(target=self._connect_serial,
                                 daemon=True, name='CAN-Connect').start()
            else:
                self.get_logger().warn(
                    'pyserial kurulu değil. CAN verisi alınamaz. '
                    'Simülasyon için simulation:=true kullanın.'
                )
                # Simülasyon moduna GEÇME — sadece uyar

        # DRFL ve SOEM her iki modda da başla
        if self.drfl:
            threading.Thread(target=self.drfl.connect,
                             daemon=True, name='DRFL-Connect').start()
        if self.soem:
            threading.Thread(target=self.soem.connect,
                             daemon=True, name='SOEM-Connect').start()

    # ── Simülasyon load cell (sadece simulation:=true ile) ────────────────────
    def _sim_load_cell_loop(self):
        """Simülasyon modu: gerçekçi load cell verisi üretir — _sim_running=False ile durur"""
        import random
        t = 0.0
        while self._running and self._sim_running:
            base = 5.0 + 3.0 * math.sin(t * 0.5)
            self.load_cells = [
                round(base + random.gauss(0, 0.25), 2),
                round(base + random.gauss(0, 0.25), 2),
                round(base + random.gauss(0, 0.25), 2),
                round(base + random.gauss(0, 0.25), 2),
            ]
            t += LISTEN_INTERVAL
            time.sleep(LISTEN_INTERVAL)
        self.load_cells = [0.0, 0.0, 0.0, 0.0]  # mod değişince sıfırla

    # ── Gerçek CAN bağlantısı ─────────────────────────────────────────────────
    def _connect_serial(self):
        try:
            port = self._find_port() or self.port
            self._ser = serial.Serial(port, self.baudrate, timeout=0.1)
            self._ser.setDTR(True)
            self._ser.setRTS(True)
            self._ser.write(INIT_PACKET)
            self._ser.flush()
            time.sleep(0.5)

            self._can_active = True
            self.pub_status.publish(Bool(data=True))
            self.get_logger().info(f'✅ CAN bağlandı: {port} @ {self.baudrate}')
            threading.Thread(target=self._listen_loop,
                             daemon=True, name='CAN-Listen').start()

        except Exception as e:
            self._can_active = False
            self.pub_status.publish(Bool(data=False))
            self.get_logger().error(
                f'❌ CAN bağlanamadı: {e}\n'
                f'   Load cell verisi devre dışı. '
                f'   Simülasyon için: simulation:=true'
            )
            # Yeniden bağlanma döngüsü (30 saniyede bir)
            threading.Thread(target=self._retry_loop,
                             daemon=True, name='CAN-Retry').start()

    def _retry_loop(self):
        """CAN bağlantısı başarısız olursa 30 saniyede bir tekrar dene (sim modunda durur)."""
        while self._running and not self._can_active and not self.simulation:
            time.sleep(30.0)
            if not self._can_active and self._running and not self.simulation:
                self.get_logger().info('CAN yeniden bağlantı deneniyor...')
                self._connect_serial()

    def _find_port(self):
        if not SERIAL_AVAILABLE: return None
        for p in serial.tools.list_ports.comports():
            desc = p.description.upper()
            if any(x in desc for x in ('USB', 'CH340', 'SERIAL', 'CAN')):
                return p.device
        return None

    def _listen_loop(self):
        """CAN verisi dinleme döngüsü"""
        consecutive_errors = 0
        while self._running:
            try:
                with self._lock:
                    waiting = (self._ser.in_waiting
                               if self._ser and self._ser.is_open else 0)
                if waiting >= PACKET_LENGTH:
                    with self._lock:
                        raw = self._ser.read(waiting)
                    vals = _parse_load_cells(raw)
                    if vals:
                        self.load_cells = vals
                        consecutive_errors = 0
            except Exception as e:
                consecutive_errors += 1
                self.get_logger().error(f'CAN dinleme hatası: {e}')
                if consecutive_errors >= 5:
                    self.get_logger().error('CAN bağlantısı koptu.')
                    self._can_active = False
                    self.pub_status.publish(Bool(data=False))
                    # Yeniden bağlanma dene
                    threading.Thread(target=self._retry_loop,
                                     daemon=True, name='CAN-Retry').start()
                    break
            time.sleep(LISTEN_INTERVAL)

    # ── Komut Callback'leri ───────────────────────────────────────────────────
    def _cb_servo_cmd(self, msg: String):
        try:
            data   = json.loads(msg.data)
            s1     = int(data.get('s1',     self.last_s1))
            s2     = int(data.get('s2',     self.last_s2))
            sander = int(data.get('sander', self.last_sander))
            self._send_frame(s1, s2, sander)
        except Exception as e:
            self.get_logger().error(f'servo_command parse: {e}')

    def _cb_sander_only(self, msg: String):
        try:
            data   = json.loads(msg.data)
            sander = int(data.get('sander', SANDER_OFF))
            self._send_frame(self.last_s1, self.last_s2, sander)
        except Exception as e:
            self.get_logger().error(f'sander_only parse: {e}')

    def _cb_emergency(self, msg: Bool):
        if msg.data:
            self.get_logger().error('!!! ACİL DURDURMA !!!')
            if self.drfl:
                self.drfl.halt()            # önce hareketi durdur
                self.drfl.emergency_stop()  # sonra donanım E-stop
            self._send_frame(160, 160, SANDER_OFF)

    # ── Watchdog ──────────────────────────────────────────────────────────────
    def _watchdog_check(self):
        """
        EN ISO 13849-1 yazılım izleme: DRFL bağlıyken 60s komut gelmezse
        güvenli durdurma uygula.
        """
        if (self.drfl and self.drfl.connected and not self.drfl.sim
                and time.time() - self._last_heartbeat > 60.0):
            self.get_logger().error('[WATCHDOG] 60s komut yok — güvenli durdurma!')
            self.drfl.halt()

    # ── Çok katmanlı komut gönderimi ──────────────────────────────────────────
    def _send_frame(self, s1: int, s2: int, sander: int):
        """
        D4.2 §3.1.1 mimarisine göre:
          1. Seri CAN   — mevcut end-effector protokolü
          2. SOEM       — EtherCAT fieldbus
          3. DRFL       — Doosan flange I/O + eklem hareketi
          4. Gazebo     — simülasyon topic'leri
        """
        self._last_heartbeat = time.time()
        self.last_s1     = s1
        self.last_s2     = s2
        self.last_sander = sander
        st = 'SANDING' if sander == SANDER_ON else 'IDLE'

        # 1. Seri CAN
        if self._can_active and not self.simulation and self._ser and self._ser.is_open:
            try:
                frame = _build_frame(s1, s2, sander)
                with self._lock:
                    self._ser.write(frame)
                    self._ser.flush()
                self.get_logger().debug(f'CAN TX → S1:{s1}° S2:{s2}° {st}')
            except Exception as e:
                self.get_logger().error(f'CAN TX: {e}')

        # 2. SOEM EtherCAT
        if self.soem:
            self.soem.send_servo(s1, s2, sander)

        # 3. DRFL — flange I/O (zımpara) + joint
        if self.drfl and self.drfl.connected:
            self.drfl.set_digital_output(DOOSAN_FLANGE_DO_SANDER, sander == SANDER_ON)
            if not self.simulation:
                j5 = float(s1 - 90)
                j6 = float(s2 - 90)
                self.drfl.move_joint([0.0, 0.0, 0.0, 0.0, j5, j6])

        # 4. Gazebo joint topic'leri (her zaman yayınla)
        self.pub_gz_s1.publish(Float64(data=math.radians(s1 - 90)))
        self.pub_gz_s2.publish(Float64(data=math.radians(s2 - 90)))

        if self.simulation:
            self.get_logger().info(f'[SIM] S1:{s1}° S2:{s2}° {st}')

    # ── Durum Yayını ──────────────────────────────────────────────────────────
    def _publish_state(self):
        # SOEM'den load cell dene
        if self.soem:
            vals = self.soem.read_load_cells()
            if vals: self.load_cells = vals

        # DRFL kuvvet sensörü (gerçek donanımda)
        if (self.drfl and self.drfl.connected
                and not self.drfl.sim and not self.simulation):
            forces = self.drfl.get_tcp_force()
            if forces and len(forces) >= 4:
                self.load_cells = [round(abs(f), 2) for f in forces[:4]]

        # Load cell sadece bağlıyken yayınla
        if self._can_active:
            self.pub_lc.publish(String(data=json.dumps({'values': self.load_cells})))

        # CAN durumu periyodik yayın — GUI'nin geçiş sonrası senkron kalması için
        self.pub_status.publish(Bool(data=self._can_active))

        # Servo durumu her zaman
        self.pub_servo.publish(String(data=json.dumps({
            's1': self.last_s1, 's2': self.last_s2, 'sander': self.last_sander,
        })))

    def _publish_drfl_status(self):
        drfl_ok = self.drfl.connected if self.drfl else False
        soem_ok = self.soem.connected if self.soem else False
        self.pub_drfl_st.publish(String(data=json.dumps({
            'drfl_connected': drfl_ok,
            'drfl_sim':       self.drfl.sim if self.drfl else True,
            'soem_connected': soem_ok,
            'soem_sim':       self.soem.sim if self.soem else True,
            'can_active':     self._can_active,
            'can_sim':        self.simulation,
        })))

    def _cb_set_mode(self, msg: String):
        new_sim = (msg.data == 'simulation')
        if new_sim == self.simulation:
            return
        self.simulation = new_sim
        if self.drfl:
            self.drfl.sim = new_sim or not DRFL_AVAILABLE

        if new_sim:
            self.get_logger().info('[MOD] Simülasyon — CAN devre dışı')
            self._can_active = False
            self.pub_status.publish(Bool(data=False))
            self._sim_running = True
            threading.Thread(target=self._sim_load_cell_loop,
                             daemon=True, name='SimLC-Mode').start()
        else:
            self.get_logger().info('[MOD] Gerçek Donanım — simülasyon verisi durduruluyor')
            self._sim_running = False   # _sim_load_cell_loop'u durdur
            self._can_active  = False
            self.pub_status.publish(Bool(data=False))
            if SERIAL_AVAILABLE and not self._can_active:
                threading.Thread(target=self._connect_serial,
                                 daemon=True, name='CAN-ModeSwitch').start()
            if self.drfl and not self.drfl.connected:
                threading.Thread(target=self.drfl.connect,
                                 daemon=True, name='DRFL-ModeSwitch').start()

    def _cb_shutdown(self, msg):
        if msg.data:
            self.get_logger().info('Shutdown sinyali alındı — kapatılıyor')
            self._running = False
            import os, signal
            os.kill(os.getpid(), signal.SIGINT)

    def destroy_node(self):
        self._running = False
        if self._ser and self._ser.is_open:
            self._ser.close()
        if self.drfl: self.drfl.disconnect()
        if self.soem: self.soem.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CANNode()
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