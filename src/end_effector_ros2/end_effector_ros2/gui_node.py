#!/usr/bin/env python3
"""
gui_node.py - PyQt6 Kontrol Arayüzü (ROS2 Düğümü)
Kamera: bytes sinyal ile thread-safe görüntü aktarımı
ROS: MultiThreadedExecutor — hiçbir mesaj kaçmaz
"""

import sys, json, math, time, traceback, threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from .dsr2_interface import Dsr2Layer

try:
    from PyQt6.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QSlider, QTextEdit, QGroupBox, QComboBox,
        QProgressBar, QScrollArea, QSizePolicy, QSpinBox, QRadioButton,
    )
    from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
    from PyQt6.QtGui import QPixmap, QImage, QFont, QKeySequence, QShortcut
    PYQT_AVAILABLE = True
except ImportError:
    PYQT_AVAILABLE = False

# ── Sabitler ──────────────────────────────────────────────────────────────────
SERVO_PRESETS = {'0°':0,'30°':30,'45°':45,'90°':90,'135°':135,'160°':160,'180°':180}
SANDER_ON  = 111
SANDER_OFF = 222


# ── ROS2 → Qt Köprüsü ────────────────────────────────────────────────────────
class ROSBridge(QObject):
    log_received        = pyqtSignal(str)
    status_received     = pyqtSignal(dict)
    load_cell_received  = pyqtSignal(list)
    guidance_received   = pyqtSignal(dict)
    can_status_received = pyqtSignal(bool)
    # Kamera: bytes + boyut — numpy değil, GIL ile thread-safe
    image_ready         = pyqtSignal(bytes, int, int, int)

    def __init__(self, node: Node):
        super().__init__()
        self.node=node; self.can_connected=False; self._got_annotated=False

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)

        node.create_subscription(String,'/end_effector/log',           self._cb_log,    10)
        node.create_subscription(String,'/end_effector/mission_status',self._cb_status, 10)
        node.create_subscription(String,'/end_effector/load_cells',    self._cb_lc,     10)
        node.create_subscription(String,'/end_effector/guidance',      self._cb_guide,  10)
        node.create_subscription(Bool,  '/end_effector/can_status',    self._cb_can,    10)

        from sensor_msgs.msg import Image as RosImage
        from cv_bridge import CvBridge
        self._bridge=CvBridge()
        node.create_subscription(RosImage,'/end_effector/camera/image_annotated',
                                 self._cb_annotated, img_qos)
        node.create_subscription(RosImage,'/end_effector/camera/image_raw',
                                 self._cb_raw, img_qos)
        node.get_logger().info('ROSBridge ready')

    def _cb_annotated(self, msg):
        self._got_annotated=True; self._process(msg)

    def _cb_raw(self, msg):
        if self._got_annotated: return
        self._process(msg)

    def _process(self, msg):
        """Encoding-aware ROS Image → BGR numpy → sinyal"""
        try:
            import cv2, numpy as np
            enc = msg.encoding.lower().replace('-', '')
            if enc == 'rgb8':
                bgr = cv2.cvtColor(self._bridge.imgmsg_to_cv2(msg, 'rgb8'), cv2.COLOR_RGB2BGR)
            elif enc in ('bgr8', 'bgr16'):
                bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            elif enc in ('mono8', 'mono16', '8uc1', '16uc1'):
                gray = self._bridge.imgmsg_to_cv2(msg, 'mono8')
                bgr  = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            elif enc in ('rgba8', 'bgra8'):
                tmp  = self._bridge.imgmsg_to_cv2(msg, 'bgra8')
                bgr  = cv2.cvtColor(tmp, cv2.COLOR_BGRA2BGR)
            else:
                try:
                    bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
                except Exception:
                    raw = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
                    bgr = cv2.cvtColor(raw.astype(np.uint8), cv2.COLOR_GRAY2BGR) \
                          if raw.ndim == 2 else raw[:, :, :3]
            frame = np.ascontiguousarray(bgr, dtype=np.uint8)
            if frame.ndim != 3 or frame.shape[2] != 3: return
            h, w = frame.shape[:2]
            self.image_ready.emit(bytes(frame.tobytes()), w, h, w * 3)
        except Exception as e:
            print(f'[CAM ERR] enc={msg.encoding} {e}')

    def _cb_log(self,msg):    self.log_received.emit(msg.data)
    def _cb_status(self,msg):
        try: self.status_received.emit(json.loads(msg.data))
        except: pass
    def _cb_lc(self,msg):
        if not self.can_connected: return
        try: self.load_cell_received.emit(json.loads(msg.data).get('values',[0]*4))
        except: pass
    def _cb_guide(self,msg):
        try: self.guidance_received.emit(json.loads(msg.data))
        except: pass
    def _cb_can(self,msg:Bool):
        self.can_connected=msg.data; self.can_status_received.emit(msg.data)


# ── Servo Panel ───────────────────────────────────────────────────────────────
class ServoPanel(QGroupBox):
    def __init__(self,label,default_val=160,parent=None):
        super().__init__(f' {label}',parent)
        self._value=default_val; self._build(default_val)

    def _build(self,dv):
        lay=QVBoxLayout(self); lay.setSpacing(5); lay.setContentsMargins(8,14,8,8)
        top=QHBoxLayout()
        self.lbl_val=QLabel(f'{dv}°')
        self.lbl_val.setFont(QFont('Consolas',13,QFont.Weight.Bold))
        self.lbl_val.setStyleSheet('color:#00ccff;'); self.lbl_val.setFixedWidth(52)
        self.spin=QSpinBox(); self.spin.setRange(0,180); self.spin.setValue(dv)
        self.spin.setSuffix('°'); self.spin.setFixedWidth(68)
        self.spin.setStyleSheet('QSpinBox{background:#1a1a3e;color:#e0e0e0;'
                                'border:1px solid #444;border-radius:4px;padding:2px;}')
        self.spin.valueChanged.connect(self._on_spin)
        top.addWidget(self.lbl_val); top.addStretch()
        top.addWidget(QLabel('Manual:')); top.addWidget(self.spin)
        lay.addLayout(top)
        self.slider=QSlider(Qt.Orientation.Horizontal)
        self.slider.setRange(0,180); self.slider.setValue(dv); self.slider.setMinimumHeight(24)
        self.slider.setStyleSheet('''
            QSlider::groove:horizontal{height:8px;background:#2a2a4a;border-radius:4px;}
            QSlider::sub-page:horizontal{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #0055ff,stop:1 #00ccff);border-radius:4px;}
            QSlider::handle:horizontal{background:#00ccff;width:18px;height:18px;
                border-radius:9px;margin:-5px 0;border:2px solid #ffffff33;}''')
        self.slider.valueChanged.connect(self._on_slider); lay.addWidget(self.slider)
        br=QHBoxLayout(); br.setSpacing(3)
        for name,deg in SERVO_PRESETS.items():
            btn=QPushButton(name); btn.setFixedHeight(26); btn.setCheckable(True)
            if deg==dv: btn.setChecked(True)
            btn.clicked.connect(lambda _,d=deg: self.set_value(d))
            btn.setStyleSheet('''QPushButton{background:#1e1e3e;color:#aaa;border:1px solid #444;
                border-radius:4px;font-size:10px;font-family:Consolas;}
                QPushButton:hover{background:#2a2a5a;color:#fff;}
                QPushButton:checked{background:#0044cc;color:#fff;border:1px solid #00aaff;}''')
            br.addWidget(btn); setattr(self,f'_b{deg}',btn)
        lay.addLayout(br)

    def _on_slider(self,v):
        self._value=v; self.lbl_val.setText(f'{v}°')
        self.spin.blockSignals(True); self.spin.setValue(v); self.spin.blockSignals(False)
        self._hi(v)
    def _on_spin(self,v):
        self._value=v; self.lbl_val.setText(f'{v}°')
        self.slider.blockSignals(True); self.slider.setValue(v); self.slider.blockSignals(False)
        self._hi(v)
    def _hi(self,v):
        for d in SERVO_PRESETS.values():
            b=getattr(self,f'_b{d}',None)
            if b: b.setChecked(d==v)
    def value(self): return self._value
    def set_value(self,v): self.slider.setValue(max(0,min(180,v)))


# ── Ana Pencere ───────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):

    def __init__(self,node,bridge,dsr2,is_sim_can=True):
        super().__init__()
        self.node=node; self.bridge=bridge; self.dsr2=dsr2
        self._can_ok=False; self._last_frame=0.0; self._cur_pix=None
        self._is_sim_can = is_sim_can

        self.pub_start    = node.create_publisher(Bool,  '/end_effector/mission_start',  10)
        self.pub_shutdown = node.create_publisher(Bool,  '/end_effector/shutdown',        10)
        self.pub_stop   = node.create_publisher(Bool,  '/end_effector/mission_stop',   10)
        self.pub_emerg  = node.create_publisher(Bool,  '/end_effector/emergency_stop', 10)
        self.pub_cam    = node.create_publisher(Bool,  '/end_effector/camera_enable',  10)
        self.pub_servo  = node.create_publisher(String,'/end_effector/servo_command',  10)
        self.pub_sander = node.create_publisher(String,'/end_effector/sander_only',    10)
        self.pub_model  = node.create_publisher(String,'/end_effector/set_model',      10)
        self.pub_mode   = node.create_publisher(String,'/end_effector/set_mode',       10)

        self._build_ui()
        self._connect_signals()
        # simulation:=false ile başlatıldıysa radio button'u gerçek donanıma ayarla
        if not is_sim_can:
            self.rb_real.setChecked(True)
        threading.Thread(target=self._init_dsr2,daemon=True).start()
        self.setWindowTitle('End Effector Control — ROS2 + DSR_ROBOT2')
        self.setMinimumSize(900,600); self.resize(1440,900)
        QShortcut(QKeySequence('Escape'), self,
                  lambda: self.showNormal() if self.isFullScreen() else self.showMinimized())
        QShortcut(QKeySequence('F11'), self,
                  lambda: self.showNormal() if self.isFullScreen() else self.showFullScreen())

    def _init_dsr2(self):
        self.dsr2.sim = self.rb_sim.isChecked()
        ok=self.dsr2.connect()
        txt=(f'DSR_ROBOT2 {"✓ Connected" if ok else "✗ Disconnected"} '
             f'{"[SIM]" if self.dsr2.sim else "[REAL]"}')
        QTimer.singleShot(0,lambda: self._drfl_ui(ok,txt))

    def _drfl_ui(self,ok,txt):
        if self.rb_sim.isChecked():
            self.lbl_drfl.setText('Simulation mode — Gazebo active')
            self.lbl_drfl.setStyleSheet('color:#00ccff;font-weight:bold;font-size:10px;')
            return
        self._log(f'[INFO] {txt}')
        self.lbl_drfl.setText(txt)
        self.lbl_drfl.setStyleSheet(
            f'color:{"#00ff88" if ok else "#ff4444"};font-weight:bold;font-size:10px;')

    def _build_ui(self):
        cw=QWidget(); self.setCentralWidget(cw)
        root=QHBoxLayout(cw); root.setSpacing(8); root.setContentsMargins(8,8,8,8)

        # ════ SOL ════
        left=QVBoxLayout(); left.setSpacing(6); root.addLayout(left,3)

        # Kamera paneli
        cam_box=QGroupBox(' Camera')
        cl=QVBoxLayout(cam_box); cl.setContentsMargins(4,14,4,4)
        self.lbl_image=QLabel('Waiting for camera...')
        self.lbl_image.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_image.setMinimumSize(400,225)
        self.lbl_image.setSizePolicy(QSizePolicy.Policy.Expanding,QSizePolicy.Policy.Expanding)
        self.lbl_image.setStyleSheet('background:#0a0a1a;color:#555;font-size:11px;border-radius:4px;')
        cl.addWidget(self.lbl_image)
        br=QHBoxLayout()
        self.btn_cam_on =QPushButton('Camera ON')
        self.btn_cam_off=QPushButton('OFF')
        self.lbl_cam_st =QLabel('● Waiting')
        self.lbl_cam_st.setStyleSheet('color:#888;font-family:Consolas;font-size:10px;')
        br.addWidget(self.btn_cam_on); br.addWidget(self.btn_cam_off)
        br.addStretch(); br.addWidget(self.lbl_cam_st)
        cl.addLayout(br); left.addWidget(cam_box,5)

        # Log paneli
        log_box=QGroupBox(' Log')
        ll=QVBoxLayout(log_box); ll.setContentsMargins(4,14,4,4)
        self.log_text=QTextEdit(); self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(
            'background:#060610;color:#88ff88;font-family:Consolas;font-size:10px;border:none;')
        ll.addWidget(self.log_text); left.addWidget(log_box,2)

        # Load Cell paneli
        lc_box=QGroupBox(' Load Cells (N) — Force Control')
        lcl=QVBoxLayout(lc_box); lcl.setContentsMargins(6,14,6,6); lcl.setSpacing(4)
        self.lc_bars=[]
        for i in range(4):
            r=QHBoxLayout()
            lb=QLabel(f'CH{i+1}:'); lb.setFixedWidth(38)
            bar=QProgressBar(); bar.setRange(0,100); bar.setFormat('%v N')
            bar.setFixedHeight(18)
            bar.setStyleSheet('QProgressBar::chunk{background:#0066cc;}')
            vl=QLabel('—'); vl.setFixedWidth(42)
            vl.setStyleSheet('color:#555;font-family:Consolas;')
            r.addWidget(lb); r.addWidget(bar); r.addWidget(vl)
            lcl.addLayout(r); self.lc_bars.append((bar,vl))

        # Toplam kuvvet göstergesi — 25N hedefi
        fg_row=QHBoxLayout()
        fg_row.addWidget(QLabel('Total:'))
        self.bar_force=QProgressBar(); self.bar_force.setRange(0,60)
        self.bar_force.setFormat('%v N'); self.bar_force.setFixedHeight(22)
        self.bar_force.setStyleSheet('''
            QProgressBar{border:1px solid #444;border-radius:4px;background:#111;}
            QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,
                stop:0 #0066cc,stop:0.4 #00aa44,stop:0.7 #ffaa00,stop:1 #ff2200);}''')
        self.lbl_force=QLabel('0.0 N')
        self.lbl_force.setStyleSheet('color:#00ccff;font-family:Consolas;font-weight:bold;')
        self.lbl_force.setFixedWidth(60)
        fg_row.addWidget(self.bar_force); fg_row.addWidget(self.lbl_force)
        lcl.addLayout(fg_row)

        self.lbl_can_st=QLabel('● CAN: Disconnected — data disabled')
        self.lbl_can_st.setStyleSheet('color:#ff6644;font-family:Consolas;font-size:10px;')
        lcl.addWidget(self.lbl_can_st); left.addWidget(lc_box,1)

        # ════ SAĞ ScrollArea ════
        sc=QScrollArea(); sc.setWidgetResizable(True)
        sc.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        sc.setMinimumWidth(320); sc.setMaximumWidth(430)
        sc.setStyleSheet('QScrollArea{border:none;background:transparent;}')
        rw=QWidget(); rw.setStyleSheet('background:transparent;')
        right=QVBoxLayout(rw); right.setSpacing(6); right.setContentsMargins(2,2,6,2)
        sc.setWidget(rw); root.addWidget(sc,0)

        # 1. Görev Kontrol — EN ÜSTTE
        mb=QGroupBox(' Mission Control')
        ml=QVBoxLayout(mb); ml.setContentsMargins(8,14,8,8); ml.setSpacing(5)
        self.btn_start=QPushButton('START AUTONOMOUS')
        self.btn_stop =QPushButton('STOP')
        self.btn_emerg=QPushButton('⚠  EMERGENCY STOP')
        self.btn_start.setStyleSheet(
            'background:#1a6a1a;color:white;font-size:13px;padding:10px;border-radius:6px;')
        self.btn_stop.setStyleSheet(
            'background:#5a5a1a;color:white;font-size:13px;padding:10px;border-radius:6px;')
        self.btn_emerg.setStyleSheet(
            'background:#7a1a1a;color:white;font-size:14px;padding:12px;'
            'border-radius:6px;font-weight:bold;')
        ml.addWidget(self.btn_emerg); ml.addWidget(self.btn_start); ml.addWidget(self.btn_stop)
        right.addWidget(mb)

        # 2. Yönlendirme + Kuvvet durumu
        gbox=QGroupBox(' Guidance & Force')
        gl=QVBoxLayout(gbox); gl.setContentsMargins(4,14,4,6); gl.setSpacing(4)
        self.lbl_guide=QLabel('STANDBY')
        self.lbl_guide.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_guide.setFont(QFont('Consolas',11,QFont.Weight.Bold))
        self.lbl_guide.setMinimumHeight(44)
        self.lbl_guide.setStyleSheet(
            'color:#00ff88;background:#050510;padding:6px;border-radius:6px;')
        gl.addWidget(self.lbl_guide)
        # Kuvvet bilgisi
        self.lbl_force_status=QLabel('Force: — N  |  Contact: NONE')
        self.lbl_force_status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_force_status.setStyleSheet(
            'color:#aaa;font-family:Consolas;font-size:10px;'
            'background:#0a0a1a;border-radius:4px;padding:3px;')
        gl.addWidget(self.lbl_force_status)
        right.addWidget(gbox)

        # 3. Servo paneller
        self.panel_s1=ServoPanel('S1 — Pan (Horizontal)',  default_val=160)
        self.panel_s2=ServoPanel('S2 — Tilt (Vertical)', default_val=160)
        right.addWidget(self.panel_s1); right.addWidget(self.panel_s2)

        # 4. Kamera & Servo Komut
        sb=QGroupBox(' Camera & Servo Command')
        sl=QVBoxLayout(sb); sl.setContentsMargins(8,14,8,8); sl.setSpacing(6)

        # Kamera kutusu AÇ / KAP
        cam_row=QHBoxLayout()
        self.btn_cam_open =QPushButton('Camera Box OPEN')
        self.btn_cam_close=QPushButton('Camera Box CLOSE')
        self.btn_cam_open.setMinimumHeight(32)
        self.btn_cam_close.setMinimumHeight(32)
        self.btn_cam_open.setStyleSheet(
            'background:#1a4a6a;color:white;font-size:11px;border-radius:5px;border:1px solid #2a7aaa;')
        self.btn_cam_close.setStyleSheet(
            'background:#2a2a4a;color:#aaa;font-size:11px;border-radius:5px;border:1px solid #444;')
        cam_row.addWidget(self.btn_cam_open); cam_row.addWidget(self.btn_cam_close)
        sl.addLayout(cam_row)

        # Kamera kutusu durum etiketi
        self.lbl_cam_box=QLabel('Box: CLOSED')
        self.lbl_cam_box.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_cam_box.setStyleSheet(
            'color:#555;font-family:Consolas;font-size:10px;')
        sl.addWidget(self.lbl_cam_box)

        # Pan/Tilt fiziksel servo gönder (CAN)
        self.btn_servo=QPushButton('Send Servo Command')
        self.btn_servo.setStyleSheet(
            'background:#1a3a6a;color:white;font-size:11px;padding:7px;border-radius:6px;')
        sl.addWidget(self.btn_servo)
        right.addWidget(sb)

        # 5. Zımpara Kontrolü
        zb=QGroupBox(' Sander Control')
        zl=QVBoxLayout(zb); zl.setContentsMargins(8,14,8,10); zl.setSpacing(6)
        self.btn_zon =QPushButton('Sander ON')
        self.btn_zoff=QPushButton('Sander OFF')
        self.btn_zon.setFixedHeight(44); self.btn_zoff.setFixedHeight(44)
        self.btn_zon.setFont(QFont('',11,QFont.Weight.Bold))
        self.btn_zoff.setFont(QFont('',11,QFont.Weight.Bold))
        self.btn_zon.setStyleSheet('''QPushButton{
            background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #1e8a1e,stop:1 #0f4a0f);
            color:white;border-radius:6px;border:1px solid #2acc2a;padding:6px;}
            QPushButton:hover{border-color:#55ff55;}
            QPushButton:pressed{background:#083a08;}''')
        self.btn_zoff.setStyleSheet('''QPushButton{
            background:qlineargradient(x1:0,y1:0,x2:0,y2:1,stop:0 #8a1e1e,stop:1 #4a0f0f);
            color:white;border-radius:6px;border:1px solid #cc2a2a;padding:6px;}
            QPushButton:hover{border-color:#ff5555;}
            QPushButton:pressed{background:#3a0808;}''')
        zl.addWidget(self.btn_zon); zl.addWidget(self.btn_zoff)
        self.lbl_zamp=QLabel('●  Sander OFF')
        self.lbl_zamp.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_zamp.setFixedHeight(32)
        self.lbl_zamp.setFont(QFont('Consolas',11,QFont.Weight.Bold))
        self.lbl_zamp.setStyleSheet(
            'color:#ff4444;background:#1a0808;border-radius:5px;'
            'padding:5px;border:1px solid #441111;font-family:Consolas;'
            'font-size:11px;font-weight:bold;')
        zl.addWidget(self.lbl_zamp); right.addWidget(zb)

        # 6. YOLO Model
        yb=QGroupBox(' YOLO Model')
        yl=QHBoxLayout(yb)
        self.cbo_model=QComboBox()
        self.cbo_model.addItems(['YOLO26s.pt','YOLOv11n.pt','YOLOv8.pt','best.pt'])
        self.btn_model=QPushButton('Load')
        yl.addWidget(self.cbo_model); yl.addWidget(self.btn_model); right.addWidget(yb)

        # 7. Sistem Durumu
        stb=QGroupBox(' System Status')
        stl=QVBoxLayout(stb); stl.setContentsMargins(8,14,8,8); stl.setSpacing(3)
        self.lbl_mission=QLabel('Mission: STANDBY')
        self.lbl_burrs  =QLabel('Detected: 0 burrs')
        self.lbl_servo_s=QLabel('Servo: S1=160° S2=160°')
        self.lbl_servo_s.setStyleSheet('font-family:Consolas;color:#88ccff;font-size:10px;')
        for w in (self.lbl_mission,self.lbl_burrs,self.lbl_servo_s):
            stl.addWidget(w)
        right.addWidget(stb)

        # 8. Doosan Bağlantı — EN ALTTA
        hw=QGroupBox(' Doosan H2515 — Connection')
        hwl=QVBoxLayout(hw); hwl.setContentsMargins(8,14,8,8); hwl.setSpacing(5)
        dr=QHBoxLayout()
        self.lbl_drfl=QLabel('DSR_ROBOT2 initializing...')
        self.lbl_drfl.setStyleSheet('color:#aaa;font-size:10px;')
        dr.addWidget(self.lbl_drfl,1)
        btn_rc=QPushButton('Connect'); btn_rc.setFixedWidth(80)
        btn_rc.clicked.connect(self._reconnect_dsr2); dr.addWidget(btn_rc)
        hwl.addLayout(dr)
        mr=QHBoxLayout()
        self.rb_sim =QRadioButton('Simulation')
        self.rb_real=QRadioButton('Real Hardware')
        self.rb_sim.setChecked(True)
        mr.addWidget(self.rb_sim); mr.addWidget(self.rb_real)
        hwl.addLayout(mr); right.addWidget(hw)
        right.addStretch()

        self.setStyleSheet('''
            QMainWindow,QWidget{background:#0d0d1f;color:#dde0ee;}
            QGroupBox{border:1px solid #2a2a4a;border-radius:7px;margin-top:9px;
                font-weight:bold;font-size:11px;padding:6px;color:#8899cc;}
            QGroupBox::title{subcontrol-origin:margin;left:10px;top:1px;}
            QPushButton{background:#1c1c38;border:1px solid #3a3a5a;
                border-radius:5px;padding:5px;color:#ccd;}
            QPushButton:hover{background:#282850;border-color:#5555aa;}
            QLabel{color:#ccd;}
            QComboBox{background:#1c1c38;border:1px solid #3a3a5a;padding:4px;
                color:#ccd;border-radius:4px;}
            QRadioButton{color:#aab;spacing:6px;}
            QScrollBar:vertical{background:#0d0d1f;width:8px;}
            QScrollBar::handle:vertical{background:#2a2a4a;border-radius:4px;}
            QScrollBar::add-line:vertical,QScrollBar::sub-line:vertical{height:0;}''')

    def _connect_signals(self):
        self.btn_start.clicked.connect(lambda: self.pub_start.publish(Bool(data=True)))
        self.btn_stop.clicked.connect( lambda: self.pub_stop.publish(Bool(data=True)))
        self.btn_emerg.clicked.connect(self._emergency)
        self.btn_cam_on.clicked.connect(self._cam_on)
        self.btn_cam_off.clicked.connect(self._cam_off)
        self.btn_cam_open.clicked.connect(self._camera_box_open)
        self.btn_cam_close.clicked.connect(self._camera_box_close)
        self.btn_servo.clicked.connect(self._send_servo)
        self.btn_zon.clicked.connect(self._sander_on)
        self.btn_zoff.clicked.connect(self._sander_off)
        self.btn_model.clicked.connect(self._set_model)

        self.bridge.log_received.connect(self._log)
        self.bridge.status_received.connect(self._on_status)
        self.bridge.load_cell_received.connect(self._on_lc)
        self.bridge.guidance_received.connect(self._on_guide)
        self.bridge.can_status_received.connect(self._on_can)
        self.bridge.image_ready.connect(self._on_image)

        self._cam_timer=QTimer()
        self._cam_timer.timeout.connect(self._check_cam)
        self._cam_timer.start(3000)

        # Mod radio butonları
        self.rb_sim.toggled.connect(self._on_mode_change)

    # ── Yardımcılar ───────────────────────────────────────────────────────────
    def _log(self, msg: str):
        # Aynı mesaj üst üste tekrar etmesin — spam engeli
        if not hasattr(self, '_last_log_msg'):
            self._last_log_msg = ''
            self._last_log_count = 0
        if msg == self._last_log_msg:
            self._last_log_count += 1
            if self._last_log_count == 3:
                # 3. tekrarda "susturuldu" yaz, sonra sus
                self.log_text.append(f'  ↑ (repeated message suppressed)')
                self.log_text.verticalScrollBar().setValue(
                    self.log_text.verticalScrollBar().maximum())
            return
        self._last_log_msg = msg
        self._last_log_count = 0
        self.log_text.append(msg)
        # Max 500 satır tut — log ekranı dolmasın
        doc = self.log_text.document()
        while doc.blockCount() > 500:
            cursor = self.log_text.textCursor()
            cursor.movePosition(cursor.MoveOperation.Start)
            cursor.select(cursor.SelectionType.BlockUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum())

    def _cam_on(self):
        self.pub_cam.publish(Bool(data=True))
        self.lbl_cam_st.setText('● Starting...')
        self.lbl_cam_st.setStyleSheet('color:#ffaa00;font-family:Consolas;font-size:10px;')

    def _cam_off(self):
        self.pub_cam.publish(Bool(data=False))
        self.lbl_cam_st.setText('● Off')
        self.lbl_cam_st.setStyleSheet('color:#888;font-family:Consolas;font-size:10px;')
        self.lbl_image.setText('Camera off'); self._cur_pix=None

    def _camera_box_open(self):
        """Kamera kutusunu dışarı iter (servo_joint = 0.025m)"""
        self.pub_servo.publish(String(data=json.dumps({'camera': 0.025})))
        self.lbl_cam_box.setText('Box: OPEN')
        self.lbl_cam_box.setStyleSheet('color:#00ccff;font-family:Consolas;font-size:10px;')
        self._log('[CAMERA] Box OPENED (0.025m)')

    def _camera_box_close(self):
        """Kamera kutusunu içeri çeker (servo_joint = 0.0m)"""
        self.pub_servo.publish(String(data=json.dumps({'camera': 0.0})))
        self.lbl_cam_box.setText('Box: CLOSED')
        self.lbl_cam_box.setStyleSheet('color:#555;font-family:Consolas;font-size:10px;')
        self._log('[CAMERA] Box CLOSED (0.0m)')

    def _send_servo(self):
        s1,s2=self.panel_s1.value(),self.panel_s2.value()
        # Sander alanı gönderilmiyor — zimpara durumunu etkilemesin
        self.pub_servo.publish(String(data=json.dumps({'s1':s1,'s2':s2})))
        self.lbl_servo_s.setText(f'Servo: S1={s1}° S2={s2}°')
        self._log(f'[SERVO] Pan/Tilt S1:{s1}° S2:{s2}°')

    def _sander_on(self):
        self.pub_sander.publish(String(data=json.dumps({'sander':SANDER_ON})))
        self.lbl_zamp.setText('●  Sander ON')
        self.lbl_zamp.setStyleSheet('color:#00ff44;background:#001a00;border-radius:5px;'
            'padding:5px;border:1px solid #114411;font-family:Consolas;'
            'font-size:11px;font-weight:bold;')
        self._log('[SANDER] ON')

    def _sander_off(self):
        self.pub_sander.publish(String(data=json.dumps({'sander':SANDER_OFF})))
        self.lbl_zamp.setText('●  Sander OFF')
        self.lbl_zamp.setStyleSheet('color:#ff4444;background:#1a0808;border-radius:5px;'
            'padding:5px;border:1px solid #441111;font-family:Consolas;'
            'font-size:11px;font-weight:bold;')
        self._log('[SANDER] OFF')

    def _emergency(self):
        self.pub_emerg.publish(Bool(data=True))
        self.dsr2.emergency_stop(); self._sander_off()
        self._log('[EMERGENCY] Stopped!')

    def _reconnect_dsr2(self):
        self.dsr2.disconnect()
        threading.Thread(target=self._init_dsr2,daemon=True).start()

    def _set_model(self):
        m=self.cbo_model.currentText()
        self.pub_model.publish(String(data=m))
        self._log(f'[INFO] Model: {m}')

    def _on_mode_change(self):
        is_sim = self.rb_sim.isChecked()
        mode   = 'simulation' if is_sim else 'hardware'
        try:
            self.pub_mode.publish(String(data=mode))
        except Exception:
            pass

        if is_sim:
            self.lbl_can_st.setText('● SIMULATION — sensor data simulated')
            self.lbl_can_st.setStyleSheet('color:#00ccff;font-family:Consolas;font-size:10px;')
            self.lbl_drfl.setText('Simulation mode — Gazebo active')
            self.lbl_drfl.setStyleSheet('color:#00ccff;font-weight:bold;font-size:10px;')
            self.bridge.can_connected = True   # sim modunda load cell göster
            self._log('[MODE] Simulation selected — Gazebo + mock sensor active')
        else:
            self.bridge.can_connected = self._can_ok
            if self._can_ok:
                self.lbl_can_st.setText('● CAN: Connected ✓')
                self.lbl_can_st.setStyleSheet('color:#00ff88;font-family:Consolas;font-size:10px;')
            else:
                self.lbl_can_st.setText('● Real Hardware — waiting for CAN...')
                self.lbl_can_st.setStyleSheet('color:#ffaa00;font-family:Consolas;font-size:10px;')
            self.lbl_drfl.setText('Real Hardware — connecting DSR_ROBOT2...')
            self.lbl_drfl.setStyleSheet('color:#ffaa00;font-weight:bold;font-size:10px;')
            self._log('[MODE] Real Hardware selected — waiting for CAN + camera + DSR_ROBOT2')
            threading.Thread(target=self._init_dsr2, daemon=True).start()

    def _check_cam(self):
        if self._last_frame>0 and (time.time()-self._last_frame)>4.0:
            self.lbl_cam_st.setText('● No signal')
            self.lbl_cam_st.setStyleSheet('color:#ff4444;font-family:Consolas;font-size:10px;')

    # ── UI Güncelleme ─────────────────────────────────────────────────────────
    def _on_status(self,d):
        s,a,e,c=(d.get('is_scanning',False),d.get('mission_active',False),
                  d.get('emergency',False),   d.get('burr_count',0))
        if e:   self.lbl_mission.setText('Mission: ⚠ EMERGENCY STOP')
        elif s: self.lbl_mission.setText('Mission: SCANNING...')
        elif a: self.lbl_mission.setText('Mission: RUNNING')
        else:   self.lbl_mission.setText('Mission: ⏸ STANDBY')
        self.lbl_burrs.setText(f'Detected: {c} burr(s)')

        # Force: gerçek donanımda CAN bağlıyken; simülasyonda otomasyon çalışırken göster
        mission_running = d.get('mission_active', False) or d.get('is_scanning', False)
        show_force = (not self.rb_sim.isChecked() and self._can_ok) or \
                     (self.rb_sim.isChecked() and mission_running)

        if show_force:
            force = d.get('contact_force', 0.0)
            z     = d.get('current_z', 0.0)
            temas = d.get('contact_made', False)
            self.lbl_force_status.setText(
                f'Force: {force:.1f}N  |  Z:{z:.3f}m  |  '
                f'Contact: {"✓" if temas else "NONE"}'
            )
            color = '#00ff44' if temas else '#aaa'
            self.lbl_force_status.setStyleSheet(
                f'color:{color};font-family:Consolas;font-size:10px;'
                'background:#0a0a1a;border-radius:4px;padding:3px;')
        else:
            self.lbl_force_status.setText('Force: — N  |  Contact: NONE')
            self.lbl_force_status.setStyleSheet(
                'color:#444;font-family:Consolas;font-size:10px;'
                'background:#0a0a1a;border-radius:4px;padding:3px;')

    def _on_can(self,ok):
        if self.rb_sim.isChecked():
            # Simülasyon modunda sahte can_status _can_ok'ı kirletmesin
            return
        self._can_ok=ok; self.bridge.can_connected=ok
        if ok:
            self.lbl_can_st.setText('● CAN: Connected ✓')
            self.lbl_can_st.setStyleSheet('color:#00ff88;font-family:Consolas;font-size:10px;')
        else:
            self.lbl_can_st.setText('● CAN: Disconnected — data disabled')
            self.lbl_can_st.setStyleSheet('color:#ff6644;font-family:Consolas;font-size:10px;')
            for bar,lbl in self.lc_bars:
                bar.setValue(0); lbl.setText('—')
                lbl.setStyleSheet('color:#555;font-family:Consolas;')

    def _on_lc(self,vals):
        if not self._can_ok: return
        total=0.0
        for i,(bar,lbl) in enumerate(self.lc_bars):
            v=vals[i] if i<len(vals) else 0.0
            bar.setValue(int(max(0,min(100,v*4))))
            lbl.setText(f'{v:.1f}')
            lbl.setStyleSheet('color:#ccc;font-family:Consolas;')
            total+=max(0,v)
        # Toplam kuvvet gauge — 25N eşiğine yaklaşınca renk değiştir
        self.bar_force.setValue(int(min(60,total)))
        self.lbl_force.setText(f'{total:.1f} N')
        if total>=25:
            self.lbl_force.setStyleSheet(
                'color:#00ff44;font-family:Consolas;font-weight:bold;')
        elif total>=15:
            self.lbl_force.setStyleSheet(
                'color:#ffaa00;font-family:Consolas;font-weight:bold;')
        else:
            self.lbl_force.setStyleSheet(
                'color:#00ccff;font-family:Consolas;font-weight:bold;')

    def _on_guide(self,d):
        txt=(f'{d.get("dir","STANDBY")}\n'
             f'dx:{d.get("dx",0):+.1f}cm  dy:{d.get("dy",0):+.1f}cm')
        self.lbl_guide.setText(txt)

    def _on_image(self, img_bytes: bytes, w: int, h: int, bpl: int):
        """bytes sinyal — BGR bytes olarak gelir, RGB'ye çevirip göster."""
        self._last_frame = time.time()
        self.lbl_cam_st.setText('● Active')
        self.lbl_cam_st.setStyleSheet('color:#00ff88;font-family:Consolas;font-size:10px;')
        try:
            import numpy as np
            # frombuffer read-only döner — copy() ile yazılabilir yap
            bgr = np.frombuffer(img_bytes, dtype=np.uint8).reshape((h, w, 3)).copy()
            # BGR → RGB dönüşümü QImage formatı için
            rgb = bgr[:, :, ::-1]  # cv2 yerine slice — daha hızlı, bağımlılık yok
            qt_img = QImage(rgb.tobytes(), w, h, bpl, QImage.Format.Format_RGB888)
            self._cur_pix = QPixmap.fromImage(qt_img)
            self._redraw()
        except Exception as e:
            self._log(f'[CAM ERROR] {e}')

    def _redraw(self):
        if self._cur_pix is None: return
        lw,lh=self.lbl_image.width(),self.lbl_image.height()
        if lw<10 or lh<10: return
        self.lbl_image.setPixmap(self._cur_pix.scaled(
            lw,lh,Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation))

    def resizeEvent(self,ev):
        super().resizeEvent(ev); self._redraw()

    def closeEvent(self, ev):
        try:
            self.pub_shutdown.publish(Bool(data=True))
            self._log('[INFO] Shutting down...')
        except Exception:
            pass
        # Diğer node'ların mesajı alması ve spin'den çıkması için bekle
        import time as _t; _t.sleep(1.2)
        self.dsr2.disconnect()
        super().closeEvent(ev)


# ── GUINode + main ────────────────────────────────────────────────────────────
class GUINode(Node):
    def __init__(self):
        super().__init__('gui_node')
        self.get_logger().info('GUI node started')


def main(args=None):
    if not PYQT_AVAILABLE:
        print('PyQt6 not installed!'); return

    rclpy.init(args=args)
    ros_node=GUINode()
    ros_node.declare_parameter('use_real_robot', False)
    ros_node.declare_parameter('simulation', True)
    use_real   = ros_node.get_parameter('use_real_robot').get_parameter_value().bool_value
    is_sim_can = ros_node.get_parameter('simulation').get_parameter_value().bool_value
    bridge  =ROSBridge(ros_node)
    dsr2    =Dsr2Layer(node=ros_node, sim=not use_real, logger=ros_node.get_logger())

    app=QApplication(sys.argv); app.setStyle('Fusion')
    win=MainWindow(ros_node,bridge,dsr2,is_sim_can=is_sim_can); win.showMaximized()

    # QTimer ile spin_once — sinyaller Qt main thread'de işlenir, cross-thread sorun olmaz
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))
    ros_timer.start(5)  # 200 Hz

    try:
        ret=app.exec()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
    sys.exit(ret)


if __name__=='__main__':
    main()
