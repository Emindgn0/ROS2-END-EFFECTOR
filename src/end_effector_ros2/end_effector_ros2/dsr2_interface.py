#!/usr/bin/env python3
"""
dsr2_interface.py — DSR_ROBOT2 Tabanlı Paylaşılan I/O Katmanı
================================================================
can_node.py ve gui_node.py için zımpara röle (tool digital output),
kuvvet sensörü okuma (tool force) ve acil durdurma (move_stop servisi)
arayüzü. Eski DRFL (doğrudan soket, port 12345) katmanının yerini alır —
robot hareketi (movej/movel) yalnızca logic_node.py üzerinden DSR_ROBOT2
ile gönderilir, böylece controller'a tek bir harici kontrol bağlantısı
kalır.

ÖNEMLİ: DSR_ROBOT2 modülü import edildiği anda DR_init.__dsr__node
üzerinden dsr_control2 servis client'larını oluşturur. Bu yüzden
DR_init.__dsr__node, import'tan ÖNCE atanmış olmalı (bkz. movel_test.py
run_real()).
"""

import math
import random
import time

ROBOT_ID         = 'dsr01'
ROBOT_MODEL      = 'h2515'


class Dsr2Layer:
    """DSR_ROBOT2 servisleri üzerinden tool I/O + kuvvet + acil durdurma."""

    def __init__(self, node, sim, logger):
        self.node      = node
        self.sim       = sim
        self.log       = logger
        self.connected = False
        self._get_tool_force          = None
        self._set_tool_digital_output = None
        self._move_stop_client        = None
        self._MoveStop                = None

    def connect(self) -> bool:
        if self.sim:
            self.connected = True
            self.log.info('[DSR2] Simülasyon modu aktif')
            return True
        try:
            import DR_init
            # setattr zorunlu: class metodu içinde __dsr__* isimleri Python name
            # mangling ile _Dsr2Layer__dsr__* olur; setattr bunu bypass eder.
            setattr(DR_init, '__dsr__id',    ROBOT_ID)
            setattr(DR_init, '__dsr__model', ROBOT_MODEL)
            setattr(DR_init, '__dsr__node',  self.node)

            from DSR_ROBOT2 import get_tool_force, set_tool_digital_output
            from dsr_msgs2.srv import MoveStop

            self._get_tool_force          = get_tool_force
            self._set_tool_digital_output = set_tool_digital_output
            self._MoveStop                = MoveStop
            self._move_stop_client = self.node.create_client(
                MoveStop, 'motion/move_stop')

            self.connected = True
            self.log.info('[DSR2] DSR_ROBOT2 hazır (dsr_control2 servisleri)')
            return True
        except Exception as e:
            self.connected = False
            self.log.error(f'[DSR2] Bağlantı hatası: {e}')
            return False

    def disconnect(self):
        self.connected = False

    def get_tcp_force(self):
        """Doosan dahili tool force sensörü (6 eksen)."""
        if not self.connected:
            return [0.0] * 6
        if self.sim:
            base = 5.0 + 3.0 * math.sin(time.time() * 0.5)
            return [round(base + random.gauss(0, 0.2), 2) for _ in range(6)]
        try:
            return list(self._get_tool_force())
        except Exception as e:
            self.log.error(f'[DSR2] get_tool_force: {e}')
            return [0.0] * 6

    def set_digital_output(self, port: int, val: bool) -> bool:
        """Flange/tool dijital çıkış — zımpara röle."""
        if not self.connected:
            return False
        if self.sim:
            self.log.debug(f'[DSR2-SIM] tool DO[{port}]={val}')
            return True
        try:
            self._set_tool_digital_output(port, 1 if val else 0)
            return True
        except Exception as e:
            self.log.error(f'[DSR2] set_tool_digital_output: {e}')
            return False

    def halt(self):
        """Yumuşak durdurma — e-stop'tan önce hareketi nazikçe kes (stop_mode=2)."""
        self._move_stop(stop_mode=2)

    def emergency_stop(self):
        """Quick stop with STO — donanım acil durdurma (stop_mode=0)."""
        self._move_stop(stop_mode=0)

    def _move_stop(self, stop_mode: int):
        if self.sim or not self.connected or self._move_stop_client is None:
            self.log.warn(f'[DSR2-SIM] MOVE STOP mode={stop_mode}')
            return
        try:
            req = self._MoveStop.Request()
            req.stop_mode = stop_mode
            self._move_stop_client.call_async(req)
        except Exception as e:
            self.log.error(f'[DSR2] move_stop: {e}')
