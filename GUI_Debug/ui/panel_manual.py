"""
panel_manual.py — Tab điều khiển Motor thủ công
"""

from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                              QGroupBox, QLabel, QRadioButton,
                              QDoubleSpinBox, QPushButton, QGridLayout)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

class ManualControlPanel(QWidget):
    """Tab cho phép điều khiển vận tốc 2 trục Pitch/Roll bằng chuột."""
    def __init__(self, send_cb, parent=None):
        super().__init__(parent)
        self.send_cb = send_cb
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(20, 20, 20, 20)
        root.setSpacing(20)

        # Mode Selection
        grp_mode = QGroupBox("System Mode")
        grp_mode.setStyleSheet("color: #E0E0E0; font-weight: bold; font-size: 11pt;")
        lay_mode = QHBoxLayout(grp_mode)
        
        self.rad_gimbal = QRadioButton("Gimbal Mode (Auto)")
        self.rad_manual = QRadioButton("Manual Control Mode")
        self.rad_gimbal.setChecked(True)
        self.rad_gimbal.setStyleSheet("color: #4ECDC4; font-weight: bold;")
        self.rad_manual.setStyleSheet("color: #FF6B6B; font-weight: bold;")
        
        self.rad_gimbal.toggled.connect(self._on_mode_change)
        self.rad_manual.toggled.connect(self._on_mode_change)
        
        lay_mode.addWidget(self.rad_gimbal)
        lay_mode.addWidget(self.rad_manual)
        lay_mode.addStretch()
        root.addWidget(grp_mode)

        # Settings
        grp_set = QGroupBox("Limits & Settings")
        grp_set.setStyleSheet("color: #E0E0E0; font-weight: bold; font-size: 11pt;")
        lay_set = QHBoxLayout(grp_set)
        
        lay_set.addWidget(QLabel("Target Speed:"))
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 20.0)
        self.spin_speed.setValue(2.0)
        self.spin_speed.setSingleStep(0.5)
        self.spin_speed.setSuffix(" rad/s")
        lay_set.addWidget(self.spin_speed)

        lay_set.addSpacing(30)

        lay_set.addWidget(QLabel("Torque Limit:"))
        self.spin_volt = QDoubleSpinBox()
        self.spin_volt.setRange(0.1, 5.0)
        self.spin_volt.setValue(1.0)
        self.spin_volt.setSingleStep(0.1)
        self.spin_volt.setSuffix(" V")
        lay_set.addWidget(self.spin_volt)
        
        btn_apply_volt = QPushButton("Apply Voltage")
        btn_apply_volt.setStyleSheet("background: #4ECDC4; color: #12122A;")
        btn_apply_volt.clicked.connect(self._on_voltage_change)
        lay_set.addWidget(btn_apply_volt)
        lay_set.addStretch()
        root.addWidget(grp_set)

        # Joystick (D-Pad)
        grp_joy = QGroupBox("Motor Direction Control (Hold to Move)")
        grp_joy.setStyleSheet("color: #E0E0E0; font-weight: bold; font-size: 11pt;")
        lay_joy = QGridLayout(grp_joy)
        
        btn_up = QPushButton("▲ PITCH UP")
        btn_down = QPushButton("▼ PITCH DOWN")
        btn_left = QPushButton("◀ ROLL LEFT")
        btn_right = QPushButton("▶ ROLL RIGHT")

        for btn in (btn_up, btn_down, btn_left, btn_right):
            btn.setFixedSize(140, 60)
            btn.setStyleSheet("""
                QPushButton {
                    background: #2A2A4A; 
                    color: white; 
                    font-weight: bold; 
                    border-radius: 8px;
                }
                QPushButton:pressed {
                    background: #FF6B6B;
                }
            """)
            
        lay_joy.addWidget(btn_up, 0, 1)
        lay_joy.addWidget(btn_left, 1, 0)
        lay_joy.addWidget(btn_right, 1, 2)
        lay_joy.addWidget(btn_down, 2, 1)
        lay_joy.setAlignment(Qt.AlignmentFlag.AlignCenter)
        
        root.addWidget(grp_joy)
        root.addStretch()

        # Connections
        btn_up.pressed.connect(lambda: self._send_move("PITCH", 1))
        btn_up.released.connect(lambda: self._send_move("PITCH", 0))
        btn_down.pressed.connect(lambda: self._send_move("PITCH", -1))
        btn_down.released.connect(lambda: self._send_move("PITCH", 0))

        btn_left.pressed.connect(lambda: self._send_move("ROLL", -1))
        btn_left.released.connect(lambda: self._send_move("ROLL", 0))
        btn_right.pressed.connect(lambda: self._send_move("ROLL", 1))
        btn_right.released.connect(lambda: self._send_move("ROLL", 0))

    def _on_mode_change(self):
        if self.rad_manual.isChecked():
            self.send_cb("$CMD,MODE,MANUAL\r\n")
        else:
            self.send_cb("$CMD,MODE,GIMBAL\r\n")

    def _on_voltage_change(self):
        v = self.spin_volt.value()
        self.send_cb(f"$CMD,LIMIT,VOLTAGE,{v:.2f}\r\n")

    def _send_move(self, axis: str, dir_sign: int):
        speed = self.spin_speed.value() * dir_sign
        self.send_cb(f"$CMD,MOVE,{axis},{speed:.2f}\r\n")

    def refresh(self):
        pass
