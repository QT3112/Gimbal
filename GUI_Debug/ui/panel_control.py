"""
panel_control.py — Tab điều khiển vị trí & vận tốc

Giao diện:
  - Mode selector (Position / Velocity)
  - Position: quick buttons, custom angle, step sequence editor
  - Velocity: slider + quick buttons + manual input
  - Motor safety: STOP / START / ALIGN
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QPushButton, QDoubleSpinBox, QSlider, QLineEdit,
    QListWidget, QListWidgetItem, QSpinBox, QGroupBox,
    QButtonGroup, QRadioButton, QSizePolicy, QSpacerItem
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFont, QColor


QUICK_POS = [(-180, "-180°"), (-90, "-90°"), (-45, "-45°"),
             (0,   "0°"),   (45, "45°"),   (90, "90°"),
             (135, "135°"), (180, "180°")]

QUICK_VEL = [(-5, "-5"), (-3, "-3"), (-1, "-1"),
             (0,  "0"),  (1,  "+1"), (3,  "+3"), (5, "+5")]


def _group(title: str) -> QGroupBox:
    g = QGroupBox(title)
    g.setStyleSheet("""
        QGroupBox {
            color: #AAAAAA;
            font-size: 9pt;
            border: 1px solid #2A2A4A;
            border-radius: 6px;
            margin-top: 8px;
            padding-top: 6px;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
            color: #4ECDC4;
            font-weight: bold;
        }
    """)
    return g


def _btn(text: str, color: str = '#2A2A4A', text_color: str = '#E0E0E0',
         border: str = '#3A3A5A', width: int = None) -> QPushButton:
    b = QPushButton(text)
    b.setStyleSheet(f"""
        QPushButton {{
            background: {color}; color: {text_color};
            border: 1px solid {border}; border-radius: 5px;
            padding: 5px 12px; font-weight: bold;
        }}
        QPushButton:hover  {{ background: #3A3A6A; border-color: #4ECDC4; }}
        QPushButton:pressed{{ background: #4A4A7A; }}
    """)
    if width:
        b.setFixedWidth(width)
    return b


class ControlPanel(QWidget):
    """Tab 2: Motor Control Panel."""

    # signal phát ra khi có lệnh cần gửi
    command_ready = pyqtSignal(str)

    def __init__(self, sender, parent=None):
        super().__init__(parent)
        self.sender = sender
        self._seq_timer = QTimer(self)
        self._seq_timer.timeout.connect(self._seq_step)
        self._seq_index = 0
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # Title
        title = QLabel("🎛  Motor Control")
        title.setFont(QFont("Inter", 13, QFont.Weight.Bold))
        title.setStyleSheet("color: #4ECDC4;")
        root.addWidget(title)

        # ─────────── Mode selector ───────────
        mode_grp = _group("Control Mode")
        mode_lay = QHBoxLayout(mode_grp)
        self.rb_pos = QRadioButton("📍  Position Mode")
        self.rb_vel = QRadioButton("💨  Velocity Mode")
        self.rb_pos.setChecked(True)
        for rb in (self.rb_pos, self.rb_vel):
            rb.setStyleSheet("color: #E0E0E0; font-size: 11pt;")
        self.rb_pos.toggled.connect(self._on_mode_changed)
        mode_lay.addWidget(self.rb_pos)
        mode_lay.addWidget(self.rb_vel)
        mode_lay.addStretch()
        root.addWidget(mode_grp)

        # ─────────── Position Control ───────────
        self._pos_grp = _group("Position Control")
        pos_lay = QVBoxLayout(self._pos_grp)

        # Quick buttons
        quick_lbl = QLabel("Quick targets:")
        quick_lbl.setStyleSheet("color: #888888; font-size: 9pt;")
        pos_lay.addWidget(quick_lbl)
        btn_row = QHBoxLayout()
        for deg, label in QUICK_POS:
            b = _btn(label, '#1A2A3A', '#4ECDC4', '#334466')
            b.clicked.connect(lambda _, d=deg: self._send_pos(float(d)))
            btn_row.addWidget(b)
        pos_lay.addLayout(btn_row)

        # Custom angle
        cust_row = QHBoxLayout()
        cust_lbl = QLabel("Custom angle:")
        cust_lbl.setStyleSheet("color: #AAAAAA;")
        self.spin_pos = QDoubleSpinBox()
        self.spin_pos.setRange(-360.0, 360.0)
        self.spin_pos.setDecimals(1)
        self.spin_pos.setSuffix(" °")
        self.spin_pos.setValue(0.0)
        self.spin_pos.setFixedWidth(100)
        self.spin_pos.setStyleSheet(
            "background:#1A1A2E; border:1px solid #3A3A5A; border-radius:4px;"
            " color:#E0E0E0; padding:3px;")
        go_btn = _btn("→  GO", '#1B4332', '#4ECDC4', '#2E8B57', 80)
        go_btn.clicked.connect(lambda: self._send_pos(self.spin_pos.value()))
        cust_row.addWidget(cust_lbl)
        cust_row.addWidget(self.spin_pos)
        cust_row.addWidget(go_btn)
        cust_row.addStretch()
        pos_lay.addLayout(cust_row)

        # Step sequence
        pos_lay.addWidget(QLabel("Step Sequence:").setStyleSheet if False else self._make_sep("Step Sequence"))
        seq_row = QHBoxLayout()
        self.seq_list = QListWidget()
        self.seq_list.setMaximumHeight(90)
        self.seq_list.setStyleSheet(
            "background:#12122A; border:1px solid #2A2A4A; border-radius:4px;"
            " color:#E0E0E0; font-size:9pt;")
        # Mặc định
        for deg in [0, 90, 180, -90]:
            self.seq_list.addItem(QListWidgetItem(f"{deg}°"))

        seq_ctrl = QVBoxLayout()
        add_spin = QDoubleSpinBox()
        add_spin.setRange(-360, 360)
        add_spin.setDecimals(1)
        add_spin.setSuffix(" °")
        add_spin.setValue(0)
        add_spin.setFixedWidth(90)
        add_spin.setStyleSheet(
            "background:#1A1A2E; border:1px solid #3A3A5A; color:#E0E0E0;"
            " padding:2px;")
        self._add_spin = add_spin

        add_btn = _btn("＋ Add", width=80)
        add_btn.clicked.connect(self._seq_add)
        del_btn = _btn("− Del", '#3A1A1A', '#FF6B6B', '#AA3333', 80)
        del_btn.clicked.connect(self._seq_del)

        seq_ctrl.addWidget(add_spin)
        seq_ctrl.addWidget(add_btn)
        seq_ctrl.addWidget(del_btn)

        seq_row.addWidget(self.seq_list, 1)
        seq_row.addLayout(seq_ctrl)
        pos_lay.addLayout(seq_row)

        # Step time + Run/Stop
        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step time:").setStyleSheet if False else _plain_lbl("Step time:"))
        self.spin_step_time = QSpinBox()
        self.spin_step_time.setRange(500, 30000)
        self.spin_step_time.setSingleStep(500)
        self.spin_step_time.setValue(4000)
        self.spin_step_time.setSuffix(" ms")
        self.spin_step_time.setFixedWidth(100)
        self.spin_step_time.setStyleSheet(
            "background:#1A1A2E; border:1px solid #3A3A5A; color:#E0E0E0; padding:2px;")
        self.btn_run_seq = _btn("▶  Run Sequence", '#1B3A1B', '#81C784', '#336633')
        self.btn_run_seq.clicked.connect(self._seq_toggle)
        self.btn_stop_seq = _btn("⏹  Stop", '#3A1A1A', '#FF6B6B', '#AA3333')
        self.btn_stop_seq.clicked.connect(self._seq_stop)
        step_row.addWidget(self.spin_step_time)
        step_row.addSpacing(10)
        step_row.addWidget(self.btn_run_seq)
        step_row.addWidget(self.btn_stop_seq)
        step_row.addStretch()
        pos_lay.addLayout(step_row)

        root.addWidget(self._pos_grp)

        # ─────────── Velocity Control ───────────
        self._vel_grp = _group("Velocity Control")
        vel_lay = QVBoxLayout(self._vel_grp)

        # Slider
        slider_row = QHBoxLayout()
        lbl_min = QLabel("-10")
        lbl_min.setStyleSheet("color:#888888;")
        self.vel_slider = QSlider(Qt.Orientation.Horizontal)
        self.vel_slider.setRange(-100, 100)
        self.vel_slider.setValue(0)
        self.vel_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                height: 6px; background: #2A2A4A; border-radius: 3px;
            }
            QSlider::handle:horizontal {
                background: #4ECDC4; border: none;
                width: 18px; height: 18px; border-radius: 9px;
                margin: -6px 0;
            }
            QSlider::sub-page:horizontal { background: #4ECDC4; border-radius: 3px; }
        """)
        self.vel_slider.valueChanged.connect(self._on_slider_changed)
        lbl_max = QLabel("+10")
        lbl_max.setStyleSheet("color:#888888;")
        self.lbl_slider_val = QLabel("0.0 rad/s")
        self.lbl_slider_val.setFont(QFont("Monospace", 11, QFont.Weight.Bold))
        self.lbl_slider_val.setStyleSheet("color:#4ECDC4; min-width:90px;")
        slider_row.addWidget(lbl_min)
        slider_row.addWidget(self.vel_slider, 1)
        slider_row.addWidget(lbl_max)
        slider_row.addSpacing(12)
        slider_row.addWidget(self.lbl_slider_val)
        vel_lay.addLayout(slider_row)

        # Quick velocity buttons
        qv_row = QHBoxLayout()
        for rads, label in QUICK_VEL:
            b = _btn(label, '#1A2A3A', '#4ECDC4', '#334466')
            b.clicked.connect(lambda _, v=rads: self._send_vel(float(v)))
            qv_row.addWidget(b)
        vel_lay.addLayout(qv_row)

        # Manual input
        man_row = QHBoxLayout()
        man_lbl = _plain_lbl("Manual (rad/s):")
        self.spin_vel = QDoubleSpinBox()
        self.spin_vel.setRange(-50.0, 50.0)
        self.spin_vel.setDecimals(2)
        self.spin_vel.setSuffix(" rad/s")
        self.spin_vel.setValue(0.0)
        self.spin_vel.setFixedWidth(120)
        self.spin_vel.setStyleSheet(
            "background:#1A1A2E; border:1px solid #3A3A5A; border-radius:4px;"
            " color:#E0E0E0; padding:3px;")
        set_btn = _btn("→  SET", '#1B4332', '#4ECDC4', '#2E8B57', 80)
        set_btn.clicked.connect(lambda: self._send_vel(self.spin_vel.value()))
        man_row.addWidget(man_lbl)
        man_row.addWidget(self.spin_vel)
        man_row.addWidget(set_btn)
        man_row.addStretch()
        vel_lay.addLayout(man_row)

        root.addWidget(self._vel_grp)
        self._vel_grp.setVisible(False)

        # ─────────── Motor Safety ───────────
        safety_grp = _group("Motor Safety")
        safety_lay = QHBoxLayout(safety_grp)

        self.btn_estop = QPushButton("🔴  EMERGENCY STOP")
        self.btn_estop.setStyleSheet("""
            QPushButton {
                background: #6B0000; color: #FF6B6B; font-size: 13pt; font-weight: bold;
                border: 2px solid #FF3333; border-radius: 8px; padding: 10px 20px;
            }
            QPushButton:hover  { background: #8B0000; border-color: #FF6666; }
            QPushButton:pressed{ background: #AA0000; }
        """)
        self.btn_estop.clicked.connect(self._on_estop)

        self.btn_enable = _btn("🟢  Enable Motor", '#0A3A0A', '#81C784', '#336633')
        self.btn_enable.setFixedHeight(50)
        self.btn_enable.setFont(QFont("Inter", 11, QFont.Weight.Bold))
        self.btn_enable.clicked.connect(self._on_enable)

        self.btn_align = _btn("⚙️  Re-Align", '#1A1A3A', '#CE93D8', '#663399')
        self.btn_align.setFixedHeight(50)
        self.btn_align.setFont(QFont("Inter", 10))
        self.btn_align.clicked.connect(self._on_align)

        safety_lay.addWidget(self.btn_estop, 2)
        safety_lay.addWidget(self.btn_enable, 1)
        safety_lay.addWidget(self.btn_align,  1)
        root.addWidget(safety_grp)

        root.addStretch()

    # ------------------------------------------------------------------ helpers
    def _make_sep(self, text: str) -> QLabel:
        lbl = QLabel(text)
        lbl.setStyleSheet("color:#888888; font-size:9pt; margin-top:4px;")
        return lbl

    # ------------------------------------------------------------------ mode
    def _on_mode_changed(self, checked: bool):
        is_pos = self.rb_pos.isChecked()
        self._pos_grp.setVisible(is_pos)
        self._vel_grp.setVisible(not is_pos)
        if is_pos:
            self.sender.set_mode_position()
        else:
            self.sender.set_mode_velocity()

    # ------------------------------------------------------------------ position
    def _send_pos(self, deg: float):
        self.sender.set_target_position(deg)

    # ------------------------------------------------------------------ velocity
    def _send_vel(self, rad_s: float):
        self.vel_slider.blockSignals(True)
        self.vel_slider.setValue(int(rad_s * 10))
        self.vel_slider.blockSignals(False)
        self.lbl_slider_val.setText(f"{rad_s:.1f} rad/s")
        self.sender.set_target_velocity(rad_s)

    def _on_slider_changed(self, val: int):
        rad_s = val / 10.0
        self.lbl_slider_val.setText(f"{rad_s:.1f} rad/s")
        self.sender.set_target_velocity(rad_s)

    # ------------------------------------------------------------------ sequence
    def _seq_add(self):
        deg = self._add_spin.value()
        self.seq_list.addItem(QListWidgetItem(f"{deg:.1f}°"))

    def _seq_del(self):
        row = self.seq_list.currentRow()
        if row >= 0:
            self.seq_list.takeItem(row)

    def _seq_toggle(self):
        if self._seq_timer.isActive():
            self._seq_stop()
        else:
            if self.seq_list.count() == 0:
                return
            self._seq_index = 0
            ms = self.spin_step_time.value()
            self._seq_timer.start(ms)
            self.btn_run_seq.setText("⏸  Pause Sequence")
            self._seq_step()

    def _seq_stop(self):
        self._seq_timer.stop()
        self.btn_run_seq.setText("▶  Run Sequence")

    def _seq_step(self):
        n = self.seq_list.count()
        if n == 0:
            self._seq_stop()
            return
        item = self.seq_list.item(self._seq_index % n)
        text = item.text().replace('°', '').strip()
        try:
            self._send_pos(float(text))
        except ValueError:
            pass
        self.seq_list.setCurrentRow(self._seq_index % n)
        self._seq_index += 1

    # ------------------------------------------------------------------ safety
    def _on_estop(self):
        self.sender.stop_motor()
        self._seq_stop()
        # Đặt slider về 0
        self.vel_slider.setValue(0)

    def _on_enable(self):
        self.sender.start_motor()

    def _on_align(self):
        self.sender.re_align()


def _plain_lbl(text: str) -> QLabel:
    l = QLabel(text)
    l.setStyleSheet("color: #AAAAAA;")
    return l
