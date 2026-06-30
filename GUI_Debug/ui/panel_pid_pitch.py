"""
panel_pid_pitch.py — Tab hiển thị Cascade PID cho trục PITCH

Vòng ngoài (Outer / Angle PID):
  error = 0 - pitch_abs  →  pid_pitch  →  target_vel_p

Feedforward:
  ff_pitch = K_ff × frame_pitch_rate

Vòng trong (Inner / Velocity PID):
  vel_error = (target_vel_p + ff_pitch) - cam_pitch_rate  →  pid_vel  →  Vq_pitch
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                              QLabel, QFrame, QSizePolicy, QGridLayout)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


LABEL_STYLE = {'color': '#AAAAAA', 'font-size': '9pt'}

# Màu
C_ERR     = '#FF6B6B'    # đỏ     - error góc
C_TVEL    = '#4ECDC4'    # ngọc   - target velocity
C_FF      = '#FFB74D'    # cam    - feedforward
C_CAMRATE = '#81C784'    # xanh   - camera rate (feedback inner)
C_VELERR  = '#CE93D8'    # tím    - velocity error
C_VQ      = '#FFCC02'    # vàng   - Vq output


def make_plot(title: str, y_label: str, y_range=None) -> pg.PlotWidget:
    pw = pg.PlotWidget()
    pw.setBackground('#1A1A2E')
    pw.showGrid(x=True, y=True, alpha=0.25)
    pw.setLabel('left', y_label, **LABEL_STYLE)
    pw.setLabel('bottom', 'Time (s)', **LABEL_STYLE)
    pw.getAxis('left').setTextPen('#CCCCCC')
    pw.getAxis('bottom').setTextPen('#CCCCCC')
    pw.setTitle(title, color='#E0E0E0', size='10pt')
    pw.addLegend(offset=(5, 5))
    if y_range:
        pw.setYRange(*y_range)
    pw.addLine(y=0, pen=pg.mkPen('#555566', width=1, style=Qt.PenStyle.DashLine))
    return pw


class NumericCard(QFrame):
    """Widget hiển thị 1 số dạng card nhỏ."""

    def __init__(self, label: str, unit: str, color: str, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame {{
                background: #1A1A2E;
                border: 1px solid {color};
                border-radius: 6px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)

        lbl = QLabel(label)
        lbl.setFont(QFont("Inter", 8))
        lbl.setStyleSheet(f"color: {color}; border: none;")

        self.val_label = QLabel("---")
        self.val_label.setFont(QFont("Monospace", 13, QFont.Weight.Bold))
        self.val_label.setStyleSheet(f"color: #FFFFFF; border: none;")
        self.val_label.setAlignment(Qt.AlignmentFlag.AlignRight)

        unit_lbl = QLabel(unit)
        unit_lbl.setFont(QFont("Inter", 7))
        unit_lbl.setStyleSheet("color: #888888; border: none;")
        unit_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)

        layout.addWidget(lbl)
        layout.addWidget(self.val_label)
        layout.addWidget(unit_lbl)

    def update_value(self, v: float):
        self.val_label.setText(f"{v:+.3f}")


class PIDPitchPanel(QWidget):
    """Tab 3: PID Cascade — Trục PITCH."""

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Title + PID params
        hdr = QHBoxLayout()
        title = QLabel("🎯  Cascade PID — Trục PITCH")
        title.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0E0E0;")
        hdr.addWidget(title)
        hdr.addStretch()
        root.addLayout(hdr)

        # ---- Numeric cards ----
        cards_row = QHBoxLayout()
        self.card_err    = NumericCard("Angle Error",    "deg",   C_ERR)
        self.card_tvel   = NumericCard("Target Vel",     "rad/s", C_TVEL)
        self.card_ff     = NumericCard("Feedforward",    "rad/s", C_FF)
        self.card_camr   = NumericCard("Camera Rate",    "rad/s", C_CAMRATE)
        self.card_verr   = NumericCard("Vel Error",      "rad/s", C_VELERR)
        self.card_vq     = NumericCard("Vq Pitch",       "V",     C_VQ)

        for c in (self.card_err, self.card_tvel, self.card_ff,
                  self.card_camr, self.card_verr, self.card_vq):
            cards_row.addWidget(c)
        root.addLayout(cards_row)

        # ---- PID params info ----
        params_frame = QFrame()
        params_frame.setStyleSheet("background:#12122A; border-radius:6px; padding:4px;")
        params_layout = QHBoxLayout(params_frame)

        outer_lbl = QLabel(
            "  <b>Outer PID (Angle):</b>  "
            "<span style='color:#4FC3F7'>Kp=2.0</span>  "
            "<span style='color:#81C784'>Ki=0.0</span>  "
            "<span style='color:#FF6B6B'>Kd=0.0</span>  │  "
            "Output: ±10 rad/s"
        )
        inner_lbl = QLabel(
            "  <b>Inner PID (Velocity):</b>  "
            "<span style='color:#4FC3F7'>Kp=0.1</span>  "
            "<span style='color:#81C784'>Ki=0.01</span>  "
            "<span style='color:#FF6B6B'>Kd=0.0</span>  │  "
            "LPF α=0.98  │  Output: ±voltage_limit V"
        )
        outer_lbl.setStyleSheet("color:#CCCCCC; font-size:9pt;")
        inner_lbl.setStyleSheet("color:#CCCCCC; font-size:9pt;")
        params_layout.addWidget(outer_lbl)
        params_layout.addWidget(inner_lbl)
        root.addWidget(params_frame)

        # ---- Plots ----
        plots_col = QVBoxLayout()

        self.pw_outer = make_plot(
            "Outer PID — Angle Loop  (Pitch error → Target velocity)",
            "deg / (rad/s)"
        )
        self.pw_ff = make_plot(
            "Feedforward + Inner velocity error",
            "rad/s"
        )
        self.pw_inner = make_plot(
            "Inner PID — Velocity Loop  (Vel error → Vq)",
            "rad/s | V"
        )

        for pw in (self.pw_outer, self.pw_ff, self.pw_inner):
            plots_col.addWidget(pw)

        root.addLayout(plots_col)

        # Curves
        pw = 1.8
        self.c_err    = self.pw_outer.plot(pen=pg.mkPen(C_ERR,     width=pw),   name="Angle Error (deg)")
        self.c_tvel   = self.pw_outer.plot(pen=pg.mkPen(C_TVEL,    width=pw),   name="Target Vel (rad/s)")

        self.c_ff     = self.pw_ff.plot(pen=pg.mkPen(C_FF,       width=pw),   name="Feedforward (rad/s)")
        self.c_camr   = self.pw_ff.plot(pen=pg.mkPen(C_CAMRATE,  width=pw),   name="Camera Rate (rad/s)")
        self.c_verr   = self.pw_ff.plot(pen=pg.mkPen(C_VELERR,   width=1.2),  name="Vel Error (rad/s)")

        self.c_verr2  = self.pw_inner.plot(pen=pg.mkPen(C_VELERR, width=pw),   name="Vel Error (rad/s)")
        self.c_vq     = self.pw_inner.plot(pen=pg.mkPen(C_VQ,     width=2.2),  name="Vq (V)")

    def refresh(self):
        t = self.store.get_time()
        if len(t) < 2:
            return

        err   = self.store.get(self.store.p_err)
        tvel  = self.store.get(self.store.t_vel_p)
        ff    = self.store.get(self.store.ff_pitch)
        camr  = self.store.get(self.store.cam_pr)
        verr  = self.store.get(self.store.vel_err_p)
        vq    = self.store.get(self.store.vq_pitch)

        def _plot(curve, t_arr, d_arr):
            n = min(len(t_arr), len(d_arr))
            if n > 1:
                curve.setData(t_arr[-n:], d_arr[-n:])

        _plot(self.c_err,   t, err)
        _plot(self.c_tvel,  t, tvel)
        _plot(self.c_ff,    t, ff)
        _plot(self.c_camr,  t, camr)
        _plot(self.c_verr,  t, verr)
        _plot(self.c_verr2, t, verr)
        _plot(self.c_vq,    t, vq)

        # Cards
        if len(err):  self.card_err.update_value(err[-1])
        if len(tvel): self.card_tvel.update_value(tvel[-1])
        if len(ff):   self.card_ff.update_value(ff[-1])
        if len(camr): self.card_camr.update_value(camr[-1])
        if len(verr): self.card_verr.update_value(verr[-1])
        if len(vq):   self.card_vq.update_value(vq[-1])
