"""
panel_attitude.py — Tab hiển thị Attitude Estimation output

Hiển thị:
  - Góc Euler tuyệt đối của Frame và Payload [deg]
  - Góc tương đối motor (relative_pitch, relative_roll) [deg]
  - Gauge trực quan cho Pitch/Roll/Yaw hiện tại
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                              QLabel, QFrame, QSizePolicy)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QPainter, QPen, QColor, QBrush


LABEL_STYLE = {'color': '#AAAAAA', 'font-size': '9pt'}

# Màu sắc
C_FRAME_P   = '#4FC3F7'   # xanh dương nhạt - frame pitch
C_FRAME_R   = '#81C784'   # xanh lá - frame roll
C_FRAME_Y   = '#FFB74D'   # cam - frame yaw
C_PAY_P     = '#FF6B6B'   # đỏ - payload pitch
C_PAY_R     = '#CE93D8'   # tím - payload roll
C_PAY_Y     = '#FFCC02'   # vàng - payload yaw
C_REL_P     = '#4ECDC4'   # ngọc - relative pitch
C_REL_R     = '#F06292'   # hồng - relative roll


class AngleGauge(QWidget):
    """Widget hiển thị góc dạng thanh trượt ngang -180°…+180°."""

    def __init__(self, label: str, color: str, parent=None):
        super().__init__(parent)
        self._value = 0.0
        self._label = label
        self._color = QColor(color)
        self.setMinimumHeight(32)
        self.setMaximumHeight(48)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_value(self, v: float):
        self._value = max(-180.0, min(180.0, v))
        self.update()

    def paintEvent(self, event):
        w, h = self.width(), self.height()
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        p.fillRect(0, 0, w, h, QColor('#1A1A2E'))

        # Track
        track_h = 8
        track_y = h // 2 - track_h // 2
        p.fillRect(10, track_y, w - 20, track_h, QColor('#2A2A4A'))

        # Fill bar
        ratio = (self._value + 180.0) / 360.0
        bar_w = int((w - 20) * ratio)
        if bar_w > 0:
            p.fillRect(10, track_y, bar_w, track_h, self._color)

        # Center marker
        cx = w // 2
        p.setPen(QPen(QColor('#FFFFFF'), 1))
        p.drawLine(cx, track_y - 2, cx, track_y + track_h + 2)

        # Text
        p.setPen(QColor('#E0E0E0'))
        p.setFont(QFont("Monospace", 9))
        label_txt = f"{self._label}: {self._value:+7.2f}°"
        p.drawText(15, h // 2 + 4, label_txt)
        p.end()


def make_plot(title: str, y_label: str = "deg") -> pg.PlotWidget:
    pw = pg.PlotWidget()
    pw.setBackground('#1A1A2E')
    pw.showGrid(x=True, y=True, alpha=0.25)
    pw.setLabel('left', y_label, **LABEL_STYLE)
    pw.setLabel('bottom', 'Time (s)', **LABEL_STYLE)
    pw.getAxis('left').setTextPen('#CCCCCC')
    pw.getAxis('bottom').setTextPen('#CCCCCC')
    pw.setTitle(title, color='#E0E0E0', size='10pt')
    pw.addLegend(offset=(5, 5))
    return pw


class AttitudePanel(QWidget):
    """Tab 2: Attitude Estimation — Frame, Payload, Relative angles."""

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Title
        title = QLabel("🧭  Attitude Estimation (Mahony Filter)")
        title.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0E0E0;")
        root.addWidget(title)

        # ---- Gauge row ----
        gauge_box = QFrame()
        gauge_box.setStyleSheet("background:#12122A; border-radius:6px;")
        gauge_layout = QVBoxLayout(gauge_box)
        gauge_layout.setContentsMargins(6, 4, 6, 4)
        gauge_layout.setSpacing(2)

        self.g_pay_p = AngleGauge("Payload Pitch", C_PAY_P)
        self.g_pay_r = AngleGauge("Payload Roll ", C_PAY_R)
        self.g_pay_y = AngleGauge("Payload Yaw  ", C_PAY_Y)
        self.g_rel_p = AngleGauge("Relative Pitch", C_REL_P)
        self.g_rel_r = AngleGauge("Relative Roll ", C_REL_R)

        for g in (self.g_pay_p, self.g_pay_r, self.g_pay_y,
                  self.g_rel_p, self.g_rel_r):
            gauge_layout.addWidget(g)

        root.addWidget(gauge_box)

        # ---- Plots ----
        plots_row = QHBoxLayout()

        left = QVBoxLayout()
        right = QVBoxLayout()

        self.pw_abs  = make_plot("Absolute Angles — Payload Camera")
        self.pw_frame = make_plot("Frame Drone Angles")
        self.pw_rel  = make_plot("Relative Motor Angles (thay Encoder)")

        left.addWidget(self.pw_abs)
        left.addWidget(self.pw_frame)
        right.addWidget(self.pw_rel)

        plots_row.addLayout(left, 3)
        plots_row.addLayout(right, 2)
        root.addLayout(plots_row)

        # Curves — Absolute payload
        pw = 1.8
        self.c_pay_p = self.pw_abs.plot(pen=pg.mkPen(C_PAY_P, width=pw), name="Payload Pitch")
        self.c_pay_r = self.pw_abs.plot(pen=pg.mkPen(C_PAY_R, width=pw), name="Payload Roll")
        self.c_pay_y = self.pw_abs.plot(pen=pg.mkPen(C_PAY_Y, width=pw), name="Payload Yaw")

        # Curves — Frame
        self.c_fr_p = self.pw_frame.plot(pen=pg.mkPen(C_FRAME_P, width=pw), name="Frame Pitch")
        self.c_fr_r = self.pw_frame.plot(pen=pg.mkPen(C_FRAME_R, width=pw), name="Frame Roll")
        self.c_fr_y = self.pw_frame.plot(pen=pg.mkPen(C_FRAME_Y, width=pw), name="Frame Yaw")

        # Curves — Relative
        self.c_rel_p = self.pw_rel.plot(pen=pg.mkPen(C_REL_P, width=2.5), name="Rel Pitch (motor)")
        self.c_rel_r = self.pw_rel.plot(pen=pg.mkPen(C_REL_R, width=2.5), name="Rel Roll (motor)")

        # Setpoint lines (0°)
        for pw_widget in (self.pw_abs, self.pw_frame, self.pw_rel):
            pw_widget.addLine(y=0, pen=pg.mkPen('#FFFFFF', width=1, style=Qt.PenStyle.DashLine))

    def refresh(self):
        t = self.store.get_time()
        if len(t) < 2:
            return

        pp = self.store.get(self.store.pay_pitch)
        pr = self.store.get(self.store.pay_roll)
        py = self.store.get(self.store.pay_yaw)
        fp = self.store.get(self.store.frame_pitch)
        fr = self.store.get(self.store.frame_roll)
        fy = self.store.get(self.store.frame_yaw)
        rp = self.store.get(self.store.rel_pitch)
        rr = self.store.get(self.store.rel_roll)

        def _plot(curve, t_arr, d_arr):
            n = min(len(t_arr), len(d_arr))
            if n > 1:
                curve.setData(t_arr[-n:], d_arr[-n:])

        _plot(self.c_pay_p, t, pp)
        _plot(self.c_pay_r, t, pr)
        _plot(self.c_pay_y, t, py)
        _plot(self.c_fr_p,  t, fp)
        _plot(self.c_fr_r,  t, fr)
        _plot(self.c_fr_y,  t, fy)
        _plot(self.c_rel_p, t, rp)
        _plot(self.c_rel_r, t, rr)

        # Update gauges (giá trị mới nhất)
        if len(pp): self.g_pay_p.set_value(pp[-1])
        if len(pr): self.g_pay_r.set_value(pr[-1])
        if len(py): self.g_pay_y.set_value(py[-1])
        if len(rp): self.g_rel_p.set_value(rp[-1])
        if len(rr): self.g_rel_r.set_value(rr[-1])
