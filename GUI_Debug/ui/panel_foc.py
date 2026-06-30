"""
panel_foc.py — Tab hiển thị FOC internal state

Hiển thị:
  - Góc điện (angle_elec) cho Pitch và Roll [rad → deg]
  - Điện áp Vq thực tế cho cả 2 trục
  - Homing offset (angle_offset) — giá trị ổn định sau khi Homing xong
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                              QLabel, QFrame, QGridLayout)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from ui.panel_pid_pitch import NumericCard


LABEL_STYLE = {'color': '#AAAAAA', 'font-size': '9pt'}

C_ELEC_P  = '#4FC3F7'   # xanh dương - elec angle pitch
C_ELEC_R  = '#FF6B6B'   # đỏ         - elec angle roll
C_VQ_P    = '#FFCC02'   # vàng       - Vq pitch
C_VQ_R    = '#A5D6A7'   # xanh lá    - Vq roll
C_OFF_P   = '#CE93D8'   # tím        - offset pitch
C_OFF_R   = '#FFAB91'   # cam        - offset roll


def make_plot(title: str, y_label: str) -> pg.PlotWidget:
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


class HomingCard(QFrame):
    """Hiển thị trạng thái Homing (angle_offset cố định)."""

    def __init__(self, axis: str, color: str, parent=None):
        super().__init__(parent)
        self._axis = axis
        self.setStyleSheet(f"""
            QFrame {{
                background: #1A1A2E;
                border: 2px solid {color};
                border-radius: 8px;
            }}
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)

        hdr = QLabel(f"⚙️  Homing Offset — {axis}")
        hdr.setFont(QFont("Inter", 9, QFont.Weight.Bold))
        hdr.setStyleSheet(f"color: {color}; border: none;")

        self.val = QLabel("---")
        self.val.setFont(QFont("Monospace", 16, QFont.Weight.Bold))
        self.val.setStyleSheet("color: #FFFFFF; border: none;")
        self.val.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.unit = QLabel("rad")
        self.unit.setStyleSheet("color: #888888; border: none; font-size: 9pt;")
        self.unit.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout.addWidget(hdr)
        layout.addWidget(self.val)
        layout.addWidget(self.unit)

    def update_value(self, v: float):
        self.val.setText(f"{v:.4f}")


class FOCPanel(QWidget):
    """Tab 5: FOC Internal State."""

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Title
        title = QLabel("⚡  FOC Internal State")
        title.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0E0E0;")
        root.addWidget(title)

        # ---- Homing cards ----
        homing_row = QHBoxLayout()
        self.homing_p = HomingCard("PITCH (TIM2)", C_OFF_P)
        self.homing_r = HomingCard("ROLL  (TIM3)", C_OFF_R)
        homing_row.addWidget(self.homing_p)
        homing_row.addWidget(self.homing_r)

        explain = QLabel(
            "angle_offset: Góc điện bù tại thời điểm Homing. "
            "Giá trị này ổn định sau khi FOC_CalibrateAngle() chạy xong."
        )
        explain.setStyleSheet("color: #888888; font-size: 9pt; font-style: italic;")
        explain.setWordWrap(True)

        homing_box = QFrame()
        homing_box.setStyleSheet("background:#12122A; border-radius:8px;")
        hbl = QVBoxLayout(homing_box)
        hbl.addLayout(homing_row)
        hbl.addWidget(explain)
        root.addWidget(homing_box)

        # ---- Numeric cards ----
        cards_row = QHBoxLayout()
        self.card_ep = NumericCard("angle_elec Pitch", "rad", C_ELEC_P)
        self.card_er = NumericCard("angle_elec Roll",  "rad", C_ELEC_R)
        self.card_vp = NumericCard("Vq Pitch",         "V",   C_VQ_P)
        self.card_vr = NumericCard("Vq Roll",          "V",   C_VQ_R)
        for c in (self.card_ep, self.card_er, self.card_vp, self.card_vr):
            cards_row.addWidget(c)
        root.addLayout(cards_row)

        # ---- Plots ----
        plots_row = QHBoxLayout()
        left = QVBoxLayout()
        right = QVBoxLayout()

        self.pw_elec = make_plot(
            "Electrical Angle  (angle_elec = rel × pole_pairs − offset)",
            "rad"
        )
        self.pw_vq = make_plot(
            "Vq Output — Điện áp thực tế đặt lên motor",
            "V"
        )

        left.addWidget(self.pw_elec)
        right.addWidget(self.pw_vq)

        plots_row.addLayout(left)
        plots_row.addLayout(right)
        root.addLayout(plots_row)

        # Voltage limit lines
        for pw in (self.pw_vq,):
            pw.addLine(y= 1.0, pen=pg.mkPen('#FF3333', width=1, style=Qt.PenStyle.DashLine))
            pw.addLine(y=-1.0, pen=pg.mkPen('#FF3333', width=1, style=Qt.PenStyle.DashLine))
            pw.addLine(y=0, pen=pg.mkPen('#555566', width=1, style=Qt.PenStyle.DashLine))

        # Curves
        lw = 2.0
        self.c_ep = self.pw_elec.plot(pen=pg.mkPen(C_ELEC_P, width=lw), name="angle_elec Pitch")
        self.c_er = self.pw_elec.plot(pen=pg.mkPen(C_ELEC_R, width=lw), name="angle_elec Roll")
        self.c_vp = self.pw_vq.plot(pen=pg.mkPen(C_VQ_P,   width=2.2), name="Vq Pitch")
        self.c_vr = self.pw_vq.plot(pen=pg.mkPen(C_VQ_R,   width=2.2), name="Vq Roll")

    def refresh(self):
        t = self.store.get_time()
        if len(t) < 2:
            return

        ep = self.store.get(self.store.elec_p)
        er = self.store.get(self.store.elec_r)
        op = self.store.get(self.store.off_p)
        or_ = self.store.get(self.store.off_r)
        vp = self.store.get(self.store.vq_pitch)
        vr = self.store.get(self.store.vq_roll)

        def _plot(curve, t_arr, d_arr):
            n = min(len(t_arr), len(d_arr))
            if n > 1:
                curve.setData(t_arr[-n:], d_arr[-n:])

        _plot(self.c_ep, t, ep)
        _plot(self.c_er, t, er)
        _plot(self.c_vp, t, vp)
        _plot(self.c_vr, t, vr)

        if len(ep):  self.card_ep.update_value(ep[-1])
        if len(er):  self.card_er.update_value(er[-1])
        if len(vp):  self.card_vp.update_value(vp[-1])
        if len(vr):  self.card_vr.update_value(vr[-1])
        if len(op):  self.homing_p.update_value(op[-1])
        if len(or_): self.homing_r.update_value(or_[-1])
