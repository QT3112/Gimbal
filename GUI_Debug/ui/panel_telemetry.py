"""
panel_telemetry.py — Tab đồ thị real-time FOC telemetry

Tự động chuyển layout dựa trên chế độ đang nhận ($VEL vs $POS).
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                              QLabel, QFrame, QSplitter)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

LABEL_STYLE = {'color': '#AAAAAA', 'font-size': '9pt'}

# Palette màu
C_TARGET = '#4ECDC4'   # target — ngọc lam
C_FILT   = '#81C784'   # vel_filt — xanh lá
C_RAW    = '#546E7A'   # vel_raw — xám xanh (mờ hơn)
C_VQ     = '#FFCC02'   # Vq — vàng
C_ENC    = '#FF6B6B'   # encoder — đỏ cam
C_ERR    = '#FF8A65'   # error — cam


def _make_plot(title: str, y_label: str) -> pg.PlotWidget:
    pw = pg.PlotWidget()
    pw.setBackground('#12122A')
    pw.showGrid(x=True, y=True, alpha=0.2)
    pw.setLabel('left',   y_label, **LABEL_STYLE)
    pw.setLabel('bottom', 'Time (s)', **LABEL_STYLE)
    pw.getAxis('left').setTextPen('#CCCCCC')
    pw.getAxis('bottom').setTextPen('#CCCCCC')
    pw.setTitle(title, color='#E0E0E0', size='9pt')
    pw.addLegend(offset=(5, 5), labelTextColor='#CCCCCC')
    pw.addLine(y=0, pen=pg.mkPen('#334455', width=1, style=Qt.PenStyle.DashLine))
    return pw


class NumericCard(QFrame):
    def __init__(self, label: str, unit: str, color: str, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame {{
                background: #1A1A2E;
                border: 1px solid {color};
                border-radius: 8px;
            }}
        """)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(10, 6, 10, 6)

        lbl = QLabel(label)
        lbl.setFont(QFont("Inter", 8))
        lbl.setStyleSheet(f"color: {color}; border: none;")

        self.val = QLabel("---")
        self.val.setFont(QFont("Monospace", 15, QFont.Weight.Bold))
        self.val.setStyleSheet("color: #FFFFFF; border: none;")
        self.val.setAlignment(Qt.AlignmentFlag.AlignRight)

        unit_lbl = QLabel(unit)
        unit_lbl.setFont(QFont("Inter", 7))
        unit_lbl.setStyleSheet("color: #666688; border: none;")
        unit_lbl.setAlignment(Qt.AlignmentFlag.AlignRight)

        lay.addWidget(lbl)
        lay.addWidget(self.val)
        lay.addWidget(unit_lbl)

    def update(self, v: float, fmt: str = "{:+.3f}"):
        self.val.setText(fmt.format(v))


class TelemetryPanel(QWidget):
    """Tab 1: FOC Telemetry — auto-switch VEL/POS layout."""

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._last_mode = None
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Header
        hdr = QHBoxLayout()
        self.title_lbl = QLabel("📊  FOC Telemetry")
        self.title_lbl.setFont(QFont("Inter", 13, QFont.Weight.Bold))
        self.title_lbl.setStyleSheet("color: #4ECDC4;")
        self.mode_lbl = QLabel("Mode: ---")
        self.mode_lbl.setFont(QFont("Inter", 10, QFont.Weight.Bold))
        self.mode_lbl.setStyleSheet("color: #888888; background: #1A1A2E; border-radius:4px; padding: 3px 10px;")
        hdr.addWidget(self.title_lbl)
        hdr.addStretch()
        hdr.addWidget(self.mode_lbl)
        root.addLayout(hdr)

        # Numeric cards row
        cards_row = QHBoxLayout()
        self.card_target = NumericCard("Target",    "—",    C_TARGET)
        self.card_filt   = NumericCard("Vel Filt",  "rad/s",C_FILT)
        self.card_vq     = NumericCard("Vq",        "V",    C_VQ)
        self.card_enc    = NumericCard("Encoder",   "deg",  C_ENC)
        self.card_err    = NumericCard("Error",     "—",    C_ERR)
        for c in (self.card_target, self.card_filt, self.card_vq,
                  self.card_enc, self.card_err):
            cards_row.addWidget(c)
        root.addLayout(cards_row)

        # Plot area — Velocity mode
        self._vel_widget = QWidget()
        vw = QVBoxLayout(self._vel_widget)
        vw.setSpacing(4)
        self.pw_vel  = _make_plot("Velocity Response  (target vs filtered vs raw)", "rad/s")
        self.pw_vq_v = _make_plot("Vq Output Voltage", "V")
        self.pw_enc_v= _make_plot("Encoder Angle", "deg")
        self.c_v_tgt = self.pw_vel.plot(pen=pg.mkPen(C_TARGET, width=2.5), name="Target")
        self.c_v_flt = self.pw_vel.plot(pen=pg.mkPen(C_FILT,   width=2.0), name="VelFilt")
        self.c_v_raw = self.pw_vel.plot(pen=pg.mkPen(C_RAW,    width=1.2), name="VelRaw")
        self.c_v_vq  = self.pw_vq_v.plot(pen=pg.mkPen(C_VQ,   width=2.0), name="Vq (V)")
        self.c_v_enc = self.pw_enc_v.plot(pen=pg.mkPen(C_ENC,  width=1.5), name="Enc (deg)")
        # Voltage limit markers
        for lim in (1.5, -1.5):
            self.pw_vq_v.addLine(y=lim, pen=pg.mkPen('#FF3333', width=1,
                                                      style=Qt.PenStyle.DashLine))
        vw.addWidget(self.pw_vel,   3)
        vw.addWidget(self.pw_vq_v,  2)
        vw.addWidget(self.pw_enc_v, 2)

        # Plot area — Position mode
        self._pos_widget = QWidget()
        pw = QVBoxLayout(self._pos_widget)
        pw.setSpacing(4)
        self.pw_pos  = _make_plot("Position Response  (target vs actual)", "deg")
        self.pw_err  = _make_plot("Position Error", "deg")
        self.pw_vq_p = _make_plot("Vq  &  Vel Filtered", "V | rad/s")
        self.c_p_tgt = self.pw_pos.plot(pen=pg.mkPen(C_TARGET, width=2.5), name="Target (deg)")
        self.c_p_enc = self.pw_pos.plot(pen=pg.mkPen(C_ENC,    width=2.0), name="Actual (deg)")
        self.c_p_err = self.pw_err.plot(pen=pg.mkPen(C_ERR,    width=2.0), name="Error (deg)")
        self.c_p_vq  = self.pw_vq_p.plot(pen=pg.mkPen(C_VQ,   width=2.0), name="Vq (V)")
        self.c_p_vf  = self.pw_vq_p.plot(pen=pg.mkPen(C_FILT,  width=1.5), name="VelFilt (rad/s)")
        for lim in (1.5, -1.5):
            self.pw_vq_p.addLine(y=lim, pen=pg.mkPen('#FF3333', width=1,
                                                       style=Qt.PenStyle.DashLine))
        pw.addWidget(self.pw_pos,  3)
        pw.addWidget(self.pw_err,  2)
        pw.addWidget(self.pw_vq_p, 2)

        # Stack cả hai widget — chỉ show 1 cái
        root.addWidget(self._vel_widget, 1)
        root.addWidget(self._pos_widget, 1)
        self._vel_widget.setVisible(False)
        self._pos_widget.setVisible(True)

    def _show_mode(self, mode: str):
        if mode == self._last_mode:
            return
        self._last_mode = mode
        is_vel = (mode == 'VEL')
        self._vel_widget.setVisible(is_vel)
        self._pos_widget.setVisible(not is_vel)
        color = '#4FC3F7' if is_vel else '#81C784'
        self.mode_lbl.setText(f"Mode: {'VELOCITY' if is_vel else 'POSITION'}")
        self.mode_lbl.setStyleSheet(
            f"color: {color}; background: #1A1A2E; border-radius:4px;"
            f" padding: 3px 10px; font-weight:bold;")
        # Cập nhật label card target
        self.card_target.findChildren(QLabel)[0].setText(
            "Target Vel" if is_vel else "Target Pos")

    def _plot(self, curve, t, d):
        n = min(len(t), len(d))
        if n > 1:
            curve.setData(t[-n:], d[-n:])

    def refresh(self):
        t = self.store.get_time()
        if len(t) < 2:
            return

        mode = self.store.get_mode()
        self._show_mode(mode)

        if mode == 'VEL':
            tgt = self.store.get(self.store.vel_target)
            flt = self.store.get(self.store.vel_filt)
            raw = self.store.get(self.store.vel_raw)
            vq  = self.store.get(self.store.vq_vel)
            enc = self.store.get(self.store.enc_vel)
            self._plot(self.c_v_tgt, t, tgt)
            self._plot(self.c_v_flt, t, flt)
            self._plot(self.c_v_raw, t, raw)
            self._plot(self.c_v_vq,  t, vq)
            self._plot(self.c_v_enc, t, enc)
            if len(tgt): self.card_target.update(tgt[-1])
            if len(flt): self.card_filt.update(flt[-1])
            if len(vq):  self.card_vq.update(vq[-1])
            if len(enc): self.card_enc.update(enc[-1], "{:.1f}")
            if len(tgt) and len(flt):
                self.card_err.update(tgt[-1] - flt[-1])
        else:
            ptgt = self.store.get(self.store.pos_target)
            penc = self.store.get(self.store.enc_pos)
            perr = self.store.get(self.store.pos_err)
            pvq  = self.store.get(self.store.vq_pos)
            pvf  = self.store.get(self.store.vel_filt_pos)
            self._plot(self.c_p_tgt, t, ptgt)
            self._plot(self.c_p_enc, t, penc)
            self._plot(self.c_p_err, t, perr)
            self._plot(self.c_p_vq,  t, pvq)
            self._plot(self.c_p_vf,  t, pvf)
            if len(ptgt): self.card_target.update(ptgt[-1], "{:.1f} °")
            if len(pvf):  self.card_filt.update(pvf[-1])
            if len(pvq):  self.card_vq.update(pvq[-1])
            if len(penc): self.card_enc.update(penc[-1], "{:.1f}")
            if len(perr): self.card_err.update(perr[-1], "{:+.1f} °")
