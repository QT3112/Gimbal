"""
panel_imu.py — Tab hiển thị dữ liệu raw từ 2 IMU MPU6050

Layout: 2 cột × 2 hàng (accel / gyro cho frame vs payload)
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout,
                              QGroupBox, QLabel, QCheckBox, QSizePolicy)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


# Bảng màu cho 3 trục X/Y/Z
COLORS = {
    'x': '#FF6B6B',   # đỏ san hô
    'y': '#4ECDC4',   # xanh ngọc
    'z': '#FFE66D',   # vàng
}

LABEL_STYLE = {'color': '#AAAAAA', 'font-size': '9pt'}


def make_plot(title: str, y_label: str, y_range=None) -> pg.PlotWidget:
    pw = pg.PlotWidget(title=title)
    pw.setBackground('#1A1A2E')
    pw.showGrid(x=True, y=True, alpha=0.25)
    pw.setLabel('left',   y_label, **LABEL_STYLE)
    pw.setLabel('bottom', 'Time (s)', **LABEL_STYLE)
    pw.getAxis('left').setTextPen('#CCCCCC')
    pw.getAxis('bottom').setTextPen('#CCCCCC')
    pw.setTitle(title, color='#E0E0E0', size='10pt')
    if y_range:
        pw.setYRange(*y_range)
    pw.addLegend(offset=(5, 5))
    return pw


class IMUPanel(QWidget):
    """Tab 1: Raw IMU sensor data cho 2 MPU6050."""

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("📡  Raw IMU Sensor Data")
        title.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0E0E0;")
        hdr.addWidget(title)
        hdr.addStretch()

        self.chk_frame   = QCheckBox("IMU Frame (0x68)")
        self.chk_payload = QCheckBox("IMU Payload (0x69)")
        self.chk_frame.setChecked(True)
        self.chk_payload.setChecked(True)
        self.chk_frame.setStyleSheet("color:#4ECDC4;")
        self.chk_payload.setStyleSheet("color:#FF6B6B;")
        hdr.addWidget(self.chk_frame)
        hdr.addWidget(self.chk_payload)
        root.addLayout(hdr)

        # 4 plots: accel_frame | accel_payload / gyro_frame | gyro_payload
        grid = QHBoxLayout()
        left_col  = QVBoxLayout()
        right_col = QVBoxLayout()

        self.pw_af = make_plot("Accelerometer — Frame",  "m/s²", (-20, 20))
        self.pw_gf = make_plot("Gyroscope — Frame",      "°/s",  (-300, 300))
        self.pw_ap = make_plot("Accelerometer — Payload","m/s²", (-20, 20))
        self.pw_gp = make_plot("Gyroscope — Payload",   "°/s",  (-300, 300))

        left_col.addWidget(self.pw_af)
        left_col.addWidget(self.pw_gf)
        right_col.addWidget(self.pw_ap)
        right_col.addWidget(self.pw_gp)

        grid.addLayout(left_col)
        grid.addLayout(right_col)
        root.addLayout(grid)

        # Tạo curves
        pen_w = 1.5
        self.c_af = {ax: self.pw_af.plot(pen=pg.mkPen(COLORS[ax], width=pen_w), name=f"A{ax.upper()}") for ax in 'xyz'}
        self.c_gf = {ax: self.pw_gf.plot(pen=pg.mkPen(COLORS[ax], width=pen_w), name=f"G{ax.upper()}") for ax in 'xyz'}
        self.c_ap = {ax: self.pw_ap.plot(pen=pg.mkPen(COLORS[ax], width=pen_w), name=f"A{ax.upper()}") for ax in 'xyz'}
        self.c_gp = {ax: self.pw_gp.plot(pen=pg.mkPen(COLORS[ax], width=pen_w), name=f"G{ax.upper()}") for ax in 'xyz'}

    def refresh(self):
        t  = self.store.get_time()
        if len(t) < 2:
            return

        show_f = self.chk_frame.isChecked()
        show_p = self.chk_payload.isChecked()

        accel_f = [self.store.get(q) for q in (self.store.imu_f_ax, self.store.imu_f_ay, self.store.imu_f_az)]
        gyro_f  = [self.store.get(q) for q in (self.store.imu_f_gx, self.store.imu_f_gy, self.store.imu_f_gz)]
        accel_p = [self.store.get(q) for q in (self.store.imu_p_ax, self.store.imu_p_ay, self.store.imu_p_az)]
        gyro_p  = [self.store.get(q) for q in (self.store.imu_p_gx, self.store.imu_p_gy, self.store.imu_p_gz)]

        for ax, d in zip('xyz', accel_f):
            n = min(len(t), len(d))
            self.c_af[ax].setData(t[-n:], d[-n:] if show_f else [])
        for ax, d in zip('xyz', gyro_f):
            n = min(len(t), len(d))
            self.c_gf[ax].setData(t[-n:], d[-n:] if show_f else [])
        for ax, d in zip('xyz', accel_p):
            n = min(len(t), len(d))
            self.c_ap[ax].setData(t[-n:], d[-n:] if show_p else [])
        for ax, d in zip('xyz', gyro_p):
            n = min(len(t), len(d))
            self.c_gp[ax].setData(t[-n:], d[-n:] if show_p else [])
