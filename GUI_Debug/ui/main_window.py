"""
main_window.py — Cửa sổ chính của Gimbal Debug Monitor

Layout:
  ┌─ Toolbar: Port selector, Connect, Demo, Record ─────────────┐
  │  Status bar: latency, frame rate, bytes received            │
  ├─ Tab widget ─────────────────────────────────────────────────┤
  │  [IMU Raw] [Attitude] [PID Pitch] [PID Roll] [FOC]          │
  └──────────────────────────────────────────────────────────────┘
"""

import time
import csv
import os
from datetime import datetime

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QComboBox, QPushButton, QLabel,
    QStatusBar, QFrame, QFileDialog, QMessageBox,
    QSizePolicy, QSpinBox
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QIcon, QPalette, QColor

import pyqtgraph as pg

from data_store import GimbalDataStore
from serial_reader import SerialReaderThread, DemoThread, list_serial_ports
from ui.panel_imu       import IMUPanel
from ui.panel_attitude  import AttitudePanel
from ui.panel_pid_pitch import PIDPitchPanel
from ui.panel_pid_roll  import PIDRollPanel
from ui.panel_foc       import FOCPanel


# Áp dụng theme tối toàn cục
pg.setConfigOption('background', '#1A1A2E')
pg.setConfigOption('foreground', '#CCCCCC')

DARK_STYLESHEET = """
QMainWindow, QWidget {
    background-color: #0F0F1E;
    color: #E0E0E0;
    font-family: 'Inter', 'Segoe UI', sans-serif;
    font-size: 10pt;
}
QTabWidget::pane {
    border: 1px solid #2A2A4A;
    background: #0F0F1E;
}
QTabBar::tab {
    background: #1A1A2E;
    color: #AAAAAA;
    padding: 8px 16px;
    border: 1px solid #2A2A4A;
    border-bottom: none;
    border-radius: 4px 4px 0 0;
    min-width: 100px;
}
QTabBar::tab:selected {
    background: #2A2A4A;
    color: #FFFFFF;
    border-color: #4ECDC4;
}
QTabBar::tab:hover {
    background: #252540;
    color: #DDDDDD;
}
QPushButton {
    background: #2A2A4A;
    color: #E0E0E0;
    border: 1px solid #3A3A5A;
    border-radius: 5px;
    padding: 5px 14px;
    font-weight: bold;
}
QPushButton:hover  { background: #3A3A6A; border-color: #4ECDC4; }
QPushButton:pressed{ background: #4A4A7A; }
QPushButton:disabled { color: #555555; border-color: #2A2A4A; }
QPushButton#btn_connect_on {
    background: #1B4332;
    border-color: #4ECDC4;
    color: #4ECDC4;
}
QPushButton#btn_record_on {
    background: #5C1010;
    border-color: #FF6B6B;
    color: #FF6B6B;
}
QComboBox {
    background: #1A1A2E;
    border: 1px solid #3A3A5A;
    border-radius: 4px;
    padding: 4px 8px;
    color: #E0E0E0;
    min-width: 100px;
}
QComboBox::drop-down { border: none; }
QComboBox:hover { border-color: #4ECDC4; }
QLabel#status_ok   { color: #4ECDC4; }
QLabel#status_err  { color: #FF6B6B; }
QLabel#status_idle { color: #888888; }
QSpinBox {
    background: #1A1A2E;
    border: 1px solid #3A3A5A;
    border-radius: 4px;
    color: #E0E0E0;
    padding: 2px 6px;
}
"""


class MainWindow(QMainWindow):
    """Cửa sổ chính của Gimbal Debug Monitor."""

    UPDATE_INTERVAL_MS = 40   # ~25fps UI refresh

    def __init__(self):
        super().__init__()
        self.store  = GimbalDataStore(size=2000)
        self.reader = None
        self._recording = False
        self._csv_writer = None
        self._csv_file   = None
        self._frame_count = 0
        self._last_fps_time = time.monotonic()
        self._fps = 0.0

        self._build_ui()
        self._apply_theme()
        self._setup_timer()

    # ---------------------------------------------------------------
    def _build_ui(self):
        self.setWindowTitle("🎯 Gimbal Debug Monitor — STM32G431 + 2×MPU6050")
        self.resize(1400, 900)
        self.setMinimumSize(900, 600)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 4)
        root.setSpacing(4)

        # ---- Toolbar ----
        root.addWidget(self._build_toolbar())

        # ---- Status row ----
        root.addWidget(self._build_status_row())

        # ---- Tab widget ----
        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)

        self.panel_imu   = IMUPanel(self.store)
        self.panel_att   = AttitudePanel(self.store)
        self.panel_pid_p = PIDPitchPanel(self.store)
        self.panel_pid_r = PIDRollPanel(self.store)
        self.panel_foc   = FOCPanel(self.store)

        self.tabs.addTab(self.panel_imu,   "📡  IMU Raw")
        self.tabs.addTab(self.panel_att,   "🧭  Attitude")
        self.tabs.addTab(self.panel_pid_p, "🎯  PID Pitch")
        self.tabs.addTab(self.panel_pid_r, "🔄  PID Roll")
        self.tabs.addTab(self.panel_foc,   "⚡  FOC")

        root.addWidget(self.tabs)

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("background:#12122A; border-radius:8px;")
        bar.setFixedHeight(54)
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(10, 6, 10, 6)
        layout.setSpacing(10)

        # Logo / Title
        logo = QLabel("⚙️  <b>Gimbal Debug</b>")
        logo.setFont(QFont("Inter", 11, QFont.Weight.Bold))
        logo.setStyleSheet("color:#4ECDC4;")
        layout.addWidget(logo)

        layout.addSpacing(20)

        # Port selector
        port_lbl = QLabel("Port:")
        port_lbl.setStyleSheet("color:#AAAAAA;")
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(140)
        self.combo_port.setToolTip("Chọn cổng serial USB CDC")
        layout.addWidget(port_lbl)
        layout.addWidget(self.combo_port)

        # Refresh port list
        btn_refresh = QPushButton("🔄")
        btn_refresh.setFixedWidth(36)
        btn_refresh.setToolTip("Làm mới danh sách cổng")
        btn_refresh.clicked.connect(self._refresh_ports)
        layout.addWidget(btn_refresh)

        # Baudrate
        baud_lbl = QLabel("Baud:")
        baud_lbl.setStyleSheet("color:#AAAAAA;")
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "230400", "460800", "921600"])
        self.combo_baud.setCurrentText("115200")
        self.combo_baud.setFixedWidth(90)
        layout.addWidget(baud_lbl)
        layout.addWidget(self.combo_baud)

        layout.addSpacing(10)

        # Connect button
        self.btn_connect = QPushButton("🔗  Connect")
        self.btn_connect.setFixedWidth(120)
        self.btn_connect.setToolTip("Kết nối với STM32 qua USB CDC")
        self.btn_connect.clicked.connect(self._toggle_connect)
        layout.addWidget(self.btn_connect)

        # Demo button
        self.btn_demo = QPushButton("🎮  Demo")
        self.btn_demo.setFixedWidth(90)
        self.btn_demo.setToolTip("Chạy với dữ liệu giả lập (không cần phần cứng)")
        self.btn_demo.clicked.connect(self._toggle_demo)
        layout.addWidget(self.btn_demo)

        layout.addStretch()

        # Buffer size
        buf_lbl = QLabel("Buffer:")
        buf_lbl.setStyleSheet("color:#AAAAAA;")
        self.spin_buf = QSpinBox()
        self.spin_buf.setRange(100, 5000)
        self.spin_buf.setValue(1000)
        self.spin_buf.setSuffix(" pts")
        self.spin_buf.setFixedWidth(100)
        self.spin_buf.setToolTip("Số điểm dữ liệu hiển thị (100–5000)")
        layout.addWidget(buf_lbl)
        layout.addWidget(self.spin_buf)

        layout.addSpacing(10)

        # Record button
        self.btn_record = QPushButton("⏺  Record")
        self.btn_record.setFixedWidth(110)
        self.btn_record.setToolTip("Ghi dữ liệu ra file CSV")
        self.btn_record.clicked.connect(self._toggle_record)
        layout.addWidget(self.btn_record)

        self._refresh_ports()
        return bar

    def _build_status_row(self) -> QFrame:
        row = QFrame()
        row.setStyleSheet("background:#0A0A1A; border-radius:4px;")
        row.setFixedHeight(28)
        layout = QHBoxLayout(row)
        layout.setContentsMargins(12, 2, 12, 2)

        self.lbl_status = QLabel("● Chưa kết nối")
        self.lbl_status.setObjectName("status_idle")
        self.lbl_fps    = QLabel("FPS: ---")
        self.lbl_bytes  = QLabel("Bytes: 0")
        self.lbl_record = QLabel("")

        for lbl in (self.lbl_status, self.lbl_fps, self.lbl_bytes, self.lbl_record):
            lbl.setFont(QFont("Monospace", 8))
            lbl.setStyleSheet("color:#888888;")
            layout.addWidget(lbl)
            if lbl != self.lbl_record:
                sep = QLabel("│")
                sep.setStyleSheet("color:#333344;")
                layout.addWidget(sep)

        layout.addStretch()
        return row

    # ---------------------------------------------------------------
    def _apply_theme(self):
        self.setStyleSheet(DARK_STYLESHEET)

    def _setup_timer(self):
        self._timer = QTimer(self)
        self._timer.setInterval(self.UPDATE_INTERVAL_MS)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start()

    # ---------------------------------------------------------------
    def _refresh_ports(self):
        ports = list_serial_ports()
        current = self.combo_port.currentText()
        self.combo_port.clear()
        if ports:
            self.combo_port.addItems(ports)
            if current in ports:
                self.combo_port.setCurrentText(current)
        else:
            self.combo_port.addItem("(không tìm thấy)")

    def _stop_reader(self):
        if self.reader:
            self.reader.stop()
            self.reader = None

    def _toggle_connect(self):
        if self.reader and self.reader.isRunning():
            self._stop_reader()
            self.btn_connect.setText("🔗  Connect")
            self.btn_connect.setObjectName("")
            self.btn_connect.setStyleSheet("")
            self.btn_demo.setEnabled(True)
            self._set_status("Đã ngắt kết nối", "idle")
        else:
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            if not port or '(' in port:
                QMessageBox.warning(self, "Lỗi", "Vui lòng chọn cổng serial hợp lệ.")
                return
            self.reader = SerialReaderThread(self.store)
            self.reader.configure(port, baud)
            self.reader.connected.connect(self._on_connected)
            self.reader.disconnected.connect(self._on_disconnected)
            self.reader.error_msg.connect(self._on_error)
            self.reader.new_frame.connect(self._on_new_frame)
            self.reader.start()
            self.btn_demo.setEnabled(False)

    def _toggle_demo(self):
        if self.reader and self.reader.isRunning():
            self._stop_reader()
            self.btn_demo.setText("🎮  Demo")
            self.btn_demo.setObjectName("")
            self.btn_demo.setStyleSheet("")
            self.btn_connect.setEnabled(True)
            self._set_status("Demo dừng", "idle")
        else:
            self.reader = DemoThread(self.store)
            self.reader.connected.connect(self._on_connected)
            self.reader.disconnected.connect(self._on_disconnected)
            self.reader.error_msg.connect(self._on_error)
            self.reader.new_frame.connect(self._on_new_frame)
            self.reader.start()
            self.btn_connect.setEnabled(False)
            self.btn_demo.setText("⏹  Stop Demo")
            self.btn_demo.setObjectName("btn_connect_on")
            self.btn_demo.setStyleSheet("""
                QPushButton#btn_connect_on {
                    background: #1B3A2E; border-color: #4ECDC4; color: #4ECDC4;
                }
            """)

    def _toggle_record(self):
        if not self._recording:
            path, _ = QFileDialog.getSaveFileName(
                self, "Lưu file CSV",
                f"gimbal_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                "CSV Files (*.csv)"
            )
            if not path:
                return
            self._csv_file   = open(path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            # Header
            self._csv_writer.writerow([
                'time_s',
                'imu_f_ax','imu_f_ay','imu_f_az','imu_f_gx','imu_f_gy','imu_f_gz',
                'imu_p_ax','imu_p_ay','imu_p_az','imu_p_gx','imu_p_gy','imu_p_gz',
                'frame_pitch','frame_roll','frame_yaw',
                'pay_pitch','pay_roll','pay_yaw',
                'rel_pitch','rel_roll',
                'p_err','t_vel_p','ff_pitch','cam_pr','vel_err_p','vq_pitch',
                'r_err','t_vel_r','ff_roll','cam_rr','vel_err_r','vq_roll',
                'elec_p','elec_r','off_p','off_r'
            ])
            self._recording = True
            self.btn_record.setText("⏹  Stop Rec")
            self.btn_record.setObjectName("btn_record_on")
            self.btn_record.setStyleSheet("""
                QPushButton#btn_record_on {
                    background: #4A0A0A; border-color: #FF6B6B; color: #FF6B6B;
                }
            """)
            self.lbl_record.setText(f"⏺ REC → {os.path.basename(path)}")
            self.lbl_record.setStyleSheet("color:#FF6B6B;")
        else:
            self._recording = False
            if self._csv_file:
                self._csv_file.close()
                self._csv_file = None
            self._csv_writer = None
            self.btn_record.setText("⏺  Record")
            self.btn_record.setObjectName("")
            self.btn_record.setStyleSheet("")
            self.lbl_record.setText("")

    # ---------------------------------------------------------------
    def _on_connected(self, port: str):
        self._set_status(f"● Đã kết nối: {port}", "ok")
        self.btn_connect.setText("⏹  Disconnect")
        self.btn_connect.setObjectName("btn_connect_on")
        self.btn_connect.setStyleSheet("""
            QPushButton#btn_connect_on {
                background: #1B4332; border-color: #4ECDC4; color: #4ECDC4;
            }
        """)

    def _on_disconnected(self):
        self._set_status("● Đã ngắt kết nối", "idle")
        self.btn_connect.setText("🔗  Connect")
        self.btn_connect.setObjectName("")
        self.btn_connect.setStyleSheet("")
        self.btn_demo.setEnabled(True)
        self.btn_connect.setEnabled(True)

    def _on_error(self, msg: str):
        self._set_status(f"✗ {msg}", "err")

    def _on_new_frame(self):
        self._frame_count += 1
        # Ghi CSV nếu đang record
        if self._recording and self._csv_writer:
            t  = self.store.get_time()
            if len(t) == 0:
                return
            ts = t[-1]

            def _last(q): return self.store.get(q)[-1] if len(q) else 0.0
            s = self.store
            self._csv_writer.writerow([
                f"{ts:.4f}",
                _last(s.imu_f_ax), _last(s.imu_f_ay), _last(s.imu_f_az),
                _last(s.imu_f_gx), _last(s.imu_f_gy), _last(s.imu_f_gz),
                _last(s.imu_p_ax), _last(s.imu_p_ay), _last(s.imu_p_az),
                _last(s.imu_p_gx), _last(s.imu_p_gy), _last(s.imu_p_gz),
                _last(s.frame_pitch), _last(s.frame_roll), _last(s.frame_yaw),
                _last(s.pay_pitch),   _last(s.pay_roll),   _last(s.pay_yaw),
                _last(s.rel_pitch),   _last(s.rel_roll),
                _last(s.p_err), _last(s.t_vel_p), _last(s.ff_pitch),
                _last(s.cam_pr), _last(s.vel_err_p), _last(s.vq_pitch),
                _last(s.r_err), _last(s.t_vel_r), _last(s.ff_roll),
                _last(s.cam_rr), _last(s.vel_err_r), _last(s.vq_roll),
                _last(s.elec_p), _last(s.elec_r), _last(s.off_p), _last(s.off_r),
            ])

    def _on_timer(self):
        """Cập nhật tất cả panel theo UI timer."""
        current_tab = self.tabs.currentIndex()
        panels = [
            self.panel_imu, self.panel_att,
            self.panel_pid_p, self.panel_pid_r, self.panel_foc
        ]
        # Chỉ refresh tab hiện tại để tiết kiệm CPU
        panels[current_tab].refresh()

        # FPS counter
        now = time.monotonic()
        dt = now - self._last_fps_time
        if dt >= 1.0:
            self._fps = self._frame_count / dt
            self._frame_count = 0
            self._last_fps_time = now
            self.lbl_fps.setText(f"FPS: {self._fps:.1f}")

    def _set_status(self, msg: str, level: str = "idle"):
        colors = {'ok': '#4ECDC4', 'err': '#FF6B6B', 'idle': '#888888'}
        self.lbl_status.setText(msg)
        self.lbl_status.setStyleSheet(f"color: {colors.get(level, '#888888')};")

    # ---------------------------------------------------------------
    def closeEvent(self, event):
        self._stop_reader()
        if self._csv_file:
            self._csv_file.close()
        super().closeEvent(event)
