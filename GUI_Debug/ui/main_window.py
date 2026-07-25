"""
main_window.py — Cửa sổ chính FOC Debug Monitor v2.0

Layout:
  ┌── Toolbar: Port, Connect, Demo, Record ───────────────────────────┐
  │   Status bar: connected / FPS / TX / RX                          │
  ├── Tab: [FOC Telemetry] [Control] [PID Tuning] [Console]          │
  │   ── separator ──                                                 │
  │   [IMU Raw] [Attitude] [PID Pitch] [PID Roll] [FOC State]        │
  └───────────────────────────────────────────────────────────────────┘

Tab FOC (mới): Telemetry, Control, PID Tuning, Console
Tab Legacy (cũ): IMU, Attitude, PID Pitch, PID Roll, FOC State
"""

import time
import csv
import os
from datetime import datetime

import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QTabWidget, QComboBox, QPushButton, QLabel,
    QFrame, QFileDialog, QMessageBox, QSpinBox, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QPalette, QColor

from data_store import FOCDataStore
from serial_reader import SerialReaderThread, DemoThread, list_serial_ports
from command_sender import CommandSender

# ── FOC panels
from ui.panel_telemetry  import TelemetryPanel
from ui.panel_control    import ControlPanel
from ui.panel_pid_tuning import PIDTuningPanel
from ui.panel_console    import ConsolePanel


# ───────────────────────── Theme ─────────────────────────────────────────────
pg.setConfigOption('background', '#12122A')
pg.setConfigOption('foreground', '#CCCCCC')

DARK_SS = """
QMainWindow, QWidget {
    background-color: #0D0D1F;
    color: #E0E0E0;
    font-family: 'Inter', 'Segoe UI', sans-serif;
    font-size: 10pt;
}
QTabWidget::pane {
    border: 1px solid #1E1E3E;
    background: #0D0D1F;
}
QTabBar::tab {
    background: #141430;
    color: #888888;
    padding: 7px 16px;
    border: 1px solid #1E1E3E;
    border-bottom: none;
    border-radius: 4px 4px 0 0;
    min-width: 90px;
}
QTabBar::tab:selected { background: #1E1E3E; color: #FFFFFF; border-color: #4ECDC4; }
QTabBar::tab:hover    { background: #181840; color: #CCCCCC; }
QPushButton {
    background: #1E1E3E; color: #E0E0E0;
    border: 1px solid #2E2E5E; border-radius: 5px;
    padding: 5px 14px; font-weight: bold;
}
QPushButton:hover  { background: #2A2A5A; border-color: #4ECDC4; }
QPushButton:pressed{ background: #3A3A6A; }
QPushButton:disabled { color: #444466; border-color: #1E1E3E; }
QPushButton#btn_on  { background: #0E3028; border-color: #4ECDC4; color: #4ECDC4; }
QPushButton#btn_rec { background: #3A0808; border-color: #FF6B6B; color: #FF6B6B; }
QComboBox {
    background: #141430; border: 1px solid #2E2E5E; border-radius: 4px;
    padding: 4px 8px; color: #E0E0E0; min-width: 110px;
}
QComboBox:hover { border-color: #4ECDC4; }
QComboBox::drop-down { border: none; }
QSpinBox {
    background: #141430; border: 1px solid #2E2E5E; border-radius: 4px;
    color: #E0E0E0; padding: 2px 6px;
}
QGroupBox {
    border: 1px solid #2A2A4A; border-radius: 6px; margin-top: 8px; padding-top: 6px;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; color: #4ECDC4; }
"""


# ───────────────────────── Main Window ───────────────────────────────────────
class MainWindow(QMainWindow):
    UPDATE_MS = 50   # 20fps

    def __init__(self):
        super().__init__()
        self.store   = FOCDataStore(size=3000)
        self.sender  = CommandSender()
        self.reader  = None
        self._recording  = False
        self._csv_writer = None
        self._csv_file   = None
        self._frame_cnt  = 0
        self._last_fps_t = time.monotonic()
        self._fps        = 0.0
        self._tx_cnt     = 0

        self._build_ui()
        self.setStyleSheet(DARK_SS)
        self._timer = QTimer(self)
        self._timer.setInterval(self.UPDATE_MS)
        self._timer.timeout.connect(self._on_timer)
        self._timer.start()

    # ──────────────────────────────────────────────────────── build UI
    def _build_ui(self):
        self.setWindowTitle("⚡  FOC Debug Monitor  —  STM32G431 Gimbal  [v2.0]")
        self.resize(1440, 920)
        self.setMinimumSize(1000, 700)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 4)
        root.setSpacing(4)

        root.addWidget(self._build_toolbar())
        root.addWidget(self._build_status_bar())

        # ── FOC Tabs
        self._foc_tabs = QTabWidget()
        self._foc_tabs.setDocumentMode(True)

        self.panel_telem   = TelemetryPanel(self.store)
        self.panel_ctrl    = ControlPanel(self.sender)
        self.panel_pid     = PIDTuningPanel(self.store, self.sender)
        self.panel_console = ConsolePanel(self.store, self.sender)

        self._foc_tabs.addTab(self.panel_telem,   "📊  Telemetry")
        self._foc_tabs.addTab(self.panel_ctrl,    "🎛  Control")
        self._foc_tabs.addTab(self.panel_pid,     "🔧  PID Tuning")
        self._foc_tabs.addTab(self.panel_console, "📋  Console")

        root.addWidget(self._foc_tabs, 1)

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("background: #0A0A20; border-radius: 8px;")
        bar.setFixedHeight(58)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 6, 12, 6)
        lay.setSpacing(10)

        # Logo
        logo = QLabel("⚡  <b>FOC Debug</b>")
        logo.setFont(QFont("Inter", 12, QFont.Weight.Bold))
        logo.setStyleSheet("color: #4ECDC4;")
        lay.addWidget(logo)
        lay.addSpacing(16)

        # Port
        lay.addWidget(_plain("Port:"))
        self.combo_port = QComboBox()
        self.combo_port.setMinimumWidth(150)
        lay.addWidget(self.combo_port)

        btn_ref = QPushButton("🔄")
        btn_ref.setFixedWidth(34)
        btn_ref.setToolTip("Refresh port list")
        btn_ref.clicked.connect(self._refresh_ports)
        lay.addWidget(btn_ref)

        # Baud
        lay.addWidget(_plain("Baud:"))
        self.combo_baud = QComboBox()
        self.combo_baud.addItems(["115200", "230400", "460800", "921600"])
        self.combo_baud.setFixedWidth(90)
        lay.addWidget(self.combo_baud)

        lay.addSpacing(8)

        # Connect
        self.btn_connect = QPushButton("🔗  Connect")
        self.btn_connect.setFixedWidth(130)
        self.btn_connect.clicked.connect(self._toggle_connect)
        lay.addWidget(self.btn_connect)

        # Demo
        self.btn_demo = QPushButton("🎮  Demo")
        self.btn_demo.setFixedWidth(100)
        self.btn_demo.clicked.connect(self._toggle_demo)
        lay.addWidget(self.btn_demo)

        lay.addStretch()

        # Buffer
        lay.addWidget(_plain("Buffer:"))
        self.spin_buf = QSpinBox()
        self.spin_buf.setRange(100, 5000)
        self.spin_buf.setValue(2000)
        self.spin_buf.setSuffix(" pts")
        self.spin_buf.setFixedWidth(100)
        lay.addWidget(self.spin_buf)

        lay.addSpacing(8)

        # Record
        self.btn_rec = QPushButton("⏺  Record")
        self.btn_rec.setFixedWidth(110)
        self.btn_rec.clicked.connect(self._toggle_record)
        lay.addWidget(self.btn_rec)

        self._refresh_ports()
        return bar

    def _build_status_bar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet("background: #080818; border-radius: 4px;")
        bar.setFixedHeight(26)
        lay = QHBoxLayout(bar)
        lay.setContentsMargins(12, 2, 12, 2)
        lay.setSpacing(0)

        self.lbl_status = _status("● Chưa kết nối", '#666688')
        self.lbl_fps    = _status("FPS: ---",        '#666688')
        self.lbl_rx     = _status("RX: ---",         '#666688')
        self.lbl_tx     = _status("TX: 0 cmd",       '#666688')
        self.lbl_rec    = _status("",                 '#FF6B6B')

        for lbl in (self.lbl_status, self.lbl_fps, self.lbl_rx, self.lbl_tx):
            lay.addWidget(lbl)
            sep = QLabel("  │  ")
            sep.setStyleSheet("color: #222244;")
            sep.setFont(QFont("Monospace", 8))
            lay.addWidget(sep)
        lay.addWidget(self.lbl_rec)
        lay.addStretch()
        return bar

    # ──────────────────────────────────────────────────────── ports
    def _refresh_ports(self):
        ports = list_serial_ports()
        cur = self.combo_port.currentText()
        self.combo_port.clear()
        self.combo_port.addItems(ports if ports else ["(none)"])
        if cur in ports:
            self.combo_port.setCurrentText(cur)

    # ──────────────────────────────────────────────────────── connect/demo
    def _stop_reader(self):
        if self.reader:
            self.reader.stop()
            self.sender.set_reader(None)
            self.reader = None

    def _toggle_connect(self):
        if self.reader and self.reader.isRunning():
            self._stop_reader()
            self._set_btn_state(self.btn_connect, "🔗  Connect", False)
            self.btn_demo.setEnabled(True)
            self._set_status("Đã ngắt kết nối", '#666688')
        else:
            port = self.combo_port.currentText()
            baud = int(self.combo_baud.currentText())
            if not port or '(' in port:
                QMessageBox.warning(self, "Lỗi", "Chọn cổng serial hợp lệ.")
                return
            self.reader = SerialReaderThread(self.store)
            self.reader.configure(port, baud)
            self._wire_reader(self.reader)
            self.reader.start()
            self.btn_demo.setEnabled(False)

    def _toggle_demo(self):
        if self.reader and self.reader.isRunning():
            self._stop_reader()
            self._set_btn_state(self.btn_demo, "🎮  Demo", False)
            self.btn_connect.setEnabled(True)
            self._set_status("Demo dừng", '#666688')
        else:
            self.reader = DemoThread(self.store)
            self._wire_reader(self.reader)
            self.reader.start()
            self.btn_connect.setEnabled(False)
            self._set_btn_state(self.btn_demo, "⏹  Stop Demo", True)

    def _wire_reader(self, reader):
        self.sender.set_reader(reader)
        reader.connected.connect(self._on_connected)
        reader.disconnected.connect(self._on_disconnected)
        reader.error_msg.connect(self._on_error)
        reader.new_frame.connect(self._on_new_frame)
        reader.raw_line.connect(self.panel_console.store.raw_lines.append
                                if False else self._on_raw_line)

    def _on_raw_line(self, line: str):
        pass  # raw_line đã lưu trong store.raw_lines, Console tự refresh

    # ──────────────────────────────────────────────────────── record
    def _toggle_record(self):
        if not self._recording:
            path, _ = QFileDialog.getSaveFileName(
                self, "Lưu CSV",
                f"foc_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
                "CSV (*.csv)")
            if not path:
                return
            self._csv_file   = open(path, 'w', newline='')
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow([
                'time_s', 'mode',
                'vel_target', 'vel_filt', 'vel_raw', 'vq_vel', 'enc_vel',
                'pos_target', 'enc_pos', 'pos_err', 'vq_pos', 'vel_filt_pos'
            ])
            self._recording = True
            self._set_btn_state(self.btn_rec, "⏹  Stop Rec", True, rec=True)
            self.lbl_rec.setText(f"⏺ REC → {os.path.basename(path)}")
        else:
            self._recording = False
            if self._csv_file:
                self._csv_file.close()
                self._csv_file   = None
                self._csv_writer = None
            self._set_btn_state(self.btn_rec, "⏺  Record", False)
            self.lbl_rec.setText("")

    # ──────────────────────────────────────────────────────── callbacks
    def _on_connected(self, port: str):
        self._set_status(f"● Đã kết nối: {port}", '#4ECDC4')
        self._set_btn_state(self.btn_connect, "⏹  Disconnect", True)

    def _on_disconnected(self):
        self._set_status("● Đã ngắt kết nối", '#666688')
        self._set_btn_state(self.btn_connect, "🔗  Connect", False)
        self.btn_demo.setEnabled(True)
        self.btn_connect.setEnabled(True)

    def _on_error(self, msg: str):
        self._set_status(f"✗ {msg}", '#FF6B6B')

    def _on_new_frame(self):
        self._frame_cnt += 1
        # CSV record
        if self._recording and self._csv_writer:
            t = self.store.get_time()
            if not len(t):
                return
            ts = float(t[-1])
            s  = self.store
            def _l(q): return float(s.get(q)[-1]) if len(q) else 0.0
            self._csv_writer.writerow([
                f"{ts:.4f}", s.get_mode(),
                _l(s.vel_target), _l(s.vel_filt), _l(s.vel_raw),
                _l(s.vq_vel), _l(s.enc_vel),
                _l(s.pos_target), _l(s.enc_pos), _l(s.pos_err),
                _l(s.vq_pos), _l(s.vel_filt_pos),
            ])

    # ──────────────────────────────────────────────────────── timer
    def _on_timer(self):
        # Refresh FOC panels
        foc_idx = self._foc_tabs.currentIndex()
        panels  = [self.panel_telem, self.panel_ctrl,
                   self.panel_pid, self.panel_console]
        if 0 <= foc_idx < len(panels):
            try:
                panels[foc_idx].refresh()
            except Exception:
                pass

        # FPS counter
        now = time.monotonic()
        dt  = now - self._last_fps_t
        if dt >= 1.0:
            self._fps = self._frame_cnt / dt
            self._frame_cnt  = 0
            self._last_fps_t = now
            self.lbl_fps.setText(f"FPS: {self._fps:.1f}")

    # ──────────────────────────────────────────────────────── helpers
    def _set_status(self, msg: str, color: str = '#666688'):
        self.lbl_status.setText(msg)
        self.lbl_status.setStyleSheet(f"color: {color}; font-family: Monospace; font-size: 8pt;")

    def _set_btn_state(self, btn: QPushButton, text: str, on: bool, rec: bool = False):
        btn.setText(text)
        if rec:
            btn.setObjectName("btn_rec")
        else:
            btn.setObjectName("btn_on" if on else "")
        btn.setStyleSheet("")  # force re-apply stylesheet

    def closeEvent(self, event):
        self._stop_reader()
        if self._csv_file:
            self._csv_file.close()
        super().closeEvent(event)


# ─────────── small helpers ─────────────────────────────────────────────────
def _plain(text: str) -> QLabel:
    l = QLabel(text)
    l.setStyleSheet("color: #888888; font-size: 9pt;")
    return l


def _status(text: str, color: str) -> QLabel:
    l = QLabel(text)
    l.setFont(QFont("Monospace", 8))
    l.setStyleSheet(f"color: {color};")
    return l
