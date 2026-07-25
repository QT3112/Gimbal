"""
panel_pid_tuning.py — Tab Live PID Tuning

Điều chỉnh Kp/Ki/Kd cho Position PID và Velocity PID,
LPF alpha, Voltage Limit — gửi ngay xuống MCU qua #CMD.
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QPushButton, QDoubleSpinBox, QGroupBox, QSlider,
    QGridLayout, QSizePolicy
)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


# ─────────────────────────────── helpers ────────────────────────────────
def _group(title: str, color: str = '#4ECDC4') -> QGroupBox:
    g = QGroupBox(title)
    g.setStyleSheet(f"""
        QGroupBox {{
            color: #AAAAAA; font-size: 9pt;
            border: 1px solid #2A2A4A; border-radius: 8px;
            margin-top: 10px; padding-top: 8px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin; left: 12px;
            padding: 0 6px; color: {color}; font-weight: bold; font-size: 10pt;
        }}
    """)
    return g


def _spinbox(lo: float, hi: float, val: float, step: float = 0.01,
             decimals: int = 4, width: int = 100) -> QDoubleSpinBox:
    s = QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setDecimals(decimals)
    s.setSingleStep(step)
    s.setValue(val)
    s.setFixedWidth(width)
    s.setStyleSheet("""
        QDoubleSpinBox {
            background: #1A1A2E; border: 1px solid #3A3A5A;
            border-radius: 4px; color: #E0E0E0; padding: 4px 6px;
            font-size: 10pt; font-family: Monospace;
        }
        QDoubleSpinBox:focus { border-color: #4ECDC4; }
    """)
    return s


def _apply_btn(text: str = "📤  Apply", color: str = '#1B4332',
               tc: str = '#4ECDC4', bc: str = '#2E8B57') -> QPushButton:
    b = QPushButton(text)
    b.setFont(QFont("Inter", 10, QFont.Weight.Bold))
    b.setStyleSheet(f"""
        QPushButton {{
            background: {color}; color: {tc}; border: 1.5px solid {bc};
            border-radius: 6px; padding: 6px 18px;
        }}
        QPushButton:hover  {{ background: #265A38; border-color: #4ECDC4; }}
        QPushButton:pressed{{ background: #3B7A4F; }}
    """)
    return b


def _lbl(text: str, color: str = '#AAAAAA', bold: bool = False) -> QLabel:
    l = QLabel(text)
    w = QFont.Weight.Bold if bold else QFont.Weight.Normal
    l.setFont(QFont("Inter", 9, w))
    l.setStyleSheet(f"color: {color};")
    return l


def _hline() -> QFrame:
    f = QFrame()
    f.setFrameShape(QFrame.Shape.HLine)
    f.setStyleSheet("color: #2A2A4A;")
    return f


# ─────────────────────────────── PID Group Widget ────────────────────────────
class PIDGroupWidget(QFrame):
    """Widget chứa Kp, Ki, Kd spinbox + Apply button."""

    def __init__(self, title: str, color: str, defaults: tuple,
                 out_lim: tuple = None, parent=None):
        super().__init__(parent)
        self.setStyleSheet(f"""
            QFrame {{
                background: #14142A;
                border: 1.5px solid {color};
                border-radius: 10px;
                padding: 6px;
            }}
        """)
        lay = QVBoxLayout(self)
        lay.setSpacing(8)

        # Title
        ttl = QLabel(title)
        ttl.setFont(QFont("Inter", 11, QFont.Weight.Bold))
        ttl.setStyleSheet(f"color: {color}; border: none;")
        lay.addWidget(ttl)

        # Kp Ki Kd row
        grid = QGridLayout()
        grid.setHorizontalSpacing(16)

        kp0, ki0, kd0 = defaults
        for col, (name, val, lo, hi, step) in enumerate([
            ("Kp", kp0,  0.0, 100.0,  0.01),
            ("Ki", ki0,  0.0, 100.0,  0.01),
            ("Kd", kd0,  0.0,  10.0,  0.001),
        ]):
            col_lbl = _lbl(name, color, bold=True)
            col_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            spin = _spinbox(lo, hi, val, step)
            grid.addWidget(col_lbl, 0, col)
            grid.addWidget(spin,    1, col)
        self.sp_kp = grid.itemAtPosition(1, 0).widget()
        self.sp_ki = grid.itemAtPosition(1, 1).widget()
        self.sp_kd = grid.itemAtPosition(1, 2).widget()
        lay.addLayout(grid)

        # Output limit info
        if out_lim:
            lim_lbl = _lbl(f"Output clamp: [{out_lim[0]} ~ {out_lim[1]}]", '#666688')
            lay.addWidget(lim_lbl)

        # Apply button
        self.apply_btn = _apply_btn()
        lay.addWidget(self.apply_btn, alignment=Qt.AlignmentFlag.AlignRight)

    def get_kpkikd(self) -> tuple[float, float, float]:
        return self.sp_kp.value(), self.sp_ki.value(), self.sp_kd.value()

    def set_kpkikd(self, kp: float, ki: float, kd: float):
        self.sp_kp.setValue(kp)
        self.sp_ki.setValue(ki)
        self.sp_kd.setValue(kd)


# ─────────────────────────────── Main Panel ──────────────────────────────────
class PIDTuningPanel(QWidget):
    """Tab 3: Live PID Tuning."""

    def __init__(self, store, sender, parent=None):
        super().__init__(parent)
        self.store  = store
        self.sender = sender
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # Title
        title = QLabel("🔧  Live PID Tuning")
        title.setFont(QFont("Inter", 13, QFont.Weight.Bold))
        title.setStyleSheet("color: #CE93D8;")
        root.addWidget(title)

        # ── Position PID ──
        self.pos_pid = PIDGroupWidget(
            "📍  Position Loop PID",
            color='#4ECDC4',
            defaults=(6.0, 0.4, 0.0),
            out_lim=("-3.0 rad/s", "+3.0 rad/s")
        )
        self.pos_pid.apply_btn.clicked.connect(self._apply_pos_pid)
        root.addWidget(self.pos_pid)

        # ── Velocity PID ──
        self.vel_pid = PIDGroupWidget(
            "💨  Velocity Loop PID",
            color='#81C784',
            defaults=(0.12, 0.4, 0.0),
            out_lim=("-Vlim", "+Vlim")
        )
        self.vel_pid.apply_btn.clicked.connect(self._apply_vel_pid)
        root.addWidget(self.vel_pid)

        # ── LPF + Voltage Limit ──
        misc_grp = _group("⚙️  Velocity LPF  &  Voltage Limit", '#FFB74D')
        misc_lay = QGridLayout(misc_grp)
        misc_lay.setHorizontalSpacing(20)

        # LPF
        misc_lay.addWidget(_lbl("LPF Alpha  (0 ~ 1.0):", '#FFB74D', True), 0, 0)
        self.sp_lpf = _spinbox(0.0, 0.9999, 0.96, 0.01, 4, 110)
        misc_lay.addWidget(self.sp_lpf, 0, 1)
        lpf_apply = _apply_btn("📤  Apply LPF", '#2A1B0A', '#FFB74D', '#AA7700')
        lpf_apply.clicked.connect(self._apply_lpf)
        misc_lay.addWidget(lpf_apply, 0, 2)

        # Voltage Limit
        misc_lay.addWidget(_lbl("Voltage Limit  (V):", '#FF6B6B', True), 1, 0)
        self.sp_vlim = _spinbox(0.1, 12.0, 1.5, 0.1, 2, 110)
        misc_lay.addWidget(self.sp_vlim, 1, 1)
        vlim_apply = _apply_btn("📤  Apply VLim", '#2A0A0A', '#FF6B6B', '#AA2222')
        vlim_apply.clicked.connect(self._apply_vlim)
        misc_lay.addWidget(vlim_apply, 1, 2)

        root.addWidget(misc_grp)

        # ── Step Response Analysis (from store) ──
        resp_grp = _group("📈  Step Response Analysis  (auto from telemetry)", '#CE93D8')
        resp_lay = QHBoxLayout(resp_grp)

        self.lbl_rise   = self._metric_card("Rise Time",   "--- ms",  '#4ECDC4')
        self.lbl_settle = self._metric_card("Settle Time", "--- ms",  '#81C784')
        self.lbl_os     = self._metric_card("Overshoot",   "---%",    '#FF6B6B')
        self.lbl_sse    = self._metric_card("SS Error",    "--- °",   '#FFB74D')
        for c in (self.lbl_rise, self.lbl_settle, self.lbl_os, self.lbl_sse):
            resp_lay.addWidget(c)

        root.addWidget(resp_grp)
        root.addStretch()

    # ---------------------------------------------------------------- apply
    def _apply_pos_pid(self):
        kp, ki, kd = self.pos_pid.get_kpkikd()
        self.sender.set_pid_pos(kp, ki, kd)

    def _apply_vel_pid(self):
        kp, ki, kd = self.vel_pid.get_kpkikd()
        self.sender.set_pid_vel(kp, ki, kd)

    def _apply_lpf(self):
        self.sender.set_lpf_alpha(self.sp_lpf.value())

    def _apply_vlim(self):
        self.sender.set_voltage_limit(self.sp_vlim.value())

    # ---------------------------------------------------------------- metric card
    def _metric_card(self, label: str, init: str, color: str) -> QFrame:
        f = QFrame()
        f.setStyleSheet(f"""
            QFrame {{
                background: #14142A; border: 1px solid {color};
                border-radius: 8px; padding: 6px;
            }}
        """)
        lay = QVBoxLayout(f)
        lay.setSpacing(2)
        lbl = QLabel(label)
        lbl.setFont(QFont("Inter", 8))
        lbl.setStyleSheet(f"color: {color}; border: none;")
        val = QLabel(init)
        val.setFont(QFont("Monospace", 14, QFont.Weight.Bold))
        val.setStyleSheet("color: #FFFFFF; border: none;")
        val.setAlignment(Qt.AlignmentFlag.AlignCenter)
        lay.addWidget(lbl)
        lay.addWidget(val)
        f._val_lbl = val
        return f

    # ---------------------------------------------------------------- refresh
    def refresh(self):
        """Phân tích step response từ dữ liệu store (cơ bản)."""
        mode = self.store.get_mode()
        if mode == 'POS':
            t   = self.store.get_time()
            err = self.store.get(self.store.pos_err)
            tgt = self.store.get(self.store.pos_target)
        else:
            t   = self.store.get_time()
            err_arr = self.store.get(self.store.vel_filt)
            tgt = self.store.get(self.store.vel_target)
            err = tgt - err_arr  # velocity error

        if len(err) < 20:
            return

        # Steady-state error: trung bình 10 sample cuối
        sse = float(np.mean(np.abs(err[-10:])))
        self.lbl_sse._val_lbl.setText(
            f"{sse:.2f} °" if mode == 'POS' else f"{sse:.3f} r/s")

        # Overshoot: dùng abs(max(err) so với step size)
        if len(tgt) > 2 and float(np.std(tgt)) > 0.5:
            step_size = float(np.max(np.abs(tgt)))
            os_pct = float(np.max(np.abs(err))) / max(step_size, 0.001) * 100.0
            self.lbl_os._val_lbl.setText(f"{os_pct:.1f}%")
