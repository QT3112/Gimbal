"""
panel_console.py — Tab Console: raw serial log + gửi lệnh thủ công

Features:
  - Hiển thị mọi dòng nhận được từ MCU (raw, có timestamp)
  - Input box gửi lệnh #CMD thủ công
  - Filter/search, Clear, Pause scroll
"""

from PyQt6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QPushButton, QLineEdit, QTextEdit, QCheckBox, QSizePolicy
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont, QColor, QTextCursor


def _btn(text: str, color: str = '#2A2A4A', tc: str = '#E0E0E0',
         bc: str = '#3A3A5A', w: int = None) -> QPushButton:
    b = QPushButton(text)
    b.setStyleSheet(f"""
        QPushButton {{
            background: {color}; color: {tc}; border: 1px solid {bc};
            border-radius: 5px; padding: 4px 12px; font-weight: bold;
        }}
        QPushButton:hover  {{ background: #3A3A6A; border-color: #4ECDC4; }}
        QPushButton:pressed{{ background: #4A4A7A; }}
    """)
    if w:
        b.setFixedWidth(w)
    return b


class ConsolePanel(QWidget):
    """Tab 4: Raw serial log console."""

    MAX_LINES = 2000

    def __init__(self, store, sender, parent=None):
        super().__init__(parent)
        self.store  = store
        self.sender = sender
        self._paused = False
        self._last_line_count = 0
        self._filter_text = ''
        self._setup_ui()

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(6)

        # Header
        hdr = QHBoxLayout()
        title = QLabel("📋  Serial Console")
        title.setFont(QFont("Inter", 13, QFont.Weight.Bold))
        title.setStyleSheet("color: #FFCC02;")
        hdr.addWidget(title)
        hdr.addStretch()

        # Toolbar
        self.btn_pause = _btn("⏸  Pause Scroll", '#1A2A3A', '#4ECDC4', '#334466')
        self.btn_pause.setCheckable(True)
        self.btn_pause.toggled.connect(self._on_pause)
        self.btn_clear = _btn("🗑  Clear", '#2A1A1A', '#FF6B6B', '#AA3333')
        self.btn_clear.clicked.connect(self._clear)

        self.chk_filter_vel = QCheckBox("$VEL only")
        self.chk_filter_pos = QCheckBox("$POS only")
        for chk in (self.chk_filter_vel, self.chk_filter_pos):
            chk.setStyleSheet("color:#AAAAAA; font-size:9pt;")
            chk.toggled.connect(self._update_filter)

        hdr.addWidget(self.chk_filter_vel)
        hdr.addWidget(self.chk_filter_pos)
        hdr.addWidget(self.btn_pause)
        hdr.addWidget(self.btn_clear)
        root.addLayout(hdr)

        # Log area
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFont(QFont("Monospace", 9))
        self.log_view.setStyleSheet("""
            QTextEdit {
                background: #0A0A1A; color: #CCCCCC;
                border: 1px solid #1A1A3A; border-radius: 6px;
                padding: 6px;
            }
        """)
        root.addWidget(self.log_view, 1)

        # Search row
        search_row = QHBoxLayout()
        srch_lbl = QLabel("🔍")
        srch_lbl.setStyleSheet("color:#888888;")
        self.search_box = QLineEdit()
        self.search_box.setPlaceholderText("Filter/search in log...")
        self.search_box.setStyleSheet("""
            QLineEdit {
                background: #12122A; border: 1px solid #3A3A5A;
                border-radius: 4px; color: #E0E0E0; padding: 4px 8px;
            }
            QLineEdit:focus { border-color: #4ECDC4; }
        """)
        self.search_box.textChanged.connect(self._update_filter)
        search_row.addWidget(srch_lbl)
        search_row.addWidget(self.search_box, 1)
        root.addLayout(search_row)

        # Command input row
        cmd_frame = QFrame()
        cmd_frame.setStyleSheet(
            "background:#12122A; border:1px solid #2A2A4A; border-radius:6px;")
        cmd_lay = QHBoxLayout(cmd_frame)
        cmd_lay.setContentsMargins(8, 6, 8, 6)

        cmd_label = QLabel("TX ▶")
        cmd_label.setFont(QFont("Monospace", 10, QFont.Weight.Bold))
        cmd_label.setStyleSheet("color:#4ECDC4; border:none;")
        self.cmd_input = QLineEdit()
        self.cmd_input.setPlaceholderText("#TPOS,90.0  |  #PID_POS,6.0,0.4,0.0  |  #STOP  ...")
        self.cmd_input.setFont(QFont("Monospace", 10))
        self.cmd_input.setStyleSheet("""
            QLineEdit {
                background: transparent; border: none;
                color: #E0E0E0; padding: 2px 4px;
            }
        """)
        self.cmd_input.returnPressed.connect(self._send_cmd)
        send_btn = _btn("Send ↵", '#1B4332', '#4ECDC4', '#2E8B57', 80)
        send_btn.clicked.connect(self._send_cmd)
        cmd_lay.addWidget(cmd_label)
        cmd_lay.addWidget(self.cmd_input, 1)
        cmd_lay.addWidget(send_btn)
        root.addWidget(cmd_frame)

        # History
        self._cmd_history = []
        self._hist_idx = -1
        self.cmd_input.installEventFilter(self)

    # ──────────────────────────────────────────────────── slots
    def _on_pause(self, paused: bool):
        self._paused = paused
        self.btn_pause.setText("▶  Resume" if paused else "⏸  Pause Scroll")

    def _clear(self):
        self.log_view.clear()
        self._last_line_count = len(self.store.raw_lines)

    def _update_filter(self):
        self._filter_text = self.search_box.text().lower()

    def _send_cmd(self):
        text = self.cmd_input.text().strip()
        if not text:
            return
        self.sender.send_raw(text)
        # Hiển thị trong log với màu vàng (TX)
        self.log_view.moveCursor(QTextCursor.MoveOperation.End)
        self.log_view.insertHtml(
            f'<span style="color:#FFCC02;">▲ TX: {text}</span><br>')
        self._cmd_history.append(text)
        self._hist_idx = -1
        self.cmd_input.clear()

    # ──────────────────────────────────────────────────── key navigation
    def eventFilter(self, obj, event):
        from PyQt6.QtCore import QEvent
        from PyQt6.QtGui import QKeyEvent
        from PyQt6.QtCore import Qt as _Qt
        if obj is self.cmd_input and event.type() == QEvent.Type.KeyPress:
            if event.key() == _Qt.Key.Key_Up and self._cmd_history:
                self._hist_idx = max(0, self._hist_idx - 1
                                     if self._hist_idx >= 0
                                     else len(self._cmd_history) - 1)
                self.cmd_input.setText(self._cmd_history[self._hist_idx])
                return True
            elif event.key() == _Qt.Key.Key_Down and self._cmd_history:
                self._hist_idx = min(len(self._cmd_history) - 1,
                                     self._hist_idx + 1)
                if self._hist_idx < len(self._cmd_history):
                    self.cmd_input.setText(self._cmd_history[self._hist_idx])
                return True
        return super().eventFilter(obj, event)

    # ──────────────────────────────────────────────────── refresh
    def refresh(self):
        """Cập nhật log với các dòng mới từ store.raw_lines."""
        lines = list(self.store.raw_lines)
        new_lines = lines[self._last_line_count:]
        self._last_line_count = len(lines)

        if not new_lines or self._paused:
            return

        # Filter
        fv = self.chk_filter_vel.isChecked()
        fp = self.chk_filter_pos.isChecked()
        srch = self._filter_text

        self.log_view.moveCursor(QTextCursor.MoveOperation.End)

        for ts, line in new_lines:
            # Apply tag filters
            if fv and not line.startswith('$VEL'):
                continue
            if fp and not line.startswith('$POS'):
                continue
            # Apply search filter
            if srch and srch not in line.lower():
                continue

            # Color coding
            if line.startswith('$VEL'):
                color = '#4FC3F7'
            elif line.startswith('$POS'):
                color = '#81C784'
            elif line.startswith('#'):
                color = '#FFCC02'
            else:
                color = '#AAAAAA'

            from datetime import datetime
            time_str = f"{ts % 3600:.3f}"
            self.log_view.insertHtml(
                f'<span style="color:#444466;">[{time_str}]</span> '
                f'<span style="color:{color};">{line}</span><br>')

        # Giới hạn dòng
        doc = self.log_view.document()
        while doc.lineCount() > self.MAX_LINES:
            cursor = QTextCursor(doc)
            cursor.movePosition(QTextCursor.MoveOperation.Start)
            cursor.select(QTextCursor.SelectionType.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

        # Scroll xuống cuối
        self.log_view.moveCursor(QTextCursor.MoveOperation.End)
        self.log_view.ensureCursorVisible()
