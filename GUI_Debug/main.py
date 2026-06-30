#!/usr/bin/env python3
"""
main.py — Entry point: Gimbal Debug Monitor

Yêu cầu:
    pip install pyqt6 pyqtgraph pyserial numpy

Chạy:
    python main.py
    python main.py --demo    (chạy chế độ giả lập không cần phần cứng)
"""

import sys
import os

# Thêm thư mục gốc vào path để import các module ngang hàng
sys.path.insert(0, os.path.dirname(__file__))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from ui.main_window import MainWindow


def main():
    # Bật High-DPI scaling
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough
    )

    app = QApplication(sys.argv)
    app.setApplicationName("Gimbal Debug Monitor")
    app.setApplicationVersion("1.0.0")
    app.setOrganizationName("STM32G431 Gimbal Project")

    # Font mặc định
    default_font = QFont("Inter", 10)
    app.setFont(default_font)

    window = MainWindow()
    window.show()

    # Nếu truyền --demo, tự động bật Demo mode
    if '--demo' in sys.argv:
        window._toggle_demo()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
