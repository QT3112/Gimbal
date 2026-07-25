#!/usr/bin/env python3
"""
main.py — Entry point: FOC Debug Monitor v2.0

Chạy:
    python main.py              (kết nối phần cứng)
    python main.py --demo       (demo mode không cần MCU)
"""

import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont, QFontDatabase

from ui.main_window import MainWindow


def main():
    QApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)

    app = QApplication(sys.argv)
    app.setApplicationName("FOC Debug Monitor")
    app.setApplicationVersion("2.0.0")
    app.setOrganizationName("STM32G431 Gimbal")

    # Font
    app.setFont(QFont("Inter", 10))

    window = MainWindow()
    window.show()

    if '--demo' in sys.argv:
        window._toggle_demo()

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
