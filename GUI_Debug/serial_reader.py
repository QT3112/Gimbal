"""
serial_reader.py — Thread đọc cổng Serial USB CDC và parse CSV tagged

Chạy trong QThread riêng để không block UI.
Phát signal PyQt mỗi khi có frame dữ liệu mới ($IMU + $ATT + $PID + $FOC).
"""

import time
import threading
from PyQt6.QtCore import QThread, pyqtSignal

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def list_serial_ports() -> list[str]:
    """Trả về danh sách tên cổng serial khả dụng."""
    if not SERIAL_AVAILABLE:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


class SerialReaderThread(QThread):
    """
    QThread đọc serial và push dữ liệu vào GimbalDataStore.

    Signals:
        new_frame  — phát mỗi khi nhận đủ 1 bộ 4 dòng ($IMU,$ATT,$PID,$FOC)
        error_msg  — phát khi có lỗi kết nối
        connected  — phát khi kết nối thành công
        disconnected — phát khi mất kết nối
    """

    new_frame    = pyqtSignal()
    error_msg    = pyqtSignal(str)
    connected    = pyqtSignal(str)       # arg: port name
    disconnected = pyqtSignal()

    EXPECTED_TAGS = {'$IMU', '$ATT', '$PID', '$FOC'}

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store = store
        self._port     = ''
        self._baudrate = 115200
        self._running  = False
        self._ser      = None

    def configure(self, port: str, baudrate: int = 115200):
        self._port     = port
        self._baudrate = baudrate

    def stop(self):
        self._running = False
        self.wait(2000)

    # ------------------------------------------------------------------
    def run(self):
        """Vòng lặp chính của thread."""
        if not SERIAL_AVAILABLE:
            self.error_msg.emit("pyserial chưa được cài đặt!")
            return

        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=1.0
            )
        except serial.SerialException as e:
            self.error_msg.emit(f"Không mở được cổng {self._port}: {e}")
            return

        self._running = True
        self.connected.emit(self._port)

        received_tags = set()
        try:
            while self._running:
                try:
                    raw = self._ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('utf-8', errors='replace').strip()
                except (serial.SerialException, OSError) as e:
                    self.error_msg.emit(f"Lỗi đọc serial: {e}")
                    break

                if not line.startswith('$'):
                    continue

                tag = line.split(',')[0]
                if tag not in self.EXPECTED_TAGS:
                    continue

                ts = time.monotonic()
                self.store.push_line(line, ts)
                received_tags.add(tag)

                # Phát signal khi nhận đủ 4 loại dòng trong 1 batch
                if received_tags >= self.EXPECTED_TAGS:
                    received_tags.clear()
                    self.new_frame.emit()

        finally:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self.disconnected.emit()

    def send(self, text: str):
        """Gửi lệnh ngược lại MCU (dùng cho Live Tuning sau này)."""
        if self._ser and self._ser.is_open:
            self._ser.write(text.encode())


class DemoThread(QThread):
    """
    Thread sinh dữ liệu giả lập để test GUI không cần phần cứng.
    API giống SerialReaderThread.
    """

    new_frame    = pyqtSignal()
    error_msg    = pyqtSignal(str)
    connected    = pyqtSignal(str)
    disconnected = pyqtSignal()

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store    = store
        self._running = False

    def configure(self, port: str = 'DEMO', baudrate: int = 0):
        pass

    def stop(self):
        self._running = False
        self.wait(1000)

    def run(self):
        self._running = True
        self.connected.emit('DEMO')
        t = 0.0
        while self._running:
            self.store.push_demo(t)
            self.new_frame.emit()
            t += 0.02
            self.msleep(20)   # 50Hz
        self.disconnected.emit()
