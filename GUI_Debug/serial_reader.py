"""
serial_reader.py — Thread đọc/ghi USB CDC bi-directional

STM32 -> PC: dòng $VEL,... hoặc $POS,...
PC -> STM32: lệnh #CMD,... qua send()
"""

import time
from PyQt6.QtCore import QThread, pyqtSignal

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


def list_serial_ports() -> list[str]:
    if not SERIAL_AVAILABLE:
        return []
    return [p.device for p in serial.tools.list_ports.comports()]


class SerialReaderThread(QThread):
    """
    QThread đọc serial line-by-line và push vào FOCDataStore.
    Cũng cung cấp send() để gửi lệnh #CMD xuống MCU.
    """

    new_frame    = pyqtSignal()
    error_msg    = pyqtSignal(str)
    connected    = pyqtSignal(str)
    disconnected = pyqtSignal()
    raw_line     = pyqtSignal(str)    # mỗi dòng raw nhận được -> Console

    def __init__(self, store, parent=None):
        super().__init__(parent)
        self.store     = store
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

    def run(self):
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
        self.store.reset_time()
        self.connected.emit(self._port)

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

                if not line:
                    continue

                ts = time.monotonic()
                self.raw_line.emit(line)

                # Parse $ telemetry hoặc [DEMO_1AXIS] vào store
                if line.startswith('$') or line.startswith('[DEMO_1AXIS]'):
                    self.store.push_line(line, ts)
                    self.new_frame.emit()

        finally:
            if self._ser and self._ser.is_open:
                self._ser.close()
            self.disconnected.emit()

    def send(self, text: str):
        """Gửi lệnh #CMD xuống MCU."""
        if self._ser and self._ser.is_open:
            if not text.endswith('\n'):
                text += '\n'
            try:
                self._ser.write(text.encode('utf-8'))
            except (serial.SerialException, OSError):
                pass


class DemoThread(QThread):
    """Thread sinh dữ liệu giả lập để test GUI không cần phần cứng."""

    new_frame    = pyqtSignal()
    error_msg    = pyqtSignal(str)
    connected    = pyqtSignal(str)
    disconnected = pyqtSignal()
    raw_line     = pyqtSignal(str)

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
        self.store.reset_time()
        self.connected.emit('DEMO')
        t = 0.0
        while self._running:
            self.store.push_demo(t)
            # Phát raw line cho Console
            if self.store.raw_lines:
                _, last_line = self.store.raw_lines[-1]
                self.raw_line.emit(last_line)
            self.new_frame.emit()
            t += 0.1
            self.msleep(100)   # 10Hz
        self.disconnected.emit()

    def send(self, text: str):
        """Demo mode: hiển thị lệnh như echo."""
        pass
