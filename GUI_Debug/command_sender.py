"""
command_sender.py — Helper gửi lệnh #CMD xuống STM32 qua serial_reader

Bọc các lệnh thành các hàm Python dễ gọi từ GUI panels.
"""


class CommandSender:
    """
    Wrapper gửi lệnh #CMD xuống MCU thông qua SerialReaderThread.send().
    Nếu chưa kết nối (reader=None), lệnh bị bỏ qua.
    """

    def __init__(self):
        self._reader = None

    def set_reader(self, reader):
        """Gắn vào SerialReaderThread hoặc DemoThread hiện tại."""
        self._reader = reader

    def _send(self, cmd: str):
        if self._reader and hasattr(self._reader, 'send'):
            self._reader.send(cmd)

    # ------------------------------------------------------------------
    # Lệnh điều khiển chế độ
    # ------------------------------------------------------------------
    def set_mode_velocity(self):
        self._send('#MODE,VEL')

    def set_mode_position(self):
        self._send('#MODE,POS')

    # ------------------------------------------------------------------
    # Lệnh setpoint
    # ------------------------------------------------------------------
    def set_target_position(self, deg: float):
        """Đặt góc mục tiêu (độ)."""
        self._send(f'#TPOS,{deg:.2f}')

    def set_target_velocity(self, rad_s: float):
        """Đặt vận tốc mục tiêu (rad/s)."""
        self._send(f'#TVEL,{rad_s:.3f}')

    # ------------------------------------------------------------------
    # Lệnh PID tuning
    # ------------------------------------------------------------------
    def set_pid_pos(self, kp: float, ki: float, kd: float):
        self._send(f'#PID_POS,{kp:.4f},{ki:.4f},{kd:.4f}')

    def set_pid_vel(self, kp: float, ki: float, kd: float):
        self._send(f'#PID_VEL,{kp:.4f},{ki:.4f},{kd:.4f}')

    def set_lpf_alpha(self, alpha: float):
        self._send(f'#LPF,{alpha:.4f}')

    def set_voltage_limit(self, vlim: float):
        self._send(f'#VLIM,{vlim:.2f}')

    # ------------------------------------------------------------------
    # Lệnh điều khiển motor
    # ------------------------------------------------------------------
    def stop_motor(self):
        self._send('#STOP')

    def start_motor(self):
        self._send('#START')

    def re_align(self):
        self._send('#ALIGN')

    # ------------------------------------------------------------------
    # Gửi lệnh thô (cho Console tab)
    # ------------------------------------------------------------------
    def send_raw(self, text: str):
        self._send(text.strip())
