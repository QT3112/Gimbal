"""
data_store.py — Ring buffer lưu dữ liệu telemetry FOC real-time

Giao thức nhận:
  $VEL,target_vel,vel_filt,vel_raw,vq,enc_deg   (chế độ velocity)
  $POS,target_pos,enc_deg,err_deg,vq,vel_filt    (chế độ position)
"""

import math
import threading
import numpy as np
from collections import deque

BUFFER_SIZE = 2000   # ~200s at 10Hz


class FOCDataStore:
    """Thread-safe ring buffer cho tất cả kênh telemetry FOC."""

    def __init__(self, size: int = BUFFER_SIZE):
        self.size = size
        self._lock = threading.Lock()

        # --- Timestamp ---
        self.time = deque(maxlen=size)
        self._t0 = None

        # --- $VEL: Velocity mode ---
        self.vel_target   = deque(maxlen=size)
        self.vel_filt     = deque(maxlen=size)
        self.vel_raw      = deque(maxlen=size)
        self.vq_vel       = deque(maxlen=size)
        self.enc_vel      = deque(maxlen=size)

        # --- $POS: Position mode ---
        self.pos_target   = deque(maxlen=size)
        self.enc_pos      = deque(maxlen=size)
        self.pos_err      = deque(maxlen=size)
        self.vq_pos       = deque(maxlen=size)
        self.vel_filt_pos = deque(maxlen=size)

        # --- DEMO: Demo 1 Axis mode ---
        self.demo_roll    = deque(maxlen=size)
        self.demo_gyro    = deque(maxlen=size)
        self.demo_enc     = deque(maxlen=size)
        self.demo_vq      = deque(maxlen=size)
        self.demo_vel     = deque(maxlen=size)

        # --- Chế độ hiện tại ---
        self.current_mode = 'POS'

        # --- Console: raw lines ---
        self.raw_lines = deque(maxlen=500)

    # ------------------------------------------------------------------
    def _push_time(self, t: float):
        if self._t0 is None:
            self._t0 = t
        self.time.append(t - self._t0)

    # ------------------------------------------------------------------
    def push_line(self, line: str, timestamp: float):
        """Parse 1 dòng telemetry và đẩy vào buffer (thread-safe)."""
        line = line.strip()
        if not line:
            return

        self.raw_lines.append((timestamp, line))

        tag, _, body = line.partition(',')
        try:
            vals = [float(x) for x in body.split(',')]
        except ValueError:
            return

        with self._lock:
            if tag == '$VEL' and len(vals) >= 5:
                self._push_time(timestamp)
                self.current_mode = 'VEL'
                self.vel_target.append(vals[0])
                self.vel_filt.append(vals[1])
                self.vel_raw.append(vals[2])
                self.vq_vel.append(vals[3])
                self.enc_vel.append(vals[4])

            elif tag == '$POS' and len(vals) >= 5:
                self._push_time(timestamp)
                self.current_mode = 'POS'
                self.pos_target.append(vals[0])
                self.enc_pos.append(vals[1])
                self.pos_err.append(vals[2])
                self.vq_pos.append(vals[3])
                self.vel_filt_pos.append(vals[4])
                
            elif tag == '[DEMO_1AXIS]' and 'Roll:' in line:
                import re
                match = re.search(r"Roll:\s*([-\d.]+).*?GyroX:\s*([-\d.]+).*?Enc:\s*([-\d.]+).*?Vq:\s*([-\d.]+).*?Vel:\s*([-\d.]+)", line)
                if match:
                    self._push_time(timestamp)
                    self.current_mode = 'DEMO'
                    self.demo_roll.append(float(match.group(1)))
                    self.demo_gyro.append(float(match.group(2)))
                    self.demo_enc.append(float(match.group(3)))
                    self.demo_vq.append(float(match.group(4)))
                    self.demo_vel.append(float(match.group(5)))

    # ------------------------------------------------------------------
    def get(self, channel: deque) -> np.ndarray:
        with self._lock:
            return np.array(channel, dtype=np.float32)

    def get_time(self) -> np.ndarray:
        return self.get(self.time)

    def get_mode(self) -> str:
        with self._lock:
            return self.current_mode

    def reset_time(self):
        with self._lock:
            self._t0 = None
            self.time.clear()

    # ------------------------------------------------------------------
    def push_demo(self, t: float):
        """Sinh dữ liệu giả lập để test GUI offline."""
        noise = lambda: (np.random.rand() - 0.5) * 0.05
        # Đổi chế độ mỗi 6s
        if int(t / 6) % 2 == 0:
            target = 3.0 * math.sin(t * 0.8)
            filt = target * (1 - math.exp(-t * 2)) + noise()
            raw = filt + (np.random.rand() - 0.5) * 0.8
            vq = (target - filt) * 0.12 + noise()
            enc = (t * 30) % 360
            line = f"$VEL,{target:.2f},{filt:.2f},{raw:.2f},{vq:.2f},{enc:.2f}"
        else:
            targets = [0, 90, 180, -90]
            target = float(targets[int(t / 1.5) % 4])
            phase = t % 1.5
            enc = target * (1 - math.exp(-phase * 4)) + noise() * 2
            err = target - enc
            vq = err * 0.05 + noise()
            vf = err * 0.3 + noise()
            line = f"$POS,{target:.1f},{enc:.1f},{err:.1f},{vq:.2f},{vf:.2f}"
        self.push_line(line, t)


# Backward compat alias cho app cũ nếu còn cần
GimbalDataStore = FOCDataStore
