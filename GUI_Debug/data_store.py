"""
data_store.py — Ring buffer lưu dữ liệu real-time từ Gimbal STM32

Cấu trúc CSV tagged được parse:
  $IMU  → 12 giá trị: ax1,ay1,az1,gx1,gy1,gz1, ax2,ay2,az2,gx2,gy2,gz2
  $ATT  → 8  giá trị: frame_p,frame_r,frame_y, pay_p,pay_r,pay_y, rel_p,rel_r  [deg]
  $PID  → 12 giá trị: p_err,t_vel_p,ff_p,cam_pr,vel_err_p,vq_p,
                       r_err,t_vel_r,ff_r,cam_rr,vel_err_r,vq_r
  $FOC  → 4  giá trị: angle_elec_p,angle_elec_r, angle_off_p,angle_off_r [rad]
"""

import numpy as np
from collections import deque
import threading

# Số điểm dữ liệu lưu trong buffer (50Hz × 20s = 1000 điểm)
BUFFER_SIZE = 1000


class GimbalDataStore:
    """Thread-safe ring buffer cho tất cả kênh dữ liệu Gimbal."""

    def __init__(self, size: int = BUFFER_SIZE):
        self.size = size
        self._lock = threading.Lock()

        # --- Timestamp ---
        self.time = deque(maxlen=size)

        # --- $IMU: Raw sensor (IMU Frame) ---
        self.imu_f_ax = deque(maxlen=size)
        self.imu_f_ay = deque(maxlen=size)
        self.imu_f_az = deque(maxlen=size)
        self.imu_f_gx = deque(maxlen=size)
        self.imu_f_gy = deque(maxlen=size)
        self.imu_f_gz = deque(maxlen=size)

        # --- $IMU: Raw sensor (IMU Payload/Camera) ---
        self.imu_p_ax = deque(maxlen=size)
        self.imu_p_ay = deque(maxlen=size)
        self.imu_p_az = deque(maxlen=size)
        self.imu_p_gx = deque(maxlen=size)
        self.imu_p_gy = deque(maxlen=size)
        self.imu_p_gz = deque(maxlen=size)

        # --- $ATT: Attitude Euler angles [deg] ---
        self.frame_pitch = deque(maxlen=size)
        self.frame_roll  = deque(maxlen=size)
        self.frame_yaw   = deque(maxlen=size)
        self.pay_pitch   = deque(maxlen=size)
        self.pay_roll    = deque(maxlen=size)
        self.pay_yaw     = deque(maxlen=size)
        self.rel_pitch   = deque(maxlen=size)
        self.rel_roll    = deque(maxlen=size)

        # --- $PID Pitch: Cascade PID ---
        self.p_err       = deque(maxlen=size)  # error góc pitch [deg]
        self.t_vel_p     = deque(maxlen=size)  # target velocity pitch [rad/s]
        self.ff_pitch    = deque(maxlen=size)  # feedforward pitch [rad/s]
        self.cam_pr      = deque(maxlen=size)  # camera pitch rate [rad/s]
        self.vel_err_p   = deque(maxlen=size)  # velocity error pitch [rad/s]
        self.vq_pitch    = deque(maxlen=size)  # Vq voltage pitch [V]

        # --- $PID Roll: Cascade PID ---
        self.r_err       = deque(maxlen=size)  # error góc roll [deg]
        self.t_vel_r     = deque(maxlen=size)  # target velocity roll [rad/s]
        self.ff_roll     = deque(maxlen=size)  # feedforward roll [rad/s]
        self.cam_rr      = deque(maxlen=size)  # camera roll rate [rad/s]
        self.vel_err_r   = deque(maxlen=size)  # velocity error roll [rad/s]
        self.vq_roll     = deque(maxlen=size)  # Vq voltage roll [V]

        # --- $FOC: FOC internal state [rad] ---
        self.elec_p      = deque(maxlen=size)  # angle_elec Pitch
        self.elec_r      = deque(maxlen=size)  # angle_elec Roll
        self.off_p       = deque(maxlen=size)  # angle_offset Pitch
        self.off_r       = deque(maxlen=size)  # angle_offset Roll

        self._t0 = None  # thời điểm frame đầu tiên

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _push_time(self, t: float):
        if self._t0 is None:
            self._t0 = t
        self.time.append(t - self._t0)

    # ------------------------------------------------------------------
    # Public: parse và append từ 1 dòng CSV string
    # ------------------------------------------------------------------
    def push_line(self, line: str, timestamp: float):
        """Parse 1 dòng CSV tagged và đẩy vào buffer (thread-safe)."""
        line = line.strip()
        if not line:
            return

        tag, _, body = line.partition(',')
        try:
            vals = [float(x) for x in body.split(',')]
        except ValueError:
            return  # dòng lỗi, bỏ qua

        with self._lock:
            if tag == '$IMU' and len(vals) >= 12:
                self._push_time(timestamp)
                (self.imu_f_ax, self.imu_f_ay, self.imu_f_az,
                 self.imu_f_gx, self.imu_f_gy, self.imu_f_gz,
                 self.imu_p_ax, self.imu_p_ay, self.imu_p_az,
                 self.imu_p_gx, self.imu_p_gy, self.imu_p_gz
                 ) = (q for q in (
                     self.imu_f_ax, self.imu_f_ay, self.imu_f_az,
                     self.imu_f_gx, self.imu_f_gy, self.imu_f_gz,
                     self.imu_p_ax, self.imu_p_ay, self.imu_p_az,
                     self.imu_p_gx, self.imu_p_gy, self.imu_p_gz))
                for q, v in zip(
                    [self.imu_f_ax, self.imu_f_ay, self.imu_f_az,
                     self.imu_f_gx, self.imu_f_gy, self.imu_f_gz,
                     self.imu_p_ax, self.imu_p_ay, self.imu_p_az,
                     self.imu_p_gx, self.imu_p_gy, self.imu_p_gz],
                    vals[:12]
                ):
                    q.append(v)

            elif tag == '$ATT' and len(vals) >= 8:
                for q, v in zip(
                    [self.frame_pitch, self.frame_roll, self.frame_yaw,
                     self.pay_pitch,   self.pay_roll,   self.pay_yaw,
                     self.rel_pitch,   self.rel_roll],
                    vals[:8]
                ):
                    q.append(v)

            elif tag == '$PID' and len(vals) >= 12:
                for q, v in zip(
                    [self.p_err, self.t_vel_p, self.ff_pitch, self.cam_pr,
                     self.vel_err_p, self.vq_pitch,
                     self.r_err, self.t_vel_r, self.ff_roll, self.cam_rr,
                     self.vel_err_r, self.vq_roll],
                    vals[:12]
                ):
                    q.append(v)

            elif tag == '$FOC' and len(vals) >= 4:
                for q, v in zip(
                    [self.elec_p, self.elec_r, self.off_p, self.off_r],
                    vals[:4]
                ):
                    q.append(v)

    # ------------------------------------------------------------------
    # Public: lấy numpy array (bản sao) để vẽ đồ thị
    # ------------------------------------------------------------------
    def get(self, channel: deque) -> np.ndarray:
        """Trả về bản sao numpy array của 1 channel (thread-safe)."""
        with self._lock:
            return np.array(channel, dtype=np.float32)

    def get_time(self) -> np.ndarray:
        return self.get(self.time)

    def push_demo(self, t: float):
        """Sinh dữ liệu giả lập để test GUI không cần phần cứng."""
        import math
        noise = lambda: (np.random.rand() - 0.5) * 0.1
        line_imu = (
            f"$IMU,"
            f"{math.sin(t)*0.1+noise():.3f},{math.cos(t)*0.05+noise():.3f},{9.8+noise():.3f},"
            f"{math.sin(t*2)*5+noise():.2f},{math.cos(t*3)*3+noise():.2f},{noise():.2f},"
            f"{math.sin(t)*0.12+noise():.3f},{math.cos(t)*0.06+noise():.3f},{9.8+noise():.3f},"
            f"{math.sin(t*2)*4.5+noise():.2f},{math.cos(t*3)*2.8+noise():.2f},{noise():.2f}"
        )
        p_deg = math.sin(t * 0.5) * 8.0
        r_deg = math.cos(t * 0.3) * 5.0
        line_att = (
            f"$ATT,"
            f"{math.sin(t*0.3)*3:.3f},{math.cos(t*0.2)*2:.3f},{t*5%360:.3f},"
            f"{p_deg:.3f},{r_deg:.3f},{t*4%360:.3f},"
            f"{p_deg*0.9:.3f},{r_deg*0.85:.3f}"
        )
        p_err = -p_deg
        r_err = -r_deg
        tv_p = p_err * 0.5
        tv_r = r_err * 0.5
        line_pid = (
            f"$PID,"
            f"{p_err:.3f},{tv_p:.3f},{math.sin(t)*0.2:.3f},"
            f"{tv_p*0.8:.3f},{tv_p*0.2:.3f},{tv_p*0.3:.3f},"
            f"{r_err:.3f},{tv_r:.3f},{math.cos(t)*0.15:.3f},"
            f"{tv_r*0.8:.3f},{tv_r*0.2:.3f},{tv_r*0.3:.3f}"
        )
        line_foc = (
            f"$FOC,"
            f"{(p_deg*0.0175*7)%6.283:.3f},{(r_deg*0.0175*7)%6.283:.3f},"
            f"{0.314:.3f},{0.271:.3f}"
        )
        for ln in [line_imu, line_att, line_pid, line_foc]:
            self.push_line(ln, t)
