# Kế hoạch GUI Debug — Gimbal STM32G431 + 2×MPU6050

## Tổng quan hệ thống

Sau khi phân tích toàn bộ mã nguồn, hệ thống gimbal có kiến trúc điều khiển **cascade 2 vòng**:

```
[Disturbance]
     │
     ▼
[IMU Frame]──► Mahony ──► q_frame ──► Feedforward Rate
                                            │
[IMU Payload]─► Mahony ──► q_payload ──►  ▼
                                  q_error = q_frame* ⊗ q_payload
                                       │
                              ┌────────▼────────────────┐
                              │    OUTER PID (Angle)    │  pid_pitch / pid_roll
                              │  error = 0 - pitch_abs  │  Kp, Ki, Kd
                              └────────┬────────────────┘
                                       │ target_vel_p/r
                                       ▼
                           target_vel + feedforward → vel_error
                              ┌────────▼────────────────┐
                              │    INNER PID (Velocity) │  foc.pid_vel / foc_roll.pid_vel
                              │  error = vel_err        │  Kp, Ki, Kd, LPF α
                              └────────┬────────────────┘
                                       │ Vq (Voltage)
                                       ▼
                              ┌────────────────────────┐
                              │ FOC (SVPWM)            │
                              │ angle_elec = rel * pp  │
                              │ InvPark → InvClarke    │
                              │ → PWM TIM2/TIM3        │
                              └────────────────────────┘
```

---

## Phân tích các giá trị cần quan sát (Debug Variables)

### 🔵 Nhóm 1 — Dữ liệu thô IMU (Raw Sensor)

| Biến | Nguồn | Đơn vị | Mô tả |
|------|--------|---------|-------|
| `imu_frame.accel_x/y/z` | `MPU6050_Handle_t` | m/s² | Gia tốc thô của IMU khung |
| `imu_frame.gyro_x/y/z` | `MPU6050_Handle_t` | °/s | Tốc độ góc thô của IMU khung |
| `imu_payload.accel_x/y/z` | `MPU6050_Handle_t` | m/s² | Gia tốc thô của IMU payload/camera |
| `imu_payload.gyro_x/y/z` | `MPU6050_Handle_t` | °/s | Tốc độ góc thô của IMU payload |
| `imu_frame.temp_c` | `MPU6050_Handle_t` | °C | Nhiệt độ nội IMU Frame |
| `imu_payload.temp_c` | `MPU6050_Handle_t` | °C | Nhiệt độ nội IMU Payload |

> **Tại sao quan trọng:** Phát hiện nhiễu sensor, sai lệch mounting, hoặc nhiễu điện từ EMI từ các cuộn dây motor.

---

### 🟢 Nhóm 2 — Mahony Filter & Attitude Estimation

| Biến | Nguồn | Đơn vị | Mô tả |
|------|--------|---------|-------|
| `att.frame_pitch/roll/yaw` | `Attitude_Handle_t` | rad | Góc Euler tuyệt đối của khung drone |
| `att.payload_pitch/roll/yaw` | `Attitude_Handle_t` | rad | Góc Euler tuyệt đối của camera |
| `att.q_error[4]` | `Attitude_Handle_t` | - | Quaternion sai số giữa frame và payload |
| `att.relative_pitch` | `Attitude_Handle_t` | rad | Góc cơ học motor Pitch (thay encoder) |
| `att.relative_roll` | `Attitude_Handle_t` | rad | Góc cơ học motor Roll (thay encoder) |
| `att.frame_rate_x/y/z` | `Attitude_Handle_t` | rad/s | Angular rate frame (feedforward input) |
| `att.payload_rate_x/y/z` | `Attitude_Handle_t` | rad/s | Angular rate camera (inner loop feedback) |

> **Tại sao quan trọng:** Mahony hội tụ sai → toàn bộ PID hoạt động trên dữ liệu sai. Quan sát được `payload_pitch` vs `relative_pitch` để kiểm tra quaternion pipeline.

---

### 🟡 Nhóm 3 — Vòng điều khiển PITCH (Outer + Inner Loop)

| Biến | Nguồn trong `main.c` | Đơn vị | Mô tả |
|------|----------------------|---------|-------|
| `pitch_abs` | `Attitude_GetPayloadPitch()` | rad | Góc Pitch tuyệt đối → feedback Outer PID |
| `pitch_err` | `0.0 - pitch_abs` | rad | Sai số góc Pitch → input Outer PID |
| `target_vel_p` | `FOC_PID_Update(&pid_pitch, ...)` | rad/s | Setpoint vận tốc từ Outer PID |
| `frame_pitch_rate` | `Attitude_GetFramePitchRate()` | rad/s | Rate khung → feedforward |
| `ff_pitch` | `K_ff * frame_pitch_rate` | rad/s | Feedforward term Pitch |
| `cam_pitch_rate` | `Attitude_GetPayloadPitchRate()` | rad/s | Vận tốc thực camera → feedback Inner PID |
| `vel_err_pitch` | `(target_vel_p + ff_pitch) - cam_pitch_rate` | rad/s | Sai số vận tốc Pitch → input Inner PID |
| `Vq_pitch` | `FOC_PID_Update(&foc.pid_vel, ...)` | V | Điện áp trục Q motor Pitch → output Inner PID |
| `foc.angle_elec` | `Attitude_GetElecAnglePitchRel()` | rad | Góc điện → vào FOC SVPWM Pitch |

**Tham số PID Pitch (Outer):**
- `pid_pitch.Kp = 2.0`, `pid_pitch.Ki = 0.0`, `pid_pitch.Kd = 0.0`
- Output clamp: `[-10.0, +10.0]` rad/s

**Tham số PID Velocity Pitch (Inner):**
- `foc.pid_vel.Kp = 0.1`, `foc.pid_vel.Ki = 0.01`, `foc.pid_vel.Kd = 0.0`
- Output clamp: `[-voltage_limit, +voltage_limit]`
- LPF alpha: `0.98`

---

### 🟠 Nhóm 4 — Vòng điều khiển ROLL (Outer + Inner Loop)

| Biến | Nguồn trong `main.c` | Đơn vị | Mô tả |
|------|----------------------|---------|-------|
| `roll_abs` | `Attitude_GetPayloadRoll()` | rad | Góc Roll tuyệt đối → feedback Outer PID |
| `roll_err` | `0.0 - roll_abs` | rad | Sai số góc Roll |
| `target_vel_r` | `FOC_PID_Update(&pid_roll, ...)` | rad/s | Setpoint vận tốc từ Outer PID |
| `frame_roll_rate` | `Attitude_GetFrameRollRate()` | rad/s | Rate khung → feedforward |
| `ff_roll` | `K_ff * frame_roll_rate` | rad/s | Feedforward term Roll |
| `cam_roll_rate` | `Attitude_GetPayloadRollRate()` | rad/s | Vận tốc thực camera → feedback Inner PID |
| `vel_err_roll` | `(target_vel_r + ff_roll) - cam_roll_rate` | rad/s | Sai số vận tốc Roll |
| `Vq_roll` | `FOC_PID_Update(&foc_roll.pid_vel, ...)` | V | Điện áp trục Q motor Roll |
| `foc_roll.angle_elec` | `Attitude_GetElecAngleRollRel()` | rad | Góc điện → vào FOC SVPWM Roll |

**Tham số PID Roll (Outer):**
- `pid_roll.Kp = 2.2`, `pid_roll.Ki = 0.2`, `pid_roll.Kd = 0.0`
- Output clamp: `[-10.0, +10.0]` rad/s

**Tham số PID Velocity Roll (Inner):**
- `foc_roll.pid_vel.Kp = 0.1`, `foc_roll.pid_vel.Ki = 0.01`, `foc_roll.pid_vel.Kd = 0.0`
- Output clamp: `[-voltage_limit, +voltage_limit]`
- LPF alpha: `0.98`

---

### 🔴 Nhóm 5 — FOC Internal State

| Biến | Nguồn | Đơn vị | Mô tả |
|------|--------|---------|-------|
| `foc.Vd_ref / Vq_ref` | `FOC_Handle_t` | V | Điện áp đặt trục D và Q (Pitch) |
| `foc_roll.Vd_ref / Vq_ref` | `FOC_Handle_t` | V | Điện áp đặt trục D và Q (Roll) |
| `foc.V_ab.alpha / beta` | `FOC_Handle_t` | V | Điện áp sau Inverse Park (αβ frame) |
| `foc.angle_mech` | `FOC_Handle_t` | rad | Góc cơ học (hiện không dùng encoder) |
| `foc.angle_offset` | `FOC_Handle_t` | rad | Offset góc điện sau Homing |

---

## Kế hoạch xây dựng GUI

### Lựa chọn công nghệ

**→ Python + PyQt6 + pyqtgraph** là lựa chọn tối ưu cho dự án này.

| Tiêu chí | Python+PyQt6+pyqtgraph | Python+Tkinter | Electron (Web) |
|----------|------------------------|----------------|----------------|
| Hiệu suất vẽ đồ thị real-time | ⭐⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| Serial port (pyserial) | ✅ Native | ✅ | ⚠️ Cần bridge |
| Cài đặt dễ dàng | ✅ pip | ✅ built-in | ❌ Node.js |
| Giao diện đẹp | ⭐⭐⭐⭐ | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| Cross-platform | ✅ | ✅ | ✅ |

> **Lý do chọn:** `pyqtgraph` có thể vẽ **hàng nghìn điểm/giây** với OpenGL backend — đủ để theo dõi real-time ở 100Hz mà không bị giật.

---

### Kiến trúc phần mềm GUI

```
GUI_Debug/
├── main.py                  # Entry point
├── requirements.txt         # Danh sách thư viện
├── serial_reader.py         # Thread đọc serial + parse dữ liệu
├── data_store.py            # Ring buffer lưu dữ liệu real-time
└── ui/
    ├── main_window.py       # Cửa sổ chính (QMainWindow)
    ├── panel_imu.py         # Tab: Raw IMU data
    ├── panel_attitude.py    # Tab: Attitude (Euler angles)
    ├── panel_pid_pitch.py   # Tab: PID Pitch (Outer + Inner)
    ├── panel_pid_roll.py    # Tab: PID Roll (Outer + Inner)
    └── panel_foc.py         # Tab: FOC state (Vq, angle_elec)
```

---

### Giao thức truyền dữ liệu STM32 → PC

Giữ nguyên `printf` hiện tại (CDC USB), nhưng **chuẩn hóa format** để dễ parse.

**Format đề xuất (CSV dạng tagged):**
```
$IMU,ax1,ay1,az1,gx1,gy1,gz1,ax2,ay2,az2,gx2,gy2,gz2\r\n
$ATT,frame_pitch,frame_roll,frame_yaw,pay_pitch,pay_roll,pay_yaw,rel_pitch,rel_roll\r\n
$PID,pitch_err,target_vel_p,cam_pitch_rate,vel_err_p,Vq_pitch,roll_err,target_vel_r,cam_roll_rate,vel_err_r,Vq_roll\r\n
```

Ví dụ dòng thực tế:
```
$PID,-0.12,0.24,0.19,0.05,0.21,-0.03,0.06,0.04,0.02,0.18
```

---

### Bố cục giao diện GUI

```
┌──────────────────────────────────────────────────────────────┐
│  🎯 Gimbal Debug Monitor                    ⚙️  PITCH ROLL  │
├──────────────┬───────────────────────────────────────────────┤
│ Serial Port: │ [COM3 ▼] [115200 ▼]  [🔗 Connect] [● Record] │
│ Status: 🟢   │ Latency: 12ms | Frame rate: 100Hz            │
├──────────────┴───────────────────────────────────────────────┤
│ [IMU Raw] [Attitude] [PID Pitch] [PID Roll] [FOC] [3D View] │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ████████████████  Attitude Plot  ████████████████████████  │
│  pitch_abs ────────────────────────────────────────────── ○ │
│  roll_abs  ────────────────────────────────────────────── ○ │
│  rel_pitch ─────────────────────────────────────────────  ○ │
│                                                              │
│  ████████████████  PID Pitch  ██████████████████████████  │
│  pitch_err  ───────── target_vel ────── cam_rate ──── Vq  │
│                                                              │
├──────────────────────────────────────────────────────────────┤
│  PID Params (Live tuning)                                    │
│  PITCH Outer: Kp [___] Ki [___] Kd [___] [Apply]           │
│  PITCH Inner: Kp [___] Ki [___] Kd [___] [Apply]           │
│  ROLL  Outer: Kp [___] Ki [___] Kd [___] [Apply]           │
│  ROLL  Inner: Kp [___] Ki [___] Kd [___] [Apply]           │
└──────────────────────────────────────────────────────────────┘
```

---

### Tính năng GUI (chi tiết)

#### Tab 1: Raw IMU
- 6 đồ thị: Accel X/Y/Z và Gyro X/Y/Z cho **cả 2 IMU**
- Toggle hiển thị Frame vs Payload
- Chỉ thị nhiệt độ sensor

#### Tab 2: Attitude (Euler Angles)
- Đồ thị **payload_pitch, payload_roll, payload_yaw** (tuyệt đối)
- Đồ thị **frame_pitch, frame_roll, frame_yaw** (khung drone)
- Đồ thị **relative_pitch, relative_roll** (góc cơ học thực tế)
- Thanh gauge trực quan hiển thị góc hiện tại dạng kim đồng hồ
- Visualizer 3D đơn giản (không gian 3 trục) — optional

#### Tab 3: PID Pitch (Cascade)
- **Outer PID:** `pitch_err` → `target_vel_p` (hiển thị Kp, Ki, Kd)
- **Feedforward:** `frame_pitch_rate`, `ff_pitch`
- **Inner PID:** `vel_err_pitch` → `Vq_pitch`
- **Điểm setpoint** = 0 (đường tham chiếu ngang)

#### Tab 4: PID Roll (Cascade)
- Tương tự Tab 3 nhưng cho trục Roll

#### Tab 5: FOC State
- `angle_elec` (Pitch và Roll) — theo dõi góc điện
- `Vq_pitch`, `Vq_roll` — điện áp thực tế
- `foc.angle_offset`, `foc_roll.angle_offset` — xác nhận Homing
- Duty cycle PWM ước tính

#### Tính năng phụ
- **Record to CSV:** Lưu toàn bộ log thành file CSV để phân tích offline
- **Live PID Tuning:** Gửi command ngược lại STM32 qua UART để chỉnh Kp/Ki/Kd hot (cần thêm parser bên MCU)
- **Freeze / Zoom:** Dừng scroll để phân tích 1 khoảng thời gian
- **Buffer size:** Cấu hình được (ví dụ: 5s / 10s / 30s dữ liệu gần nhất)

---

## Yêu cầu cài đặt

### Trên máy tính (Linux/Windows/macOS)

```bash
# Python 3.10+
python --version

# Cài thư viện
pip install pyqt6 pyqtgraph pyserial numpy
```

**Danh sách packages:**

| Package | Phiên bản khuyến nghị | Vai trò |
|---------|-----------------------|---------|
| `python` | ≥ 3.10 | Runtime |
| `PyQt6` | ≥ 6.5 | Framework GUI |
| `pyqtgraph` | ≥ 0.13 | Đồ thị real-time (OpenGL) |
| `pyserial` | ≥ 3.5 | Đọc cổng serial USB CDC |
| `numpy` | ≥ 1.24 | Tính toán dữ liệu, ring buffer |

**Optional (nếu muốn 3D View):**
```bash
pip install PyOpenGL PyOpenGL_accelerate
```

### Trên STM32 (MCU) — thay đổi nhỏ

Chỉ cần sửa format `printf` trong `main.c` từ dạng chuỗi đọc người sang **CSV tagged** như đề xuất ở trên. Không cần thay đổi logic điều khiển.

---

## Thứ tự phát triển (Roadmap)

```
Phase 1: Serial Parser + Data Store          (1 ngày)
  ├── serial_reader.py: Thread đọc, parse CSV
  └── data_store.py: Ring buffer numpy

Phase 2: UI cơ bản + Attitude Tab            (1 ngày)
  ├── main_window.py: Khung chính, tab widget
  └── panel_attitude.py: 3 đồ thị real-time

Phase 3: PID Panels                          (1 ngày)
  ├── panel_pid_pitch.py
  └── panel_pid_roll.py

Phase 4: IMU Raw + FOC Panel                 (0.5 ngày)
  ├── panel_imu.py
  └── panel_foc.py

Phase 5: Polish + Record + Export            (0.5 ngày)
  ├── Record to CSV
  └── Connect/Disconnect logic
```

---

## Quyết định đã xác nhận ✅

| # | Câu hỏi | Quyết định |
|---|---------|------------|
| 1 | Format printf | **CSV tagged** (`$IMU`, `$ATT`, `$PID`, `$FOC`) |
| 2 | Tần số gửi | **20ms (50Hz)** — mỗi 2 vòng lặp 10ms |
| 3 | Live PID Tuning | **Để sau** — Phase 2 |
| 4 | 3D Visualizer | **Để sau** — Phase 2 |
