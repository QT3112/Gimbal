# Attitude Estimator Library (2-IMU Gimbal)

Thư viện **Attitude** cung cấp giải pháp ước lượng tư thế không gian (Euler Angles) chuyên dụng cho hệ thống Gimbal dựa trên **2 IMU** (không sử dụng encoder).

## 🚀 Tính năng chính
- Tích hợp 2 bộ lọc **Mahony AHRS** cho:
  - **IMU Frame** (gắn trên khung drone)
  - **IMU Payload** (gắn trên camera)
- Trích xuất góc tuyệt đối của camera để làm **Feedback** cho vòng PID góc.
- Cung cấp tốc độ góc khung (Frame Rate) để làm **Feedforward** triệt tiêu nhiễu rung lắc từ drone.
- Tính toán trực tiếp **Góc Điện (Electrical Angle)** cho thuật toán FOC (thay thế chức năng của Encoder).

---

## 📐 Kiến trúc luồng dữ liệu (Data Flow)

Sử dụng biểu đồ dưới đây để hiểu cách dữ liệu IMU đi qua thư viện:

```mermaid
graph TD
    subgraph Sensors
        IF[IMU Frame raw data\n(Accel + Gyro)]
        IP[IMU Payload raw data\n(Accel + Gyro)]
    end

    subgraph Attitude Library
        M_F[Mahony Filter Frame]
        M_P[Mahony Filter Payload]
        
        IF -->|dt| M_F
        IP -->|dt| M_P
        
        M_F --> FR[Frame Euler Angles\n& Raw Rates]
        M_P --> PR[Payload Euler Angles\n& Raw Rates]
        
        PR --> EA[Electrical Angle Calculation]
    end

    subgraph Control Loop
        FR -.->|Frame Rate Y| FF[Feedforward Disturbance Rejection]
        PR -.->|Payload Pitch| PID_A[Outer PID Angle Feedback]
        PR -.->|Payload Rate Y| PID_V[Inner PID Velocity Feedback]
        EA -.->|Angle Elec| FOC[FOC InvPark Transformation]
    end
```

---

## 🛠 Hướng dẫn sử dụng

### 1. Khởi tạo

Khai báo biến handle toàn cục và gọi hàm khởi tạo `Attitude_Init`:

```c
#include "attitude.h"

Attitude_Handle_t att;

void App_Init(void) {
    // Khởi tạo 2 bộ lọc Mahony với Kp = 2.0, Ki = 0.005
    Attitude_Init(&att, 2.0f, 0.005f);
}
```

### 2. Cập nhật định kỳ (Update Loop)

Cập nhật dữ liệu từ 2 IMU vào bộ ước lượng (thường đặt trong ngắt Timer, ví dụ `TIM6` với `dt = 0.01s`):

```c
void TIM6_ISR(void) {
    // Đọc raw data từ 2 MPU6050
    MPU6050_ReadAll(&imu_frame);
    MPU6050_ReadAll(&imu_payload);
    
    // Đổi gyro ra rad/s, accel ra g
    // ... (Code chuyển đổi đơn vị)

    // Cập nhật thư viện
    Attitude_Update(&att,
                    f_gx, f_gy, f_gz, f_ax, f_ay, f_az, // Frame IMU
                    p_gx, p_gy, p_gz, p_ax, p_ay, p_az, // Payload IMU
                    0.01f); // dt = 10ms
}
```

### 3. Tích hợp vào thuật toán FOC PID

Trích xuất các giá trị cần thiết từ cấu trúc `Attitude_Handle_t`:

```c
// 1. Lấy góc camera làm Feedback cho PID Góc (Outer Loop)
float camera_pitch = Attitude_GetPayloadPitch(&att);
float target_vel = FOC_PID_Update(&pid_angle, target_pitch - camera_pitch, 0.01f);

// 2. Feedforward bù nhiễu từ khung (Tùy chọn)
float frame_rate = Attitude_GetFramePitchRate(&att);
target_vel += K_feedforward * frame_rate;

// 3. FOC Control (Inner Loop) thay thế Encoder bằng Attitude
float elec_angle = Attitude_GetElecAngle(&att, pole_pairs, angle_offset);
// Truyền góc camera và target_vel vào FOC
FOC_RunVelocity(&foc, camera_pitch, target_vel); 
```

---

## ⚠️ Lưu ý quan trọng
- **Đơn vị đầu vào**: `Attitude_Update` yêu cầu con quay hồi chuyển (Gyro) ở đơn vị **Radian/s**.
- **Chiều trục (Axis alignment)**: Phải đảm bảo quy ước trục của 2 IMU khớp với chuyển động vật lý (X, Y, Z). Nếu gắn ngược, phải đổi dấu tham số trước khi truyền vào `Attitude_Update`.
