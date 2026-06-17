# IMU Filter Library

Thư viện **IMU Filter** cung cấp các thuật toán lọc tín hiệu cảm biến không gian để hợp nhất dữ liệu từ Gia tốc kế (Accelerometer) và Con quay hồi chuyển (Gyroscope).

## 🚀 Các bộ lọc được hỗ trợ
1. **Complementary Filter (1 Trục):** Nhẹ, cực nhanh, tốn ít tài nguyên.
2. **Kalman Filter (1 Trục):** Khử nhiễu xuất sắc, tự động bù bias của gyro.
3. **Mahony AHRS (3 Trục - Quaternion):** Tránh hiện tượng Gimbal Lock, bù drift hiệu quả, lý tưởng cho UAV và Gimbal 3D.

---

## 📐 Kiến trúc thuật toán

```mermaid
graph TD
    subgraph Raw Sensor Data
        GYRO[Gyroscope\nRate rad/s]
        ACCEL[Accelerometer\nm/s² or g]
    end

    subgraph Mahony AHRS Core
        INT[Gyro Integration\nQuaternion Update]
        NORM1[Normalize Accel]
        EST[Estimate Gravity\nFrom Quaternion]
        ERR[Error = Cross Product\n(Accel, Est_Gravity)]
        
        PI[PI Controller\nKp, Ki]
        
        ACCEL --> NORM1
        NORM1 --> ERR
        EST --> ERR
        ERR --> PI
        PI -->|Feedback to correct Gyro bias| INT
        GYRO --> INT
        
        INT --> Q[Updated Quaternion]
        Q --> EST
    end

    subgraph Output Conversion
        Q --> EULER[Euler Angles\nRoll, Pitch, Yaw]
    end
```

---

## 🛠 Hướng dẫn sử dụng (Ví dụ Mahony)

### 1. Khởi tạo
```c
#include "imu_filter.h"

MahonyFilter_t ahrs;

// Khởi tạo Mahony với Kp = 1.5 (tốc độ hội tụ) và Ki = 0.005 (khử drift)
Mahony_Init(&ahrs, 1.5f, 0.005f);
```

### 2. Cập nhật định kỳ
Bắt buộc phải cập nhật định kỳ (ví dụ mỗi 10ms) với `dt` chính xác:

```c
// Lấy dữ liệu thô từ MPU6050
float gx = imu.gyro_x * PI / 180.0f; // Chuyển đổi sang rad/s
float gy = imu.gyro_y * PI / 180.0f;
float gz = imu.gyro_z * PI / 180.0f;
float ax = imu.accel_x; // Gia tốc có thể giữ nguyên (sẽ bị normalize)
float ay = imu.accel_y;
float az = imu.accel_z;

// dt = 0.01s (10ms)
Mahony_Update(&ahrs, gx, gy, gz, ax, ay, az, 0.01f);
```

### 3. Đọc dữ liệu
```c
// Dữ liệu đầu ra tính bằng Radian
float pitch_rad = ahrs.pitch;
float roll_rad = ahrs.roll;
float yaw_rad = ahrs.yaw;
```
