# MPU6050 6-DoF IMU Library

Thư viện giao tiếp I2C cho cảm biến quán tính 6 trục **MPU6050** (3 trục Accelerometer + 3 trục Gyroscope).

## 🚀 Tính năng chính
- Giao tiếp qua I2C (hỗ trợ STM32 HAL).
- Hỗ trợ cấu hình địa chỉ thay đổi (`0x68` hoặc `0x69` dựa vào chân AD0) cho phép dùng **2 IMU** trên cùng 1 bus I2C.
- Tự động Calibrate (cân bằng Zero) Gyroscope lúc khởi động.
- Trích xuất dữ liệu Float chuẩn hóa (G và Độ/s).

---

## 📐 Kiến trúc bộ nhớ & I2C

```mermaid
graph TD
    subgraph STM32 I2C Master
        I2C_TX[I2C Transmit]
        I2C_RX[I2C Receive]
    end

    subgraph MPU6050 Registers
        PWR[PWR_MGMT_1\nWake up]
        CFG_G[GYRO_CONFIG\nScale: ±500°/s]
        CFG_A[ACCEL_CONFIG\nScale: ±2g]
        
        DATA[Data Registers\n0x3B to 0x48]
    end

    I2C_TX -->|Init Commands| PWR
    I2C_TX -->|Config| CFG_G
    I2C_TX -->|Config| CFG_A
    
    I2C_TX -->|Request Burst Read| DATA
    DATA -->|14 bytes| I2C_RX
    
    subgraph Parsing Logic
        I2C_RX -->|int16_t| RAW[Raw Data]
        RAW -->|Subtract Bias & Multiply Scale| FLOAT[Float Data\nAccel(g), Gyro(°/s)]
    end
```

---

## 🛠 Hướng dẫn sử dụng

### 1. Khởi tạo và Calibration

Khởi tạo cảm biến và thực hiện hiệu chuẩn (nên để cảm biến nằm yên hoàn toàn trong quá trình này):

```c
#include "mpu6050.h"

MPU6050_Handle_t imu_frame;
MPU6050_Handle_t imu_camera;

// Khởi tạo IMU 1 (GND nối chân AD0 -> Địa chỉ 0x68)
MPU6050_Init(&imu_frame, &hi2c3, MPU6050_ADDR_LOW);

// Khởi tạo IMU 2 (VCC nối chân AD0 -> Địa chỉ 0x69)
MPU6050_Init(&imu_camera, &hi2c3, MPU6050_ADDR_HIGH);

// Calibrate Gyro (Lấy 500 mẫu để tìm Bias)
MPU6050_CalibrateGyro(&imu_frame, 500);
MPU6050_CalibrateGyro(&imu_camera, 500);
```

### 2. Đọc dữ liệu liên tục

Trong vòng lặp hoặc ngắt Timer:

```c
// Lệnh này đọc 14 byte từ MPU6050 qua I2C bằng chế độ Burst (nhanh)
if (MPU6050_ReadAll(&imu_frame) == MPU6050_OK) {
    // Gia tốc (Đơn vị: G, 1G = 9.81m/s2)
    float ax = imu_frame.accel_x;
    float ay = imu_frame.accel_y;
    float az = imu_frame.accel_z;
    
    // Vận tốc góc (Đơn vị: Độ/s)
    float gx = imu_frame.gyro_x;
    float gy = imu_frame.gyro_y;
    float gz = imu_frame.gyro_z;
    
    // Nhiệt độ
    float temp = imu_frame.temperature;
}
```

### 3. Lưu ý về 2 IMU
Nếu sử dụng 2 IMU trên cùng một đường I2C (như I2C3), một cảm biến phải có chân `AD0 = GND` và cảm biến kia phải có `AD0 = 3.3V`. Nếu không, sẽ xảy ra xung đột địa chỉ I2C.
