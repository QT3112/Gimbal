/**
 ******************************************************************************
 * @file    mpu6050.h
 * @brief   Thư viện giao tiếp với cảm biến MPU6050 qua I2C (STM32 HAL)
 *
 * Hỗ trợ:
 *  - Đọc gia tốc kế (Accelerometer) 3 trục
 *  - Đọc con quay hồi chuyển (Gyroscope) 3 trục
 *  - Đọc nhiệt độ nội
 *  - Cấu hình thang đo (Full-Scale Range)
 *  - Cấu hình bộ lọc số (Digital Low-Pass Filter)
 *  - Kiểm tra kết nối qua WHO_AM_I register
 *
 * Cách sử dụng:
 *  1. Khai báo: MPU6050_Handle_t imu;
 *  2. Khởi tạo: MPU6050_Init(&imu, &hi2c1, MPU6050_ADDR_LOW);
 *  3. Đọc dữ liệu: MPU6050_ReadAll(&imu);
 *  4. Truy cập: imu.accel_x, imu.gyro_x, ...
 ******************************************************************************
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ===========================================================================
 * Địa chỉ I2C của MPU6050
 * AD0 = GND (nối đất): ADDR_LOW  (0x68)
 * AD0 = VCC (nối 3.3V): ADDR_HIGH (0x69)
 * =========================================================================== */
#define MPU6050_ADDR_LOW    (0x68 << 1)   /*!< Địa chỉ khi chân AD0 = GND */
#define MPU6050_ADDR_HIGH   (0x69 << 1)   /*!< Địa chỉ khi chân AD0 = VCC */

/* ===========================================================================
 * Địa chỉ thanh ghi (Registers)
 * =========================================================================== */
#define MPU6050_REG_SMPLRT_DIV       0x19  /*!< Sample Rate Divider */
#define MPU6050_REG_CONFIG           0x1A  /*!< Cấu hình DLPF và FSYNC */
#define MPU6050_REG_GYRO_CONFIG      0x1B  /*!< Cấu hình thang đo Gyroscope */
#define MPU6050_REG_ACCEL_CONFIG     0x1C  /*!< Cấu hình thang đo Accelerometer */
#define MPU6050_REG_ACCEL_XOUT_H     0x3B  /*!< Dữ liệu Accel X (byte cao) */
#define MPU6050_REG_ACCEL_XOUT_L     0x3C  /*!< Dữ liệu Accel X (byte thấp) */
#define MPU6050_REG_ACCEL_YOUT_H     0x3D
#define MPU6050_REG_ACCEL_YOUT_L     0x3E
#define MPU6050_REG_ACCEL_ZOUT_H     0x3F
#define MPU6050_REG_ACCEL_ZOUT_L     0x40
#define MPU6050_REG_TEMP_OUT_H       0x41  /*!< Dữ liệu nhiệt độ (byte cao) */
#define MPU6050_REG_TEMP_OUT_L       0x42  /*!< Dữ liệu nhiệt độ (byte thấp) */
#define MPU6050_REG_GYRO_XOUT_H      0x43  /*!< Dữ liệu Gyro X (byte cao) */
#define MPU6050_REG_GYRO_XOUT_L      0x44
#define MPU6050_REG_GYRO_YOUT_H      0x45
#define MPU6050_REG_GYRO_YOUT_L      0x46
#define MPU6050_REG_GYRO_ZOUT_H      0x47
#define MPU6050_REG_GYRO_ZOUT_L      0x48
#define MPU6050_REG_PWR_MGMT_1       0x6B  /*!< Quản lý nguồn (wake-up) */
#define MPU6050_REG_WHO_AM_I         0x75  /*!< Định danh chip, mong đợi = 0x68 */

/* ===========================================================================
 * Thang đo Accelerometer (Full-Scale Range)
 * =========================================================================== */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0x00,  /*!< ±2g  | LSB sensitivity = 16384 LSB/g */
    MPU6050_ACCEL_FS_4G  = 0x08,  /*!< ±4g  | LSB sensitivity =  8192 LSB/g */
    MPU6050_ACCEL_FS_8G  = 0x10,  /*!< ±8g  | LSB sensitivity =  4096 LSB/g */
    MPU6050_ACCEL_FS_16G = 0x18,  /*!< ±16g | LSB sensitivity =  2048 LSB/g */
} MPU6050_AccelFS_t;

/* ===========================================================================
 * Thang đo Gyroscope (Full-Scale Range)
 * =========================================================================== */
typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0x00, /*!< ±250 °/s  | sensitivity = 131.0 LSB/(°/s) */
    MPU6050_GYRO_FS_500DPS  = 0x08, /*!< ±500 °/s  | sensitivity =  65.5 LSB/(°/s) */
    MPU6050_GYRO_FS_1000DPS = 0x10, /*!< ±1000 °/s | sensitivity =  32.8 LSB/(°/s) */
    MPU6050_GYRO_FS_2000DPS = 0x18, /*!< ±2000 °/s | sensitivity =  16.4 LSB/(°/s) */
} MPU6050_GyroFS_t;

/* ===========================================================================
 * Bộ lọc thông thấp số (DLPF - Digital Low-Pass Filter)
 * Tần số cắt càng thấp thì càng lọc nhiều nhiễu nhưng độ trễ cao hơn
 * =========================================================================== */
typedef enum {
    MPU6050_DLPF_256HZ = 0x00,  /*!< Bandwidth Accel: 260Hz | Gyro: 256Hz */
    MPU6050_DLPF_188HZ = 0x01,  /*!< Bandwidth Accel: 184Hz | Gyro: 188Hz */
    MPU6050_DLPF_98HZ  = 0x02,  /*!< Bandwidth Accel: 94Hz  | Gyro: 98Hz  */
    MPU6050_DLPF_42HZ  = 0x03,  /*!< Bandwidth Accel: 44Hz  | Gyro: 42Hz  */
    MPU6050_DLPF_20HZ  = 0x04,  /*!< Bandwidth Accel: 21Hz  | Gyro: 20Hz  */
    MPU6050_DLPF_10HZ  = 0x05,  /*!< Bandwidth Accel: 10Hz  | Gyro: 10Hz  */
    MPU6050_DLPF_5HZ   = 0x06,  /*!< Bandwidth Accel: 5Hz   | Gyro: 5Hz   */
} MPU6050_DLPF_t;

/* ===========================================================================
 * Mã trạng thái trả về
 * =========================================================================== */
typedef enum {
    MPU6050_OK    = 0x00,  /*!< Thành công */
    MPU6050_ERROR = 0x01,  /*!< Lỗi giao tiếp I2C */
    MPU6050_BUSY  = 0x02,  /*!< Bus I2C đang bận */
    MPU6050_WRONG_DEVICE = 0x03, /*!< WHO_AM_I không khớp, sai thiết bị */
} MPU6050_Status_t;

/* ===========================================================================
 * Cấu trúc cấu hình khởi tạo
 * =========================================================================== */
typedef struct {
    MPU6050_AccelFS_t accel_fs;  /*!< Thang đo gia tốc kế */
    MPU6050_GyroFS_t  gyro_fs;   /*!< Thang đo con quay */
    MPU6050_DLPF_t    dlpf;      /*!< Cài đặt bộ lọc số */
    uint8_t           sample_rate_div; /*!< Chia tần số mẫu (0 = không chia = tốc độ cực đại) */
} MPU6050_Config_t;

/* ===========================================================================
 * Handle chính của thư viện (tương tự HAL_HandleTypeDef)
 * =========================================================================== */
typedef struct {
    /* --- Phần cứng --- */
    I2C_HandleTypeDef  *hi2c;       /*!< Con trỏ đến I2C handle của HAL */
    uint8_t             dev_addr;   /*!< Địa chỉ I2C (đã dịch 1 bit: <<1) */

    /* --- Cấu hình --- */
    MPU6050_Config_t    config;     /*!< Lưu trữ cấu hình hiện tại */

    /* --- Dữ liệu thô (raw) từ cảm biến --- */
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_temp;
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;

    /* --- Dữ liệu đã quy đổi sang đơn vị thực --- */
    float accel_x;   /*!< Gia tốc trục X [m/s²] */
    float accel_y;   /*!< Gia tốc trục Y [m/s²] */
    float accel_z;   /*!< Gia tốc trục Z [m/s²] */
    float temp_c;    /*!< Nhiệt độ [°C] */
    float gyro_x;    /*!< Vận tốc góc trục X [°/s] */
    float gyro_y;    /*!< Vận tốc góc trục Y [°/s] */
    float gyro_z;    /*!< Vận tốc góc trục Z [°/s] */

    /* --- Giá trị offset hiệu chỉnh (calibration bias) --- */
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
} MPU6050_Handle_t;

/* ===========================================================================
 * Khai báo hàm API công khai
 * =========================================================================== */

/**
 * @brief  Khởi tạo MPU6050 với cấu hình mặc định (±2g, ±250°/s, DLPF 42Hz)
 * @param  hdev     Con trỏ đến MPU6050_Handle_t
 * @param  hi2c     Con trỏ đến I2C_HandleTypeDef đã được CubeMX khởi tạo
 * @param  dev_addr Địa chỉ I2C: MPU6050_ADDR_LOW hoặc MPU6050_ADDR_HIGH
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c, uint8_t dev_addr);

/**
 * @brief  Khởi tạo MPU6050 với cấu hình tùy chỉnh
 * @param  hdev     Con trỏ đến MPU6050_Handle_t
 * @param  hi2c     Con trỏ đến I2C_HandleTypeDef
 * @param  dev_addr Địa chỉ I2C
 * @param  config   Con trỏ đến cấu trúc cấu hình MPU6050_Config_t
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_InitEx(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c,
                                 uint8_t dev_addr, const MPU6050_Config_t *config);

/**
 * @brief  Đọc toàn bộ dữ liệu (accel, gyro, nhiệt độ) trong 1 lần burst read
 * @note   Đây là cách đọc hiệu quả nhất: chỉ 1 giao dịch I2C để đọc 14 byte
 * @param  hdev  Con trỏ đến MPU6050_Handle_t
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadAll(MPU6050_Handle_t *hdev);

/**
 * @brief  Chỉ đọc dữ liệu gia tốc kế (6 byte)
 * @param  hdev  Con trỏ đến MPU6050_Handle_t
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadAccel(MPU6050_Handle_t *hdev);

/**
 * @brief  Chỉ đọc dữ liệu con quay hồi chuyển (6 byte)
 * @param  hdev  Con trỏ đến MPU6050_Handle_t
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadGyro(MPU6050_Handle_t *hdev);

/**
 * @brief  Chỉ đọc nhiệt độ nội (2 byte)
 * @param  hdev  Con trỏ đến MPU6050_Handle_t
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_ReadTemp(MPU6050_Handle_t *hdev);

/**
 * @brief  Hiệu chỉnh offset Gyroscope bằng cách tính trung bình khi đứng yên
 * @note   Đặt cảm biến ở vị trí cố định, không rung trong khi gọi hàm này.
 *         Hàm sẽ block trong khoảng (samples * 5ms)
 * @param  hdev     Con trỏ đến MPU6050_Handle_t
 * @param  samples  Số lần đo để lấy trung bình (khuyến nghị: 200~500)
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_Handle_t *hdev, uint16_t samples);

/**
 * @brief  Kiểm tra thiết bị có sẵn sàng không qua thanh ghi WHO_AM_I
 * @param  hdev  Con trỏ đến MPU6050_Handle_t
 * @retval MPU6050_Status_t (MPU6050_OK nếu thành công)
 */
MPU6050_Status_t MPU6050_IsReady(MPU6050_Handle_t *hdev);

/**
 * @brief  Đặt lại cảm biến về trạng thái ban đầu (Software Reset)
 * @param  hdev  Con trỏ đến MPU6050_Handle_t
 * @retval MPU6050_Status_t
 */
MPU6050_Status_t MPU6050_Reset(MPU6050_Handle_t *hdev);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
