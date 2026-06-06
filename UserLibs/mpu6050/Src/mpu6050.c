/**
 ******************************************************************************
 * @file    mpu6050.c
 * @brief   Thư viện giao tiếp với cảm biến MPU6050 qua I2C (STM32 HAL)
 ******************************************************************************
 */

#include "mpu6050.h"
#include <string.h>  /* memset */

/* ===========================================================================
 * Hệ số nhạy cảm theo datasheet (để quy đổi từ raw -> đơn vị thực)
 * =========================================================================== */
#define ACCEL_SENS_2G    16384.0f
#define ACCEL_SENS_4G     8192.0f
#define ACCEL_SENS_8G     4096.0f
#define ACCEL_SENS_16G    2048.0f

#define GYRO_SENS_250DPS  131.0f
#define GYRO_SENS_500DPS   65.5f
#define GYRO_SENS_1000DPS  32.8f
#define GYRO_SENS_2000DPS  16.4f

#define GRAVITY_M_S2      9.80665f  /* Gia tốc trọng trường [m/s²] */

/* Timeout I2C tính bằng ms */
#define I2C_TIMEOUT_MS    10U

/* ===========================================================================
 * Hàm nội bộ (static - không public ra bên ngoài)
 * =========================================================================== */

/**
 * @brief  Ghi 1 byte vào thanh ghi
 */
static MPU6050_Status_t MPU6050_WriteReg(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Write(hdev->hi2c,
                             hdev->dev_addr,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1,
                             I2C_TIMEOUT_MS);
    return (ret == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/**
 * @brief  Đọc 1 byte từ thanh ghi
 */
static MPU6050_Status_t MPU6050_ReadReg(MPU6050_Handle_t *hdev, uint8_t reg, uint8_t *value)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(hdev->hi2c,
                            hdev->dev_addr,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            value,
                            1,
                            I2C_TIMEOUT_MS);
    return (ret == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/**
 * @brief  Đọc nhiều byte liên tiếp (burst read)
 */
static MPU6050_Status_t MPU6050_ReadRegs(MPU6050_Handle_t *hdev, uint8_t reg,
                                          uint8_t *buf, uint16_t len)
{
    HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(hdev->hi2c,
                            hdev->dev_addr,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            I2C_TIMEOUT_MS);
    return (ret == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/**
 * @brief  Lấy hệ số nhạy cảm của Accelerometer dựa vào cài đặt thang đo
 */
static float MPU6050_GetAccelSens(MPU6050_AccelFS_t fs)
{
    switch (fs) {
        case MPU6050_ACCEL_FS_2G:  return ACCEL_SENS_2G;
        case MPU6050_ACCEL_FS_4G:  return ACCEL_SENS_4G;
        case MPU6050_ACCEL_FS_8G:  return ACCEL_SENS_8G;
        case MPU6050_ACCEL_FS_16G: return ACCEL_SENS_16G;
        default:                   return ACCEL_SENS_2G;
    }
}

/**
 * @brief  Lấy hệ số nhạy cảm của Gyroscope dựa vào cài đặt thang đo
 */
static float MPU6050_GetGyroSens(MPU6050_GyroFS_t fs)
{
    switch (fs) {
        case MPU6050_GYRO_FS_250DPS:  return GYRO_SENS_250DPS;
        case MPU6050_GYRO_FS_500DPS:  return GYRO_SENS_500DPS;
        case MPU6050_GYRO_FS_1000DPS: return GYRO_SENS_1000DPS;
        case MPU6050_GYRO_FS_2000DPS: return GYRO_SENS_2000DPS;
        default:                      return GYRO_SENS_250DPS;
    }
}

/**
 * @brief  Nội dung thực thi chung cho cả Init và InitEx
 */
static MPU6050_Status_t MPU6050_Configure(MPU6050_Handle_t *hdev)
{
    MPU6050_Status_t status;
    uint8_t who_am_i = 0;

    /* 1. Kiểm tra chip thông qua WHO_AM_I
     *   0x68 = MPU6050 chính hãng
     *   0x72 = MPU6050 revision B1
     *   0x70 = Clone chip phổ biến trên module GY-521 hàng Trung Quốc
     *   0x98 = Clone chip variant khác
     */
    status = MPU6050_ReadReg(hdev, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (status != MPU6050_OK) return MPU6050_ERROR;
    if (who_am_i != 0x68 && who_am_i != 0x72 &&
        who_am_i != 0x70 && who_am_i != 0x98) return MPU6050_WRONG_DEVICE;

    /* 2. Đánh thức chip (xóa bit SLEEP trong PWR_MGMT_1, chọn clock PLL từ Gyro X) */
    status = MPU6050_WriteReg(hdev, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (status != MPU6050_OK) return status;
    HAL_Delay(10); /* Chờ clock ổn định */

    /* 3. Cấu hình tần số lấy mẫu */
    status = MPU6050_WriteReg(hdev, MPU6050_REG_SMPLRT_DIV, hdev->config.sample_rate_div);
    if (status != MPU6050_OK) return status;

    /* 4. Cấu hình DLPF */
    status = MPU6050_WriteReg(hdev, MPU6050_REG_CONFIG, (uint8_t)hdev->config.dlpf);
    if (status != MPU6050_OK) return status;

    /* 5. Cấu hình thang đo Gyroscope */
    status = MPU6050_WriteReg(hdev, MPU6050_REG_GYRO_CONFIG, (uint8_t)hdev->config.gyro_fs);
    if (status != MPU6050_OK) return status;

    /* 6. Cấu hình thang đo Accelerometer */
    status = MPU6050_WriteReg(hdev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)hdev->config.accel_fs);
    if (status != MPU6050_OK) return status;

    return MPU6050_OK;
}

/* ===========================================================================
 * Hàm API công khai (implementation)
 * =========================================================================== */

/**
 * @brief  Khởi tạo MPU6050 với cấu hình mặc định
 */
MPU6050_Status_t MPU6050_Init(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c, uint8_t dev_addr)
{
    /* Cấu hình mặc định an toàn cho hầu hết ứng dụng */
    MPU6050_Config_t default_config = {
        .accel_fs        = MPU6050_ACCEL_FS_2G,
        .gyro_fs         = MPU6050_GYRO_FS_250DPS,
        .dlpf            = MPU6050_DLPF_42HZ,
        .sample_rate_div = 0,  /* Tốc độ lấy mẫu tối đa (1kHz khi DLPF bật) */
    };
    return MPU6050_InitEx(hdev, hi2c, dev_addr, &default_config);
}

/**
 * @brief  Khởi tạo MPU6050 với cấu hình tùy chỉnh
 */
MPU6050_Status_t MPU6050_InitEx(MPU6050_Handle_t *hdev, I2C_HandleTypeDef *hi2c,
                                  uint8_t dev_addr, const MPU6050_Config_t *config)
{
    if (hdev == NULL || hi2c == NULL || config == NULL) return MPU6050_ERROR;

    /* Xóa toàn bộ struct và thiết lập thông tin phần cứng */
    memset(hdev, 0, sizeof(MPU6050_Handle_t));
    hdev->hi2c     = hi2c;
    hdev->dev_addr = dev_addr;
    hdev->config   = *config;

    /* Reset offset về 0 */
    hdev->gyro_offset_x = 0.0f;
    hdev->gyro_offset_y = 0.0f;
    hdev->gyro_offset_z = 0.0f;

    return MPU6050_Configure(hdev);
}

/**
 * @brief  Đọc toàn bộ 14 byte dữ liệu (accel + temp + gyro) bằng burst read
 * @note   Đây là cách đọc nhanh nhất và nên dùng trong vòng lặp điều khiển
 */
MPU6050_Status_t MPU6050_ReadAll(MPU6050_Handle_t *hdev)
{
    uint8_t buf[14];
    MPU6050_Status_t status;
    float accel_sens, gyro_sens;

    /* Đọc 14 byte liên tiếp từ ACCEL_XOUT_H (0x3B) đến GYRO_ZOUT_L (0x48) */
    status = MPU6050_ReadRegs(hdev, MPU6050_REG_ACCEL_XOUT_H, buf, 14);
    if (status != MPU6050_OK) return status;

    /* Ghép byte cao + byte thấp thành số nguyên 16-bit có dấu */
    hdev->raw_accel_x = (int16_t)((buf[0]  << 8) | buf[1]);
    hdev->raw_accel_y = (int16_t)((buf[2]  << 8) | buf[3]);
    hdev->raw_accel_z = (int16_t)((buf[4]  << 8) | buf[5]);
    hdev->raw_temp    = (int16_t)((buf[6]  << 8) | buf[7]);
    hdev->raw_gyro_x  = (int16_t)((buf[8]  << 8) | buf[9]);
    hdev->raw_gyro_y  = (int16_t)((buf[10] << 8) | buf[11]);
    hdev->raw_gyro_z  = (int16_t)((buf[12] << 8) | buf[13]);

    /* Quy đổi sang đơn vị thực */
    accel_sens = MPU6050_GetAccelSens(hdev->config.accel_fs);
    gyro_sens  = MPU6050_GetGyroSens(hdev->config.gyro_fs);

    /* Gia tốc [m/s²] = (raw / sensitivity_LSBperg) * g */
    hdev->accel_x = ((float)hdev->raw_accel_x / accel_sens) * GRAVITY_M_S2;
    hdev->accel_y = ((float)hdev->raw_accel_y / accel_sens) * GRAVITY_M_S2;
    hdev->accel_z = ((float)hdev->raw_accel_z / accel_sens) * GRAVITY_M_S2;

    /* Nhiệt độ [°C] theo công thức trong datasheet MPU6050 */
    hdev->temp_c = ((float)hdev->raw_temp / 340.0f) + 36.53f;

    /* Vận tốc góc [°/s] trừ đi offset hiệu chỉnh */
    hdev->gyro_x = ((float)hdev->raw_gyro_x / gyro_sens) - hdev->gyro_offset_x;
    hdev->gyro_y = ((float)hdev->raw_gyro_y / gyro_sens) - hdev->gyro_offset_y;
    hdev->gyro_z = ((float)hdev->raw_gyro_z / gyro_sens) - hdev->gyro_offset_z;

    return MPU6050_OK;
}

/**
 * @brief  Chỉ đọc dữ liệu gia tốc kế (6 byte)
 */
MPU6050_Status_t MPU6050_ReadAccel(MPU6050_Handle_t *hdev)
{
    uint8_t buf[6];
    MPU6050_Status_t status;
    float accel_sens;

    status = MPU6050_ReadRegs(hdev, MPU6050_REG_ACCEL_XOUT_H, buf, 6);
    if (status != MPU6050_OK) return status;

    hdev->raw_accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    hdev->raw_accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    hdev->raw_accel_z = (int16_t)((buf[4] << 8) | buf[5]);

    accel_sens = MPU6050_GetAccelSens(hdev->config.accel_fs);
    hdev->accel_x = ((float)hdev->raw_accel_x / accel_sens) * GRAVITY_M_S2;
    hdev->accel_y = ((float)hdev->raw_accel_y / accel_sens) * GRAVITY_M_S2;
    hdev->accel_z = ((float)hdev->raw_accel_z / accel_sens) * GRAVITY_M_S2;

    return MPU6050_OK;
}

/**
 * @brief  Chỉ đọc dữ liệu con quay hồi chuyển (6 byte)
 */
MPU6050_Status_t MPU6050_ReadGyro(MPU6050_Handle_t *hdev)
{
    uint8_t buf[6];
    MPU6050_Status_t status;
    float gyro_sens;

    status = MPU6050_ReadRegs(hdev, MPU6050_REG_GYRO_XOUT_H, buf, 6);
    if (status != MPU6050_OK) return status;

    hdev->raw_gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
    hdev->raw_gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
    hdev->raw_gyro_z = (int16_t)((buf[4] << 8) | buf[5]);

    gyro_sens = MPU6050_GetGyroSens(hdev->config.gyro_fs);
    hdev->gyro_x = ((float)hdev->raw_gyro_x / gyro_sens) - hdev->gyro_offset_x;
    hdev->gyro_y = ((float)hdev->raw_gyro_y / gyro_sens) - hdev->gyro_offset_y;
    hdev->gyro_z = ((float)hdev->raw_gyro_z / gyro_sens) - hdev->gyro_offset_z;

    return MPU6050_OK;
}

/**
 * @brief  Chỉ đọc nhiệt độ nội (2 byte)
 */
MPU6050_Status_t MPU6050_ReadTemp(MPU6050_Handle_t *hdev)
{
    uint8_t buf[2];
    MPU6050_Status_t status;

    status = MPU6050_ReadRegs(hdev, MPU6050_REG_TEMP_OUT_H, buf, 2);
    if (status != MPU6050_OK) return status;

    hdev->raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
    hdev->temp_c   = ((float)hdev->raw_temp / 340.0f) + 36.53f;

    return MPU6050_OK;
}

/**
 * @brief  Hiệu chỉnh offset Gyroscope
 * @note   Hàm block trong (samples * 5ms). Đặt cảm biến bất động trong lúc chạy.
 */
MPU6050_Status_t MPU6050_CalibrateGyro(MPU6050_Handle_t *hdev, uint16_t samples)
{
    MPU6050_Status_t status;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    float gyro_sens;

    /* Đặt lại offset trước khi hiệu chỉnh */
    hdev->gyro_offset_x = 0.0f;
    hdev->gyro_offset_y = 0.0f;
    hdev->gyro_offset_z = 0.0f;

    gyro_sens = MPU6050_GetGyroSens(hdev->config.gyro_fs);

    for (uint16_t i = 0; i < samples; i++)
    {
        status = MPU6050_ReadGyro(hdev);
        if (status != MPU6050_OK) return status;

        /* Cộng dồn giá trị raw (chưa trừ offset) */
        sum_x += (float)hdev->raw_gyro_x / gyro_sens;
        sum_y += (float)hdev->raw_gyro_y / gyro_sens;
        sum_z += (float)hdev->raw_gyro_z / gyro_sens;

        HAL_Delay(5); /* Chờ 5ms giữa các lần đọc (~200Hz) */
    }

    /* Tính giá trị offset trung bình */
    hdev->gyro_offset_x = sum_x / (float)samples;
    hdev->gyro_offset_y = sum_y / (float)samples;
    hdev->gyro_offset_z = sum_z / (float)samples;

    return MPU6050_OK;
}

/**
 * @brief  Kiểm tra WHO_AM_I để xác nhận chip đang hoạt động
 */
MPU6050_Status_t MPU6050_IsReady(MPU6050_Handle_t *hdev)
{
    uint8_t who_am_i = 0;
    MPU6050_Status_t status;

    status = MPU6050_ReadReg(hdev, MPU6050_REG_WHO_AM_I, &who_am_i);
    if (status != MPU6050_OK) return MPU6050_ERROR;
    if (who_am_i != 0x68 && who_am_i != 0x72) return MPU6050_WRONG_DEVICE;

    return MPU6050_OK;
}

/**
 * @brief  Reset chip bằng phần mềm (Software Reset)
 */
MPU6050_Status_t MPU6050_Reset(MPU6050_Handle_t *hdev)
{
    MPU6050_Status_t status;

    /* Bit 7 của PWR_MGMT_1 là DEVICE_RESET */
    status = MPU6050_WriteReg(hdev, MPU6050_REG_PWR_MGMT_1, 0x80);
    if (status != MPU6050_OK) return status;

    HAL_Delay(100); /* Chờ chip hoàn thành reset */
    return MPU6050_OK;
}
