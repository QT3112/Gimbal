/**
 ******************************************************************************
 * @file    icm42688.c
 * @brief   Triển khai thư viện ICM-42688-P SPI (STM32 HAL)
 ******************************************************************************
 */

#include "icm42688.h"
#include <string.h>

/* ===========================================================================
 * Hàm nội bộ — CS control
 * ===========================================================================
 */

static inline void _cs_low(ICM42688_Handle_t *hdev) {
  HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_RESET);
}

static inline void _cs_high(ICM42688_Handle_t *hdev) {
  HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_SET);
}

/* ===========================================================================
 * Hàm nội bộ — Tính sensitivity từ FSR config
 * ===========================================================================
 */

static float _gyro_sensitivity(uint8_t gyro_fsr) {
  switch (gyro_fsr & 0xE0U) {
  case ICM42688_GYRO_FSR_2000DPS:
    return ICM42688_GYRO_SENS_2000DPS;
  case ICM42688_GYRO_FSR_1000DPS:
    return ICM42688_GYRO_SENS_1000DPS;
  case ICM42688_GYRO_FSR_500DPS:
    return ICM42688_GYRO_SENS_500DPS;
  case ICM42688_GYRO_FSR_250DPS:
    return ICM42688_GYRO_SENS_250DPS;
  case ICM42688_GYRO_FSR_125DPS:
    return ICM42688_GYRO_SENS_125DPS;
  case ICM42688_GYRO_FSR_62_5DPS:
    return ICM42688_GYRO_SENS_62_5DPS;
  case ICM42688_GYRO_FSR_31_25DPS:
    return ICM42688_GYRO_SENS_31_25DPS;
  case ICM42688_GYRO_FSR_15_625DPS:
    return ICM42688_GYRO_SENS_15_625DPS;
  default:
    return ICM42688_GYRO_SENS_2000DPS;
  }
}

static float _accel_sensitivity(uint8_t accel_fsr) {
  switch (accel_fsr & 0xE0U) {
  case ICM42688_ACCEL_FSR_16G:
    return ICM42688_ACCEL_SENS_16G;
  case ICM42688_ACCEL_FSR_8G:
    return ICM42688_ACCEL_SENS_8G;
  case ICM42688_ACCEL_FSR_4G:
    return ICM42688_ACCEL_SENS_4G;
  case ICM42688_ACCEL_FSR_2G:
    return ICM42688_ACCEL_SENS_2G;
  default:
    return ICM42688_ACCEL_SENS_16G;
  }
}

/* ===========================================================================
 * API cấp thấp — SPI Primitives
 * ===========================================================================
 */

ICM42688_Status_t ICM42688_WriteReg(ICM42688_Handle_t *hdev, uint8_t reg,
                                    uint8_t data) {
  uint8_t tx[2];
  tx[0] = reg & 0x7FU; /* R/W bit = 0 (Write) */
  tx[1] = data;

  _cs_low(hdev);
  HAL_StatusTypeDef ret =
      HAL_SPI_Transmit(hdev->hspi, tx, 2, ICM42688_SPI_TIMEOUT_MS);
  _cs_high(hdev);

  if (ret != HAL_OK) {
    hdev->spi_error_count++;
    return ICM42688_ERROR_SPI;
  }
  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadReg(ICM42688_Handle_t *hdev, uint8_t reg,
                                   uint8_t *data) {
  uint8_t tx = reg | ICM42688_READ_BIT;
  uint8_t rx = 0;

  _cs_low(hdev);
  HAL_StatusTypeDef ret =
      HAL_SPI_Transmit(hdev->hspi, &tx, 1, ICM42688_SPI_TIMEOUT_MS);
  if (ret == HAL_OK) {
    ret = HAL_SPI_Receive(hdev->hspi, &rx, 1, ICM42688_SPI_TIMEOUT_MS);
  }
  _cs_high(hdev);

  if (ret != HAL_OK) {
    hdev->spi_error_count++;
    return ICM42688_ERROR_SPI;
  }
  *data = rx;
  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadRegs(ICM42688_Handle_t *hdev, uint8_t reg,
                                    uint8_t *pData, uint16_t len) {
  uint8_t tx = reg | ICM42688_READ_BIT;

  _cs_low(hdev);
  HAL_StatusTypeDef ret =
      HAL_SPI_Transmit(hdev->hspi, &tx, 1, ICM42688_SPI_TIMEOUT_MS);
  if (ret == HAL_OK) {
    ret = HAL_SPI_Receive(hdev->hspi, pData, len, ICM42688_SPI_TIMEOUT_MS);
  }
  _cs_high(hdev);

  if (ret != HAL_OK) {
    hdev->spi_error_count++;
    return ICM42688_ERROR_SPI;
  }
  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_SelectBank(ICM42688_Handle_t *hdev, uint8_t bank) {
  if (bank > 4U)
    return ICM42688_ERROR_PARAM;
  return ICM42688_WriteReg(hdev, ICM42688_REG_BANK_SEL, bank);
}

/* ===========================================================================
 * API công khai
 * ===========================================================================
 */

ICM42688_Status_t ICM42688_SoftReset(ICM42688_Handle_t *hdev) {
  ICM42688_Status_t ret;

  /* Chuyển về Bank 0 trước khi reset */
  ret = ICM42688_SelectBank(hdev, 0);
  if (ret != ICM42688_OK)
    return ret;

  /* Ghi SOFT_RESET_CONFIG bit vào DEVICE_CONFIG */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_DEVICE_CONFIG, ICM42688_RESET_BIT);
  if (ret != ICM42688_OK)
    return ret;

  /* Datasheet: chờ tối thiểu 1ms sau soft reset */
  HAL_Delay(ICM42688_RESET_DELAY_MS);
  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_GetWhoAmI(ICM42688_Handle_t *hdev, uint8_t *who) {
  return ICM42688_ReadReg(hdev, ICM42688_REG_WHO_AM_I, who);
}

ICM42688_Status_t ICM42688_Init(ICM42688_Handle_t *hdev,
                                SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                                uint16_t cs_pin,
                                const ICM42688_Config_t *config) {
  ICM42688_Status_t ret;
  uint8_t who = 0;

  if (hdev == NULL || hspi == NULL || cs_port == NULL) {
    return ICM42688_ERROR_PARAM;
  }

  /* Khởi tạo handle */
  memset(hdev, 0, sizeof(ICM42688_Handle_t));
  hdev->hspi = hspi;
  hdev->cs_port = cs_port;
  hdev->cs_pin = cs_pin;

  /* Đặt CS = HIGH (idle) */
  _cs_high(hdev);

  /* Cài thông số mặc định nếu config = NULL */
  uint8_t gyro_fsr, gyro_odr, accel_fsr, accel_odr, gyro_lpf, accel_lpf;
  if (config != NULL) {
    gyro_fsr = config->gyro_fsr;
    gyro_odr = config->gyro_odr;
    accel_fsr = config->accel_fsr;
    accel_odr = config->accel_odr;
    gyro_lpf = config->gyro_lpf;
    accel_lpf = config->accel_lpf;
  } else {
    gyro_fsr = ICM42688_GYRO_FSR_2000DPS;
    gyro_odr = ICM42688_GYRO_ODR_1KHZ;
    accel_fsr = ICM42688_ACCEL_FSR_16G;
    accel_odr = ICM42688_ACCEL_ODR_1KHZ;
    gyro_lpf = ICM42688_UI_FILT_BW_ODR_DIV_20;  /* ~50Hz LPF */
    accel_lpf = ICM42688_UI_FILT_BW_ODR_DIV_20; /* ~50Hz LPF */
  }

  /* Tính sensitivity ngay từ đầu để dùng khi convert */
  hdev->gyro_sensitivity = _gyro_sensitivity(gyro_fsr);
  hdev->accel_sensitivity = _accel_sensitivity(accel_fsr);

  /* --- Bước 1: Soft Reset để đưa chip về trạng thái mặc định --- */
  ret = ICM42688_SoftReset(hdev);
  if (ret != ICM42688_OK)
    return ret;

  /* --- Bước 2: Xác minh WHO_AM_I --- */
  ret = ICM42688_GetWhoAmI(hdev, &who);
  if (ret != ICM42688_OK)
    return ret;
  if (who != ICM42688_WHO_AM_I_VAL)
    return ICM42688_ERROR_WHO_AM_I;

  /* --- Bước 3: Tắt tất cả sensor trước khi cấu hình (tránh lỗi thanh ghi) ---
   */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_PWR_MGMT0, 0x00);
  if (ret != ICM42688_OK)
    return ret;
  HAL_Delay(1); /* Datasheet: 200µs sau khi thay đổi PWR_MGMT0 */

  /* --- Bước 4: Cài Gyroscope — FSR + ODR --- */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_GYRO_CONFIG0, gyro_fsr | gyro_odr);
  if (ret != ICM42688_OK)
    return ret;

  /* --- Bước 5: Cài Accelerometer — FSR + ODR --- */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_ACCEL_CONFIG0,
                          accel_fsr | accel_odr);
  if (ret != ICM42688_OK)
    return ret;

  /* --- Bước 6: Cấu hình UI Low-Pass Filter (LPF) phần cứng --- */
  ret = ICM42688_SetFilter(hdev, gyro_lpf, accel_lpf);
  if (ret != ICM42688_OK)
    return ret;

  /* --- Bước 7: Bật Gyro + Accel ở chế độ Low-Noise --- */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_PWR_MGMT0,
                          ICM42688_GYRO_MODE_LN | ICM42688_ACCEL_MODE_LN);
  if (ret != ICM42688_OK)
    return ret;

  /*
   * Datasheet Section 4.16: Gyroscope cần 45ms để ổn định sau khi
   * chuyển từ OFF → Low-Noise mode. Accel cần 20ms.
   */
  HAL_Delay(ICM42688_STARTUP_DELAY_MS);

  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_SetFilter(ICM42688_Handle_t *hdev,
                                     uint8_t gyro_lpf_bw,
                                     uint8_t accel_lpf_bw) {
  ICM42688_Status_t ret;

  /* GYRO_ACCEL_CONFIG0 (0x52): [7:4] ACCEL_UI_FILT_BW, [3:0] GYRO_UI_FILT_BW */
  uint8_t bw_val = ((accel_lpf_bw & 0x0FU) << 4) | (gyro_lpf_bw & 0x0FU);
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_GYRO_ACCEL_CONFIG0, bw_val);
  if (ret != ICM42688_OK)
    return ret;

  /* GYRO_CONFIG1 (0x51): bit [3:1] GYRO_UI_FILT_ORD = 2 (3rd Order Butterworth)
   */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_GYRO_CONFIG1, 0x04U);
  if (ret != ICM42688_OK)
    return ret;

  /* ACCEL_CONFIG1 (0x53): bit [4:3] ACCEL_UI_FILT_ORD = 2 (3rd Order
   * Butterworth) */
  ret = ICM42688_WriteReg(hdev, ICM42688_REG_ACCEL_CONFIG1, 0x10U);
  if (ret != ICM42688_OK)
    return ret;

  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_CalibrateGyroBias(ICM42688_Handle_t *hdev,
                                             uint16_t samples) {
  if (hdev == NULL || samples == 0)
    return ICM42688_ERROR_PARAM;

  float sum_x = 0.0f;
  float sum_y = 0.0f;
  float sum_z = 0.0f;
  uint16_t valid_cnt = 0;

  hdev->gyro_calibrated = 0;
  hdev->gyro_bias_x = 0.0f;
  hdev->gyro_bias_y = 0.0f;
  hdev->gyro_bias_z = 0.0f;

  for (uint16_t i = 0; i < samples; i++) {
    if (ICM42688_ReadGyro(hdev) == ICM42688_OK) {
      sum_x += hdev->gyro_x_dps;
      sum_y += hdev->gyro_y_dps;
      sum_z += hdev->gyro_z_dps;
      valid_cnt++;
    }
    HAL_Delay(2); /* 2ms per sample */
  }

  if (valid_cnt > 0) {
    hdev->gyro_bias_x = sum_x / (float)valid_cnt;
    hdev->gyro_bias_y = sum_y / (float)valid_cnt;
    hdev->gyro_bias_z = sum_z / (float)valid_cnt;
    hdev->gyro_calibrated = 1;
    return ICM42688_OK;
  }

  return ICM42688_ERROR_SPI;
}

/* ===========================================================================
 * Đọc dữ liệu cảm biến
 * ===========================================================================
 */

ICM42688_Status_t ICM42688_ReadSensor(ICM42688_Handle_t *hdev) {
  ICM42688_Status_t ret;

  /*
   * Burst read 14 bytes liên tiếp bắt đầu từ TEMP_DATA1 (0x1D):
   *   [0-1]:  TEMP_DATA1, TEMP_DATA0
   *   [2-3]:  ACCEL_DATA_X1, ACCEL_DATA_X0
   *   [4-5]:  ACCEL_DATA_Y1, ACCEL_DATA_Y0
   *   [6-7]:  ACCEL_DATA_Z1, ACCEL_DATA_Z0
   *   [8-9]:  GYRO_DATA_X1,  GYRO_DATA_X0
   *   [10-11]: GYRO_DATA_Y1, GYRO_DATA_Y0
   *   [12-13]: GYRO_DATA_Z1, GYRO_DATA_Z0
   */
  uint8_t buf[14];
  ret = ICM42688_ReadRegs(hdev, ICM42688_REG_TEMP_DATA1, buf, 14);
  if (ret != ICM42688_OK)
    return ret;

  /* Ghép 2 byte thành int16 (two's complement, MSB first) */
  hdev->raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
  hdev->raw_accel_x = (int16_t)((buf[2] << 8) | buf[3]);
  hdev->raw_accel_y = (int16_t)((buf[4] << 8) | buf[5]);
  hdev->raw_accel_z = (int16_t)((buf[6] << 8) | buf[7]);
  hdev->raw_gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
  hdev->raw_gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
  hdev->raw_gyro_z = (int16_t)((buf[12] << 8) | buf[13]);

  /* Quy đổi sang đơn vị thực */
  float gs = hdev->gyro_sensitivity;
  float as = hdev->accel_sensitivity;

  float gx = (float)hdev->raw_gyro_x / gs;
  float gy = (float)hdev->raw_gyro_y / gs;
  float gz = (float)hdev->raw_gyro_z / gs;

  if (hdev->gyro_calibrated) {
    gx -= hdev->gyro_bias_x;
    gy -= hdev->gyro_bias_y;
    gz -= hdev->gyro_bias_z;
  }

  hdev->gyro_x_dps = gx;
  hdev->gyro_y_dps = gy;
  hdev->gyro_z_dps = gz;

  hdev->accel_x_g = (float)hdev->raw_accel_x / as;
  hdev->accel_y_g = (float)hdev->raw_accel_y / as;
  hdev->accel_z_g = (float)hdev->raw_accel_z / as;
  hdev->temp_c =
      (float)hdev->raw_temp / ICM42688_TEMP_SENS + ICM42688_TEMP_OFFSET;

  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadGyro(ICM42688_Handle_t *hdev) {
  ICM42688_Status_t ret;
  uint8_t buf[6];

  ret = ICM42688_ReadRegs(hdev, ICM42688_REG_GYRO_DATA_X1, buf, 6);
  if (ret != ICM42688_OK)
    return ret;

  hdev->raw_gyro_x = (int16_t)((buf[0] << 8) | buf[1]);
  hdev->raw_gyro_y = (int16_t)((buf[2] << 8) | buf[3]);
  hdev->raw_gyro_z = (int16_t)((buf[4] << 8) | buf[5]);

  float gs = hdev->gyro_sensitivity;
  float gx = (float)hdev->raw_gyro_x / gs;
  float gy = (float)hdev->raw_gyro_y / gs;
  float gz = (float)hdev->raw_gyro_z / gs;

  if (hdev->gyro_calibrated) {
    gx -= hdev->gyro_bias_x;
    gy -= hdev->gyro_bias_y;
    gz -= hdev->gyro_bias_z;
  }

  hdev->gyro_x_dps = gx;
  hdev->gyro_y_dps = gy;
  hdev->gyro_z_dps = gz;

  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadAccel(ICM42688_Handle_t *hdev) {
  ICM42688_Status_t ret;
  uint8_t buf[6];

  ret = ICM42688_ReadRegs(hdev, ICM42688_REG_ACCEL_DATA_X1, buf, 6);
  if (ret != ICM42688_OK)
    return ret;

  hdev->raw_accel_x = (int16_t)((buf[0] << 8) | buf[1]);
  hdev->raw_accel_y = (int16_t)((buf[2] << 8) | buf[3]);
  hdev->raw_accel_z = (int16_t)((buf[4] << 8) | buf[5]);

  float as = hdev->accel_sensitivity;
  hdev->accel_x_g = (float)hdev->raw_accel_x / as;
  hdev->accel_y_g = (float)hdev->raw_accel_y / as;
  hdev->accel_z_g = (float)hdev->raw_accel_z / as;

  return ICM42688_OK;
}

ICM42688_Status_t ICM42688_ReadTemp(ICM42688_Handle_t *hdev) {
  ICM42688_Status_t ret;
  uint8_t buf[2];

  ret = ICM42688_ReadRegs(hdev, ICM42688_REG_TEMP_DATA1, buf, 2);
  if (ret != ICM42688_OK)
    return ret;

  hdev->raw_temp = (int16_t)((buf[0] << 8) | buf[1]);
  hdev->temp_c =
      (float)hdev->raw_temp / ICM42688_TEMP_SENS + ICM42688_TEMP_OFFSET;

  return ICM42688_OK;
}
