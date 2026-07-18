/**
 ******************************************************************************
 * @file    icm42688.h
 * @brief   Thư viện giao tiếp với ICM-42688-P qua SPI 4-wire (STM32 HAL)
 *
 * === GIAO THỨC SPI (Datasheet DS-000347 Rev 1.9, Section 9.6) ===
 *
 *  - Mode: SPI Mode 3 (CPOL=1, CPHA=1) — dữ liệu latch cạnh lên của SCLK
 *  - Tốc độ tối đa: 24 MHz
 *  - Dữ liệu: MSB first
 *  - Frame 1 byte: [bit7]=R/W (1=Read, 0=Write), [bit6:0]=Register Address
 *  - Frame 2+: dữ liệu đọc/ghi
 *  - CS: Active LOW
 *
 * === KIẾN TRÚC BANK REGISTER ===
 *
 *  ICM-42688-P có 5 Register Bank (0-4), chọn bằng REG_BANK_SEL (địa chỉ 0x76):
 *    Bank 0: Thanh ghi dữ liệu chính (ACCEL, GYRO, TEMP, WHO_AM_I...)
 *    Bank 1: Cấu hình nâng cao Gyroscope (AAF, notch filter...)
 *    Bank 2: Cấu hình nâng cao Accelerometer (AAF...)
 *    Bank 3, 4: APEX motion functions
 *
 * === CẤU HÌNH MẶC ĐỊNH SAU INIT ===
 *
 *  - Gyroscope:     Low-Noise mode, ±2000 dps, ODR 1kHz
 *  - Accelerometer: Low-Noise mode, ±16g,     ODR 1kHz
 *  - SPI mode được kích hoạt bằng cách kéo CS thấp 1 lần trong Init
 *
 * === CÔNG THỨC QUY ĐỔI (Datasheet Table 1, Table 2) ===
 *
 *  Gyroscope  [dps]:   raw_gyro  / sensitivity (LSB/dps)
 *  Accel      [g]:     raw_accel / sensitivity (LSB/g)
 *  Nhiệt độ   [°C]:    raw_temp  / 132.48 + 25
 *
 * === CÁCH SỬ DỤNG ===
 *
 *   1. Khai báo:  ICM42688_Handle_t imu;
 *   2. Khởi tạo:  ICM42688_Init(&imu, &hspi3, GPIOA, GPIO_PIN_4);
 *   3. Đọc data:  ICM42688_ReadSensor(&imu);
 *   4. Truy cập:  imu.gyro_x_dps, imu.accel_x_g, imu.temp_c
 ******************************************************************************
 */

#ifndef ICM42688_H
#define ICM42688_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <math.h>

/* ===========================================================================
 * Địa chỉ các thanh ghi — Bank 0 (hay dùng nhất)
 * =========================================================================== */
#define ICM42688_REG_DEVICE_CONFIG      0x11U  /*!< Soft Reset */
#define ICM42688_REG_INT_CONFIG         0x14U  /*!< Interrupt pin config */
#define ICM42688_REG_FIFO_CONFIG        0x16U  /*!< FIFO mode */
#define ICM42688_REG_TEMP_DATA1         0x1DU  /*!< Temperature [15:8] */
#define ICM42688_REG_TEMP_DATA0         0x1EU  /*!< Temperature [7:0] */
#define ICM42688_REG_ACCEL_DATA_X1      0x1FU  /*!< Accel X [15:8] */
#define ICM42688_REG_ACCEL_DATA_X0      0x20U  /*!< Accel X [7:0] */
#define ICM42688_REG_ACCEL_DATA_Y1      0x21U  /*!< Accel Y [15:8] */
#define ICM42688_REG_ACCEL_DATA_Y0      0x22U  /*!< Accel Y [7:0] */
#define ICM42688_REG_ACCEL_DATA_Z1      0x23U  /*!< Accel Z [15:8] */
#define ICM42688_REG_ACCEL_DATA_Z0      0x24U  /*!< Accel Z [7:0] */
#define ICM42688_REG_GYRO_DATA_X1       0x25U  /*!< Gyro X [15:8] */
#define ICM42688_REG_GYRO_DATA_X0       0x26U  /*!< Gyro X [7:0] */
#define ICM42688_REG_GYRO_DATA_Y1       0x27U  /*!< Gyro Y [15:8] */
#define ICM42688_REG_GYRO_DATA_Y0       0x28U  /*!< Gyro Y [7:0] */
#define ICM42688_REG_GYRO_DATA_Z1       0x29U  /*!< Gyro Z [15:8] */
#define ICM42688_REG_GYRO_DATA_Z0       0x2AU  /*!< Gyro Z [7:0] */
#define ICM42688_REG_INT_STATUS         0x2DU  /*!< Interrupt status */
#define ICM42688_REG_PWR_MGMT0          0x4EU  /*!< Power management */
#define ICM42688_REG_GYRO_CONFIG0       0x4FU  /*!< Gyro FSR & ODR */
#define ICM42688_REG_ACCEL_CONFIG0      0x50U  /*!< Accel FSR & ODR */
#define ICM42688_REG_GYRO_CONFIG1       0x51U  /*!< Gyro UI filter */
#define ICM42688_REG_GYRO_ACCEL_CONFIG0 0x52U  /*!< Accel UI filter */
#define ICM42688_REG_ACCEL_CONFIG1      0x53U  /*!< Accel averaging */
#define ICM42688_REG_TMST_CONFIG        0x54U  /*!< Timestamp config */
#define ICM42688_REG_INT_CONFIG0        0x63U  /*!< Interrupt latch/deassert */
#define ICM42688_REG_INT_CONFIG1        0x64U  /*!< Interrupt async reset */
#define ICM42688_REG_INT_SOURCE0        0x65U  /*!< INT1 interrupt source */
#define ICM42688_REG_WHO_AM_I           0x75U  /*!< Device ID = 0x47 */
#define ICM42688_REG_BANK_SEL           0x76U  /*!< Register bank selection */

/* ===========================================================================
 * Hằng số nhận dạng thiết bị
 * =========================================================================== */
#define ICM42688_WHO_AM_I_VAL           0x47U  /*!< Giá trị mong đợi từ WHO_AM_I */

/* ===========================================================================
 * Cấu trúc bit PWR_MGMT0 (0x4E)
 *   [3:2] GYRO_MODE:  00=off, 01=standby, 11=Low-Noise
 *   [1:0] ACCEL_MODE: 00=off, 01=LP, 10=LP, 11=Low-Noise
 * =========================================================================== */
#define ICM42688_GYRO_MODE_OFF          0x00U
#define ICM42688_GYRO_MODE_STANDBY      0x04U
#define ICM42688_GYRO_MODE_LN           0x0CU  /*!< Low-Noise (recommended for gimbal) */
#define ICM42688_ACCEL_MODE_OFF         0x00U
#define ICM42688_ACCEL_MODE_LP          0x02U  /*!< Low-Power */
#define ICM42688_ACCEL_MODE_LN          0x03U  /*!< Low-Noise */

/* ===========================================================================
 * Lựa chọn Full-Scale Range (FSR) cho GYRO_CONFIG0 (0x4F)
 * [7:5] GYRO_UI_FS_SEL
 * =========================================================================== */
#define ICM42688_GYRO_FSR_2000DPS       0x00U  /*!< ±2000 dps, Sensitivity: 16.4 LSB/dps */
#define ICM42688_GYRO_FSR_1000DPS       0x20U  /*!< ±1000 dps, Sensitivity: 32.8 LSB/dps */
#define ICM42688_GYRO_FSR_500DPS        0x40U  /*!< ±500  dps, Sensitivity: 65.5 LSB/dps */
#define ICM42688_GYRO_FSR_250DPS        0x60U  /*!< ±250  dps, Sensitivity: 131  LSB/dps */
#define ICM42688_GYRO_FSR_125DPS        0x80U  /*!< ±125  dps, Sensitivity: 262  LSB/dps */
#define ICM42688_GYRO_FSR_62_5DPS       0xA0U  /*!< ±62.5 dps, Sensitivity: 524  LSB/dps */
#define ICM42688_GYRO_FSR_31_25DPS      0xC0U  /*!< ±31.25 dps */
#define ICM42688_GYRO_FSR_15_625DPS     0xE0U  /*!< ±15.625 dps */

/* ===========================================================================
 * Lựa chọn ODR cho GYRO_CONFIG0 (0x4F)
 * [3:0] GYRO_ODR
 * =========================================================================== */
#define ICM42688_GYRO_ODR_32KHZ         0x01U
#define ICM42688_GYRO_ODR_16KHZ         0x02U
#define ICM42688_GYRO_ODR_8KHZ          0x03U
#define ICM42688_GYRO_ODR_4KHZ          0x04U
#define ICM42688_GYRO_ODR_2KHZ          0x05U
#define ICM42688_GYRO_ODR_1KHZ          0x06U  /*!< Mặc định */
#define ICM42688_GYRO_ODR_200HZ         0x07U
#define ICM42688_GYRO_ODR_100HZ         0x08U
#define ICM42688_GYRO_ODR_50HZ          0x09U
#define ICM42688_GYRO_ODR_25HZ          0x0AU
#define ICM42688_GYRO_ODR_12_5HZ        0x0BU
#define ICM42688_GYRO_ODR_500HZ         0x0FU

/* ===========================================================================
 * Lựa chọn Full-Scale Range cho ACCEL_CONFIG0 (0x50)
 * [7:5] ACCEL_UI_FS_SEL
 * =========================================================================== */
#define ICM42688_ACCEL_FSR_16G          0x00U  /*!< ±16g, Sensitivity: 2048  LSB/g */
#define ICM42688_ACCEL_FSR_8G           0x20U  /*!< ±8g,  Sensitivity: 4096  LSB/g */
#define ICM42688_ACCEL_FSR_4G           0x40U  /*!< ±4g,  Sensitivity: 8192  LSB/g */
#define ICM42688_ACCEL_FSR_2G           0x60U  /*!< ±2g,  Sensitivity: 16384 LSB/g */

/* ===========================================================================
 * Lựa chọn ODR cho ACCEL_CONFIG0 (0x50)
 * [3:0] ACCEL_ODR
 * =========================================================================== */
#define ICM42688_ACCEL_ODR_32KHZ        0x01U
#define ICM42688_ACCEL_ODR_16KHZ        0x02U
#define ICM42688_ACCEL_ODR_8KHZ         0x03U
#define ICM42688_ACCEL_ODR_4KHZ         0x04U
#define ICM42688_ACCEL_ODR_2KHZ         0x05U
#define ICM42688_ACCEL_ODR_1KHZ         0x06U  /*!< Mặc định */
#define ICM42688_ACCEL_ODR_200HZ        0x07U
#define ICM42688_ACCEL_ODR_100HZ        0x08U
#define ICM42688_ACCEL_ODR_50HZ         0x09U
#define ICM42688_ACCEL_ODR_25HZ         0x0AU
#define ICM42688_ACCEL_ODR_12_5HZ       0x0BU
#define ICM42688_ACCEL_ODR_500HZ        0x0FU

/* ===========================================================================
 * Timeout và hằng số SPI
 * =========================================================================== */
#define ICM42688_SPI_TIMEOUT_MS         10U
#define ICM42688_READ_BIT               0x80U   /*!< Bit R/W=1 để đọc */
#define ICM42688_RESET_BIT              0x01U   /*!< Bit SOFT_RESET trong DEVICE_CONFIG */
#define ICM42688_RESET_DELAY_MS         2U      /*!< Datasheet: chờ ít nhất 1ms sau reset */
#define ICM42688_STARTUP_DELAY_MS       100U    /*!< Chờ sensor ổn định sau power-on */

/* ===========================================================================
 * Độ nhạy (Sensitivity) tương ứng FSR — dùng để quy đổi raw → đơn vị thực
 * =========================================================================== */
#define ICM42688_GYRO_SENS_2000DPS      16.4f
#define ICM42688_GYRO_SENS_1000DPS      32.8f
#define ICM42688_GYRO_SENS_500DPS       65.5f
#define ICM42688_GYRO_SENS_250DPS       131.0f
#define ICM42688_GYRO_SENS_125DPS       262.0f
#define ICM42688_GYRO_SENS_62_5DPS      524.3f
#define ICM42688_GYRO_SENS_31_25DPS     1048.6f
#define ICM42688_GYRO_SENS_15_625DPS    2097.2f

#define ICM42688_ACCEL_SENS_16G         2048.0f
#define ICM42688_ACCEL_SENS_8G          4096.0f
#define ICM42688_ACCEL_SENS_4G          8192.0f
#define ICM42688_ACCEL_SENS_2G          16384.0f

#define ICM42688_TEMP_SENS              132.48f  /*!< LSB/°C */
#define ICM42688_TEMP_OFFSET            25.0f    /*!< °C tại output = 0 */

/* ===========================================================================
 * Mã trạng thái trả về
 * =========================================================================== */
typedef enum {
    ICM42688_OK              = 0x00, /*!< Thành công */
    ICM42688_ERROR_SPI       = 0x01, /*!< Lỗi giao tiếp SPI */
    ICM42688_ERROR_WHO_AM_I  = 0x02, /*!< WHO_AM_I không khớp (sai chip / kết nối) */
    ICM42688_ERROR_PARAM     = 0x03, /*!< Tham số đầu vào không hợp lệ */
} ICM42688_Status_t;

/* ===========================================================================
 * Cấu hình cảm biến — truyền vào ICM42688_Init()
 * =========================================================================== */
typedef struct {
    uint8_t gyro_fsr;    /*!< ICM42688_GYRO_FSR_xxx */
    uint8_t gyro_odr;    /*!< ICM42688_GYRO_ODR_xxx */
    uint8_t accel_fsr;   /*!< ICM42688_ACCEL_FSR_xxx */
    uint8_t accel_odr;   /*!< ICM42688_ACCEL_ODR_xxx */
} ICM42688_Config_t;

/* ===========================================================================
 * Handle chính của thư viện
 * =========================================================================== */
typedef struct {
    /* --- Phần cứng --- */
    SPI_HandleTypeDef *hspi;     /*!< Con trỏ SPI handle của HAL */
    GPIO_TypeDef      *cs_port;  /*!< GPIO Port chân CS */
    uint16_t           cs_pin;   /*!< GPIO Pin chân CS */

    /* --- Cấu hình hiện tại --- */
    float gyro_sensitivity;      /*!< LSB/dps — được tính từ FSR khi Init */
    float accel_sensitivity;     /*!< LSB/g   — được tính từ FSR khi Init */

    /* --- Dữ liệu thô (two's complement) --- */
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
    int16_t raw_accel_x;
    int16_t raw_accel_y;
    int16_t raw_accel_z;
    int16_t raw_temp;

    /* --- Dữ liệu đã quy đổi --- */
    float gyro_x_dps;    /*!< Vận tốc góc trục X [dps] */
    float gyro_y_dps;    /*!< Vận tốc góc trục Y [dps] */
    float gyro_z_dps;    /*!< Vận tốc góc trục Z [dps] */
    float accel_x_g;     /*!< Gia tốc trục X [g] */
    float accel_y_g;     /*!< Gia tốc trục Y [g] */
    float accel_z_g;     /*!< Gia tốc trục Z [g] */
    float temp_c;        /*!< Nhiệt độ chip [°C] */

    /* --- Thống kê lỗi --- */
    uint32_t spi_error_count;
} ICM42688_Handle_t;

/* ===========================================================================
 * API công khai
 * =========================================================================== */

/**
 * @brief  Khởi tạo ICM-42688-P với cấu hình mặc định
 *
 * Quy trình:
 *   1. Soft reset
 *   2. Xác minh WHO_AM_I = 0x47
 *   3. Cấu hình Gyro/Accel theo config, bật Low-Noise mode
 *
 * @param  hdev     Con trỏ handle của thư viện
 * @param  hspi     Con trỏ SPI handle (SPI Mode 3: CPOL=1, CPHA=1, 8-bit)
 * @param  cs_port  GPIO Port chân CS
 * @param  cs_pin   GPIO Pin chân CS
 * @param  config   Con trỏ cấu hình (NULL để dùng mặc định: ±2000dps, ±16g, 1kHz)
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_Init(ICM42688_Handle_t *hdev,
                                SPI_HandleTypeDef *hspi,
                                GPIO_TypeDef *cs_port, uint16_t cs_pin,
                                const ICM42688_Config_t *config);

/**
 * @brief  Đọc toàn bộ dữ liệu cảm biến (Accel + Gyro + Temp) trong 1 lần Burst Read
 *
 * Hàm này đọc 14 bytes liên tiếp từ địa chỉ TEMP_DATA1 (0x1D),
 * bao gồm: TEMP(2) + ACCEL_XYZ(6) + GYRO_XYZ(6).
 * Kết quả được lưu vào các trường raw_xxx và xxx_dps/g/c của handle.
 *
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_ReadSensor(ICM42688_Handle_t *hdev);

/**
 * @brief  Đọc chỉ dữ liệu Gyroscope (6 bytes)
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_ReadGyro(ICM42688_Handle_t *hdev);

/**
 * @brief  Đọc chỉ dữ liệu Accelerometer (6 bytes)
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_ReadAccel(ICM42688_Handle_t *hdev);

/**
 * @brief  Đọc chỉ dữ liệu Nhiệt độ (2 bytes)
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_ReadTemp(ICM42688_Handle_t *hdev);

/**
 * @brief  Đọc giá trị WHO_AM_I để kiểm tra kết nối
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @param  who   Con trỏ nhận giá trị (0x47 = OK)
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_GetWhoAmI(ICM42688_Handle_t *hdev, uint8_t *who);

/**
 * @brief  Thực hiện Soft Reset chip
 * @note   Hàm sẽ delay 2ms sau khi ghi reset bit
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_SoftReset(ICM42688_Handle_t *hdev);

/**
 * @brief  Ghi một thanh ghi (Bank 0) — hàm cấp thấp
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @param  reg   Địa chỉ thanh ghi (Bank 0)
 * @param  data  Giá trị cần ghi
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_WriteReg(ICM42688_Handle_t *hdev, uint8_t reg, uint8_t data);

/**
 * @brief  Đọc một thanh ghi (Bank 0) — hàm cấp thấp
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @param  reg   Địa chỉ thanh ghi (Bank 0)
 * @param  data  Con trỏ nhận dữ liệu
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_ReadReg(ICM42688_Handle_t *hdev, uint8_t reg, uint8_t *data);

/**
 * @brief  Đọc nhiều thanh ghi liên tiếp (Burst Read) — hàm cấp thấp
 * @param  hdev   Con trỏ ICM42688_Handle_t
 * @param  reg    Địa chỉ bắt đầu
 * @param  pData  Buffer nhận dữ liệu
 * @param  len    Số byte cần đọc
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_ReadRegs(ICM42688_Handle_t *hdev, uint8_t reg,
                                    uint8_t *pData, uint16_t len);

/**
 * @brief  Chọn Register Bank
 * @note   Phải gọi trước khi truy cập thanh ghi thuộc Bank 1-4.
 *         Nhớ chuyển về Bank 0 sau khi dùng xong.
 * @param  hdev  Con trỏ ICM42688_Handle_t
 * @param  bank  Số bank: 0, 1, 2, 3, 4
 * @retval ICM42688_Status_t
 */
ICM42688_Status_t ICM42688_SelectBank(ICM42688_Handle_t *hdev, uint8_t bank);

#ifdef __cplusplus
}
#endif

#endif /* ICM42688_H */
