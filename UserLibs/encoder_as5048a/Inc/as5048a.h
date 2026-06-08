/**
 ******************************************************************************
 * @file    as5048a.h
 * @brief   Thư viện giao tiếp với encoder từ tính AS5048A qua SPI (STM32 HAL)
 *
 * === THÔNG SỐ KỸ THUẬT TỪ DATASHEET ===
 *
 * Giao tiếp: SPI Mode 1 (CPOL=0, CPHA=1)
 *   - Dữ liệu được SAMPLE ở cạnh XUỐNG của clock
 *   - Dữ liệu được SHIFT ở cạnh LÊN của clock
 *   - Word length: 16 bit mỗi frame
 *   - CSn: Active LOW
 *   - Tốc độ tối đa: 10 MHz
 *
 * Frame Structure:
 *   MOSI (Command): [15]=PAR(even parity) [14]=RWn(1=Read,0=Write) [13:0]=Address
 *   MISO (Data):    [15]=PAR(even parity) [14]=EF(Error Flag)       [13:0]=Data
 *
 * Đọc dữ liệu cần 2 giao dịch SPI (pipeline):
 *   - Frame 1: Gửi lệnh READ địa chỉ thanh ghi
 *   - Frame 2: Gửi NOP (0xC000) → MISO trả về dữ liệu của Frame 1
 *
 * Độ phân giải góc: 14-bit → 16384 vị trí trên 360°
 *
 * Cách sử dụng:
 *   1. Khai báo: AS5048A_Handle_t enc;
 *   2. Khởi tạo: AS5048A_Init(&enc, &hspi1, GPIOA, GPIO_PIN_4);
 *   3. Đọc góc: AS5048A_ReadAngle(&enc);
 *   4. Truy cập: enc.angle_deg  (độ)  hoặc enc.angle_rad (radian)
 ******************************************************************************
 */

#ifndef AS5048A_H
#define AS5048A_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ===========================================================================
 * Địa chỉ thanh ghi (từ datasheet trang Register Map)
 * =========================================================================== */
#define AS5048A_REG_NOP            0x0000  /*!< NOP / dummy command */
#define AS5048A_REG_CLEAR_ERROR    0x0001  /*!< Đọc để xóa Error Flags */
#define AS5048A_REG_PROG_CTRL      0x0003  /*!< Programming Control (OTP) */
#define AS5048A_REG_ZERO_POS_HI    0x0016  /*!< OTP Zero Position [13:6] */
#define AS5048A_REG_ZERO_POS_LO    0x0017  /*!< OTP Zero Position [5:0] */
#define AS5048A_REG_DIAG_AGC       0x3FFD  /*!< Diagnostics + AGC */
#define AS5048A_REG_MAGNITUDE      0x3FFE  /*!< CORDIC Magnitude 14-bit */
#define AS5048A_REG_ANGLE          0x3FFF  /*!< Góc đầu ra 14-bit (sau zero offset) */

/* ===========================================================================
 * Bit mask và hằng số giao thức SPI
 * =========================================================================== */
#define AS5048A_SPI_READ_BIT       (1U << 14)    /*!< Bit RWn = 1: lệnh READ */
#define AS5048A_SPI_PARITY_BIT     (1U << 15)    /*!< Bit parity ở vị trí bit 15 */
#define AS5048A_DATA_MASK          0x3FFF         /*!< Lấy 14 bit dữ liệu */
#define AS5048A_ERROR_FLAG         (1U << 14)    /*!< Bit EF trong response MISO */
#define AS5048A_MAX_VALUE          16384.0f       /*!< 2^14 = 16384 bước/vòng */
#define AS5048A_CMD_NOP            0xC000U        /*!< Lệnh NOP: READ(1) + addr(0) + parity */

/* Bit mask cho thanh ghi DIAG_AGC (0x3FFD) */
#define AS5048A_DIAG_COMP_HI       (1U << 11) /*!< Từ trường quá mạnh */
#define AS5048A_DIAG_COMP_LO       (1U << 10) /*!< Từ trường quá yếu */
#define AS5048A_DIAG_COF           (1U << 9)  /*!< CORDIC Overflow - dữ liệu không hợp lệ */
#define AS5048A_DIAG_OCF           (1U << 8)  /*!< Offset Compensation Finished - OK khi = 1 */

/* Bit mask cho thanh ghi Error (0x0001) */
#define AS5048A_ERR_PARITY         (1U << 2)  /*!< Lỗi parity */
#define AS5048A_ERR_CMD_INVALID    (1U << 1)  /*!< Lệnh không hợp lệ */
#define AS5048A_ERR_FRAMING        (1U << 0)  /*!< Lỗi framing */

/* SPI timeout */
#define AS5048A_SPI_TIMEOUT_MS     10U

/* ===========================================================================
 * Mã trạng thái
 * =========================================================================== */
typedef enum {
    AS5048A_OK              = 0x00, /*!< Thành công */
    AS5048A_ERROR_SPI       = 0x01, /*!< Lỗi giao tiếp SPI */
    AS5048A_ERROR_PARITY    = 0x02, /*!< Sai parity trong frame nhận */
    AS5048A_ERROR_FLAG      = 0x03, /*!< Cảm biến báo lỗi (EF bit = 1) */
    AS5048A_ERROR_CORDIC    = 0x04, /*!< CORDIC Overflow - từ trường bất thường */
} AS5048A_Status_t;

/* ===========================================================================
 * Handle chính của thư viện
 * =========================================================================== */
typedef struct {
    /* --- Phần cứng --- */
    SPI_HandleTypeDef *hspi;       /*!< Con trỏ SPI handle của HAL */
    GPIO_TypeDef      *cs_port;    /*!< GPIO Port của chân CS (Chip Select) */
    uint16_t           cs_pin;     /*!< GPIO Pin của chân CS */

    /* --- Dữ liệu thô --- */
    uint16_t raw_angle;            /*!< Giá trị góc thô 14-bit (0 đến 16383) */
    uint16_t raw_magnitude;        /*!< Giá trị magnitude thô 14-bit */
    uint16_t raw_diag_agc;         /*!< Giá trị thanh ghi Diagnostics+AGC */

    /* --- Dữ liệu đã quy đổi --- */
    float    angle_deg;            /*!< Góc [độ]: 0.0 đến 359.978 */
    float    angle_rad;            /*!< Góc [radian]: 0.0 đến 2π */

    /* --- Thông tin chẩn đoán --- */
    uint8_t  agc_value;            /*!< Giá trị AGC (0=từ trường mạnh, 255=yếu) */
    uint8_t  comp_high;            /*!< 1 = Từ trường quá mạnh */
    uint8_t  comp_low;             /*!< 1 = Từ trường quá yếu */
    uint8_t  cordic_overflow;      /*!< 1 = CORDIC lỗi, dữ liệu không hợp lệ */
    uint8_t  offset_comp_finished; /*!< 1 = Cảm biến sẵn sàng */

    /* --- Thống kê lỗi --- */
    uint32_t parity_error_count;   /*!< Đếm số lần lỗi parity */
    uint32_t spi_error_count;      /*!< Đếm số lần lỗi SPI */
} AS5048A_Handle_t;

/* ===========================================================================
 * Khai báo hàm API công khai
 * =========================================================================== */

/**
 * @brief  Khởi tạo AS5048A
 * @param  hdev     Con trỏ đến AS5048A_Handle_t
 * @param  hspi     Con trỏ đến SPI_HandleTypeDef đã được CubeMX khởi tạo
 *                  Yêu cầu: SPI Mode 1 (CPOL=0, CPHA=1), 16-bit data size
 * @param  cs_port  GPIO Port của chân CS (ví dụ: GPIOA)
 * @param  cs_pin   GPIO Pin của chân CS (ví dụ: GPIO_PIN_4)
 * @retval AS5048A_Status_t
 */
AS5048A_Status_t AS5048A_Init(AS5048A_Handle_t *hdev, SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
 * @brief  Đọc góc hiện tại (hàm chính, thường dùng trong vòng lặp)
 * @note   Kết quả lưu vào hdev->angle_deg và hdev->angle_rad
 * @param  hdev  Con trỏ đến AS5048A_Handle_t
 * @retval AS5048A_Status_t
 */
AS5048A_Status_t AS5048A_ReadAngle(AS5048A_Handle_t *hdev);

/**
 * @brief  Đọc magnitude (cường độ từ trường - dùng để kiểm tra nam châm)
 * @note   Giá trị lý tưởng: ~8000–12000. Quá thấp/cao = nam châm sai vị trí.
 * @param  hdev  Con trỏ đến AS5048A_Handle_t
 * @retval AS5048A_Status_t
 */
AS5048A_Status_t AS5048A_ReadMagnitude(AS5048A_Handle_t *hdev);

/**
 * @brief  Đọc thanh ghi Diagnostics + AGC
 * @note   Cập nhật các trường: agc_value, comp_high, comp_low, cordic_overflow,
 *         offset_comp_finished trong handle
 * @param  hdev  Con trỏ đến AS5048A_Handle_t
 * @retval AS5048A_Status_t
 */
AS5048A_Status_t AS5048A_ReadDiagnostics(AS5048A_Handle_t *hdev);

/**
 * @brief  Đọc và xóa Error Register (0x0001)
 * @note   Gọi hàm này khi phát hiện EF bit = 1 trong response để biết lỗi gì
 * @param  hdev       Con trỏ đến AS5048A_Handle_t
 * @param  error_bits Con trỏ nhận giá trị lỗi (bits: parity, cmd_invalid, framing)
 * @retval AS5048A_Status_t
 */
AS5048A_Status_t AS5048A_ClearErrors(AS5048A_Handle_t *hdev, uint16_t *error_bits);

/**
 * @brief  Kiểm tra parity của 1 frame 16-bit (even parity)
 * @param  value  Giá trị 16-bit cần kiểm tra
 * @retval 1 nếu parity hợp lệ (số bit 1 là số chẵn), 0 nếu lỗi
 */
uint8_t AS5048A_CheckParity(uint16_t value);

/**
 * @brief  Tính và thêm parity bit vào command frame
 * @param  command  Giá trị 16-bit command (chưa có parity)
 * @retval Giá trị 16-bit command với parity bit được set đúng
 */
uint16_t AS5048A_AddParity(uint16_t command);

#ifdef __cplusplus
}
#endif

#endif /* AS5048A_H */
