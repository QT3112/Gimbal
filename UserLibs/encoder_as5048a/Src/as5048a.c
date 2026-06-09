/**
 ******************************************************************************
 * @file    as5048a.c
 * @brief   Thư viện giao tiếp với encoder từ tính AS5048A qua SPI (STM32 HAL)
 *
 * === NGUYÊN LÝ GIAO TIẾP SPI CỦA AS5048A ===
 *
 * 1. SPI Mode 1 (CPOL=0, CPHA=1): Sample ở cạnh XUỐNG, Shift ở cạnh LÊN
 *
 * 2. Mỗi giao dịch = 1 frame 16-bit, gồm:
 *    MOSI: [15]=Parity [14]=RWn [13:0]=Address
 *    MISO: [15]=Parity [14]=EF  [13:0]=Data (của LỆNH TRƯỚC đó)
 *
 * 3. DO ĐÓ, đọc một thanh ghi cần 2 frame (pipeline):
 *    Frame 1 → MOSI: gửi lệnh READ(addr)    | MISO: chứa dữ liệu cũ (bỏ qua)
 *    Frame 2 → MOSI: gửi NOP (0xC000)       | MISO: chứa dữ liệu của Frame 1 ← đây là data cần
 *
 * 4. Parity: Even parity trên 16 bit. Số bit = 1 phải là số CHẴN.
 *
 * === VÍ DỤ TIMING ===
 *    CS ___┐                                       ┌___
 *          └───────────────────────────────────────┘
 *   CLK         ↑↓  ↑↓  ↑↓  ...  ↑↓  ||  ↑↓  ↑↓  ...  ↑↓
 *   MOSI [PAR][RWn][A13..A0]     || [PAR][1][1][0][0][0..0] (NOP=0xC000)
 *   MISO  (bỏ qua frame 1)       || [PAR][EF][D13..D0]  ← GÓC Ở ĐÂY
 ******************************************************************************
 */

#include "as5048a.h"
#include <string.h>

/* ===========================================================================
 * Hàm nội bộ (static)
 * =========================================================================== */

/**
 * @brief  Kéo CS xuống LOW (chọn chip)
 */
static inline void AS5048A_CS_Low(AS5048A_Handle_t *hdev)
{
    HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief  Kéo CS lên HIGH (bỏ chọn chip)
 */
static inline void AS5048A_CS_High(AS5048A_Handle_t *hdev)
{
    HAL_GPIO_WritePin(hdev->cs_port, hdev->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief  Tính even parity của giá trị 16-bit
 *         Even parity: tổng số bit = 1 phải là số CHẴN
 * @param  value  Giá trị 16-bit
 * @retval 0 nếu parity đúng (số bit 1 là chẵn), 1 nếu sai
 */
static uint8_t AS5048A_CalcParity(uint16_t value)
{
    uint16_t v = value;
    v ^= (v >> 8);
    v ^= (v >> 4);
    v ^= (v >> 2);
    v ^= (v >> 1);
    return (v & 0x01U); /* 0 = số bit 1 là chẵn, 1 = lẻ */
}

/**
 * @brief  Thực hiện 1 giao dịch SPI 16-bit với CS tự động
 *         HAL_SPI_TransmitReceive truyền/nhận đồng thời (full-duplex)
 * @param  hdev     Handle AS5048A
 * @param  tx_data  Dữ liệu gửi đi (MOSI)
 * @param  rx_data  Con trỏ nhận dữ liệu về (MISO)
 * @retval AS5048A_Status_t
 */
static AS5048A_Status_t AS5048A_SPI_Transfer(AS5048A_Handle_t *hdev,
                                              uint16_t tx_data,
                                              uint16_t *rx_data)
{
    HAL_StatusTypeDef ret;
    uint8_t tx_buf[2], rx_buf[2];

    /* Chuyển từ uint16 sang byte array theo thứ tự Big-Endian (MSB first) */
    tx_buf[0] = (uint8_t)(tx_data >> 8);
    tx_buf[1] = (uint8_t)(tx_data & 0xFF);

    AS5048A_CS_Low(hdev);

    ret = HAL_SPI_TransmitReceive(hdev->hspi, tx_buf, rx_buf, 2, AS5048A_SPI_TIMEOUT_MS);

    AS5048A_CS_High(hdev);

    if (ret != HAL_OK) {
        hdev->spi_error_count++;
        return AS5048A_ERROR_SPI;
    }

    *rx_data = ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
    return AS5048A_OK;
}

/**
 * @brief  Đọc 1 thanh ghi của AS5048A (cần 2 frame SPI)
 * @param  hdev     Handle AS5048A
 * @param  reg_addr Địa chỉ thanh ghi (14-bit)
 * @param  data     Con trỏ nhận dữ liệu 14-bit
 * @retval AS5048A_Status_t
 */
static AS5048A_Status_t AS5048A_ReadRegister(AS5048A_Handle_t *hdev,
                                              uint16_t reg_addr,
                                              uint16_t *data)
{
    AS5048A_Status_t status;
    uint16_t cmd, rx_frame1, rx_frame2;

    /* --- Frame 1: Gửi lệnh READ địa chỉ thanh ghi --- */
    /* Bit 14 = 1 (READ), Bit 15 = parity */
    cmd = AS5048A_SPI_READ_BIT | (reg_addr & 0x3FFFU);
    cmd = AS5048A_AddParity(cmd);

    status = AS5048A_SPI_Transfer(hdev, cmd, &rx_frame1);
    if (status != AS5048A_OK) return status;
    /* rx_frame1 chứa dữ liệu của lệnh TRƯỚC đó → bỏ qua */

    /* --- Frame 2: Gửi NOP để clock ra dữ liệu của Frame 1 --- */
    status = AS5048A_SPI_Transfer(hdev, AS5048A_CMD_NOP, &rx_frame2);
    if (status != AS5048A_OK) return status;

    /* --- Kiểm tra parity của frame nhận về --- */
    if (AS5048A_CalcParity(rx_frame2) != 0) {
        hdev->parity_error_count++;
        return AS5048A_ERROR_PARITY;
    }

    /* --- Kiểm tra Error Flag (bit 14 của MISO) --- */
    if (rx_frame2 & AS5048A_EF_BIT) {
        return AS5048A_ERROR_FLAG;
    }

    /* --- Trả về 14 bit dữ liệu --- */
    *data = rx_frame2 & AS5048A_DATA_MASK;
    return AS5048A_OK;
}

/* ===========================================================================
 * Hàm API công khai
 * =========================================================================== */

/**
 * @brief  Tính và thêm even parity bit vào command frame
 */
uint16_t AS5048A_AddParity(uint16_t command)
{
    /* Tính parity trên 15 bit thấp (bit 14:0) */
    uint16_t parity = AS5048A_CalcParity(command & 0x7FFFU);
    if (parity) {
        command |= AS5048A_SPI_PARITY_BIT;   /* Đặt bit parity = 1 để tổng thành chẵn */
    } else {
        command &= ~AS5048A_SPI_PARITY_BIT;  /* Bit parity = 0, tổng đã chẵn */
    }
    return command;
}

/**
 * @brief  Kiểm tra parity của frame nhận
 */
uint8_t AS5048A_CheckParity(uint16_t value)
{
    return (AS5048A_CalcParity(value) == 0) ? 1U : 0U;
}

/**
 * @brief  Khởi tạo AS5048A
 */
AS5048A_Status_t AS5048A_Init(AS5048A_Handle_t *hdev, SPI_HandleTypeDef *hspi,
                               GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    if (hdev == NULL || hspi == NULL || cs_port == NULL) return AS5048A_ERROR_SPI;

    /* Xóa struct và thiết lập phần cứng */
    memset(hdev, 0, sizeof(AS5048A_Handle_t));
    hdev->hspi    = hspi;
    hdev->cs_port = cs_port;
    hdev->cs_pin  = cs_pin;

    /* Đảm bảo CS ở mức HIGH */
    AS5048A_CS_High(hdev);
    HAL_Delay(1); /* Chờ chip ổn định sau power-up */

    /* =========================================================
     * BƯỚC 1: Flush SPI pipeline bằng 1 frame NOP giả
     *
     * AS5048A dùng cơ chế pipeline 1 frame trễ. Khi chip vừa
     * khởi động, "lệnh trước đó" không tồn tại, nên EF bit sẽ
     * được bật trong response của frame đầu tiên. Gửi NOP để
     * frame giả này hấp thụ phần "kết quả rác" đó.
     * ========================================================= */
    {
        uint16_t dummy = 0;
        AS5048A_SPI_Transfer(hdev, AS5048A_CMD_NOP, &dummy);
        /* Kết quả dummy bị bỏ qua hoàn toàn */
    }

    /* =========================================================
     * BƯỚC 2: Xóa Error Register (0x0001)
     *
     * Đọc thanh ghi lỗi sẽ đồng thời xóa tất cả các error flag
     * (parity error, command invalid, framing error). Bỏ qua
     * trạng thái trả về vì EF có thể vẫn còn ở bước này.
     * ========================================================= */
    {
        uint16_t err_bits = 0;
        AS5048A_ClearErrors(hdev, &err_bits);
    }

    /* =========================================================
     * BƯỚC 3: Đọc Diagnostics để xác nhận chip hoạt động bình thường
     *
     * Sau khi đã xóa lỗi, frame này sẽ không còn EF = 1 nữa.
     * OCF = 1 nghĩa là chip đã hoàn thành Offset Compensation.
     * ========================================================= */
    AS5048A_Status_t status = AS5048A_ReadDiagnostics(hdev);

    return status;
}

/**
 * @brief  Đọc góc hiện tại từ thanh ghi ANGLE (0x3FFF)
 *
 * Công thức quy đổi:
 *   angle_deg = raw_angle * (360.0 / 16384.0)
 *   angle_rad = raw_angle * (2*PI / 16384.0)
 */
AS5048A_Status_t AS5048A_ReadAngle(AS5048A_Handle_t *hdev)
{
    AS5048A_Status_t status;
    uint16_t raw = 0;

    status = AS5048A_ReadRegister(hdev, AS5048A_REG_ANGLE, &raw);
    if (status != AS5048A_OK) return status;

    hdev->raw_angle  = raw;
    hdev->angle_deg  = (float)raw * (360.0f / AS5048A_MAX_VALUE);
    hdev->angle_rad  = (float)raw * (6.28318530718f / AS5048A_MAX_VALUE);

    return AS5048A_OK;
}

/**
 * @brief  Đọc magnitude (cường độ từ trường) từ thanh ghi MAGNITUDE (0x3FFE)
 *
 * Giá trị lý tưởng: 8000–12000
 *   < 4000  → Nam châm quá xa hoặc quá yếu
 *   > 14000 → Nam châm quá gần hoặc quá mạnh
 */
AS5048A_Status_t AS5048A_ReadMagnitude(AS5048A_Handle_t *hdev)
{
    AS5048A_Status_t status;
    uint16_t raw = 0;

    status = AS5048A_ReadRegister(hdev, AS5048A_REG_MAGNITUDE, &raw);
    if (status != AS5048A_OK) return status;

    hdev->raw_magnitude = raw;
    return AS5048A_OK;
}

/**
 * @brief  Đọc thanh ghi Diagnostics + AGC (0x3FFD)
 *
 * Layout thanh ghi (14-bit hữu dụng):
 *   [13:8] = AGC value (0–63): 0=từ trường mạnh, 63=yếu (cần điều chỉnh nam châm)
 *   [11]   = COMP_H: Từ trường quá mạnh (nam châm quá gần)
 *   [10]   = COMP_L: Từ trường quá yếu (nam châm quá xa)
 *   [9]    = COF: CORDIC Overflow - lỗi tính toán, dữ liệu góc không tin cậy
 *   [8]    = OCF: Offset Compensation Finished = 1 khi cảm biến sẵn sàng
 *   [7:0]  = AGC value (8-bit đầy đủ)
 */
AS5048A_Status_t AS5048A_ReadDiagnostics(AS5048A_Handle_t *hdev)
{
    AS5048A_Status_t status;
    uint16_t raw = 0;

    status = AS5048A_ReadRegister(hdev, AS5048A_REG_DIAG_AGC, &raw);
    if (status != AS5048A_OK) return status;

    hdev->raw_diag_agc         = raw;
    hdev->agc_value            = (uint8_t)(raw & 0xFFU);       /* Byte thấp = AGC */
    hdev->comp_high            = (raw & AS5048A_DIAG_COMP_HI) ? 1U : 0U;
    hdev->comp_low             = (raw & AS5048A_DIAG_COMP_LO) ? 1U : 0U;
    hdev->cordic_overflow      = (raw & AS5048A_DIAG_COF)     ? 1U : 0U;
    hdev->offset_comp_finished = (raw & AS5048A_DIAG_OCF)     ? 1U : 0U;

    return AS5048A_OK;
}

/**
 * @brief  Đọc và xóa Error Register (0x0001)
 *
 * Phải đọc thanh ghi này khi EF bit = 1 trong response.
 * Hành động đọc đồng thời xóa lỗi.
 *
 * Bit definition (3 bit thấp):
 *   [2] = Framing Error: số bit trong frame không đúng
 *   [1] = Invalid Command Error: địa chỉ không hợp lệ
 *   [0] = Parity Error: parity sai
 */
AS5048A_Status_t AS5048A_ClearErrors(AS5048A_Handle_t *hdev, uint16_t *error_bits)
{
    AS5048A_Status_t status;
    uint16_t raw = 0;

    /* Địa chỉ 0x0001 là READ-only, đọc sẽ đồng thời clear */
    status = AS5048A_ReadRegister(hdev, AS5048A_REG_CLEAR_ERROR, &raw);

    if (error_bits != NULL) {
        *error_bits = raw & 0x0007U; /* Chỉ 3 bit thấp */
    }

    /* Bỏ qua lỗi EF ở đây vì chính việc đọc error reg có thể trigger EF */
    if (status == AS5048A_ERROR_FLAG) return AS5048A_OK;
    return status;
}
