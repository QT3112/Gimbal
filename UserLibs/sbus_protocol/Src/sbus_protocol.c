/**
 ******************************************************************************
 * @file    sbus_protocol.c
 * @brief   Thư viện giải mã giao thức SBUS cho STM32 (HAL + DMA)
 *
 * Tham khảo giao thức:
 *   - SBUS specification: Futaba / FrSky / TBS Crossfire
 *   - Frame: 25 bytes, 100000 baud, 8E2
 *   - Kênh: 16 x 11-bit analog + 2 digital + flags
 *
 * Kiến trúc nhận DMA:
 *   - Dùng HAL_UARTEx_ReceiveToIdle_DMA() — tự restart khi Idle Line detected
 *   - Double-buffer: 2 x SBUS_FRAME_SIZE = 50 bytes
 *   - SBUS_RxEventCallback() được gọi từ ISR → set flag → SBUS_Process() parse
 *
 * Giải mã 16 kênh từ 22 bytes dữ liệu:
 *   Mỗi kênh 11-bit được đóng gói theo thứ tự LSB-first trong các byte dữ liệu.
 *   Byte [1]...[22] → bit-pack 16 kênh × 11 bit = 176 bit = 22 byte
 *
 ******************************************************************************
 */

#include "sbus_protocol.h"

/* =========================================================================
 * PRIVATE HELPERS
 * ========================================================================= */

/**
 * @brief Clamp float trong khoảng [lo, hi]
 */
static inline float sbus_clampf(float val, float lo, float hi) {
  if (val < lo)
    return lo;
  if (val > hi)
    return hi;
  return val;
}

/**
 * @brief Kiểm tra frame có hợp lệ không (start/end byte + frame size)
 *
 * @param buf  Con trỏ đến buffer dữ liệu
 * @param len  Độ dài buffer
 * @retval 1 nếu hợp lệ, 0 nếu không
 */
static uint8_t sbus_is_valid_frame(const uint8_t *buf, uint16_t len) {
  if (len < SBUS_FRAME_SIZE) {
    return 0;
  }
  if (buf[0] != SBUS_START_BYTE) {
    return 0;
  }
  if (buf[SBUS_FRAME_SIZE - 1] != SBUS_END_BYTE) {
    return 0;
  }
  return 1;
}

/**
 * @brief Giải mã 22 bytes dữ liệu SBUS thành 16 kênh 11-bit
 *
 * Thuật toán: Mỗi kênh 11-bit được bit-pack liên tiếp theo thứ tự LSB-first.
 * Tổng: 16 × 11 = 176 bit = 22 byte (byte [1] đến [22] trong frame).
 *
 * @param  data     Con trỏ đến byte [1] của frame (bỏ qua start byte)
 * @param  channels Mảng 16 phần tử nhận giá trị kênh (0..2047)
 */
static void sbus_decode_channels(const uint8_t *data, uint16_t *channels) {
  /*
   * Giải thích bit-packing:
   * Channel 1 : bits  0..10  → data[0] bits[7:0], data[1] bits[2:0]
   * Channel 2 : bits 11..21  → data[1] bits[7:3], data[2] bits[5:0]
   * Channel 3 : bits 22..32  → data[2] bits[7:6], data[3] bits[7:0], data[4]
   * bit[0]
   * ...dst
   */
  channels[0] = ((uint16_t)data[0] | ((uint16_t)data[1] << 8)) & 0x07FF;
  channels[1] = ((uint16_t)data[1] >> 3 | ((uint16_t)data[2] << 5)) & 0x07FF;
  channels[2] = ((uint16_t)data[2] >> 6 | ((uint16_t)data[3] << 2) |
                 ((uint16_t)data[4] << 10)) &
                0x07FF;
  channels[3] = ((uint16_t)data[4] >> 1 | ((uint16_t)data[5] << 7)) & 0x07FF;
  channels[4] = ((uint16_t)data[5] >> 4 | ((uint16_t)data[6] << 4)) & 0x07FF;
  channels[5] = ((uint16_t)data[6] >> 7 | ((uint16_t)data[7] << 1) |
                 ((uint16_t)data[8] << 9)) &
                0x07FF;
  channels[6] = ((uint16_t)data[8] >> 2 | ((uint16_t)data[9] << 6)) & 0x07FF;
  channels[7] = ((uint16_t)data[9] >> 5 | ((uint16_t)data[10] << 3)) & 0x07FF;
  channels[8] = ((uint16_t)data[11] | ((uint16_t)data[12] << 8)) & 0x07FF;
  channels[9] = ((uint16_t)data[12] >> 3 | ((uint16_t)data[13] << 5)) & 0x07FF;
  channels[10] = ((uint16_t)data[13] >> 6 | ((uint16_t)data[14] << 2) |
                  ((uint16_t)data[15] << 10)) &
                 0x07FF;
  channels[11] = ((uint16_t)data[15] >> 1 | ((uint16_t)data[16] << 7)) & 0x07FF;
  channels[12] = ((uint16_t)data[16] >> 4 | ((uint16_t)data[17] << 4)) & 0x07FF;
  channels[13] = ((uint16_t)data[17] >> 7 | ((uint16_t)data[18] << 1) |
                  ((uint16_t)data[19] << 9)) &
                 0x07FF;
  channels[14] = ((uint16_t)data[19] >> 2 | ((uint16_t)data[20] << 6)) & 0x07FF;
  channels[15] = ((uint16_t)data[20] >> 5 | ((uint16_t)data[21] << 3)) & 0x07FF;
}

/**
 * @brief Parse và cập nhật toàn bộ trường của SBUS handle từ một frame hợp lệ
 *
 * @param  hsbus  Con trỏ đến SBUS_Handle_t
 * @param  buf    Con trỏ đến đầu frame (byte[0] = 0x0F)
 */
static void sbus_parse_frame(SBUS_Handle_t *hsbus, const uint8_t *buf) {
  /* Sao chép frame vào buffer nội bộ */
  memcpy(hsbus->frame, buf, SBUS_FRAME_SIZE);

  /* Giải mã 16 kênh analog từ byte [1]..[22] */
  sbus_decode_channels(&buf[1], hsbus->channel);

  /* Giải mã Flags byte [23] */
  hsbus->flags = buf[23];
  hsbus->ch17 = (buf[23] & SBUS_FLAG_CH17) ? 1U : 0U;
  hsbus->ch18 = (buf[23] & SBUS_FLAG_CH18) ? 1U : 0U;
  hsbus->frame_lost = (buf[23] & SBUS_FLAG_FRAME_LOST) ? 1U : 0U;
  hsbus->failsafe = (buf[23] & SBUS_FLAG_FAILSAFE) ? 1U : 0U;

  /* Cập nhật timestamp và trạng thái */
  hsbus->last_frame_tick = HAL_GetTick();
  hsbus->new_data = 1U;

  /* Xác định trạng thái ưu tiên */
  if (hsbus->failsafe) {
    hsbus->status = SBUS_FAILSAFE;
  } else if (hsbus->frame_lost) {
    hsbus->status = SBUS_FRAME_LOST;
  } else {
    hsbus->status = SBUS_OK;
  }
}

/* =========================================================================
 * PUBLIC IMPLEMENTATION
 * ========================================================================= */

/**
 * @brief Khởi tạo thư viện SBUS và bắt đầu nhận DMA qua Idle Line Detection
 */
SBUS_Status_t SBUS_Init(SBUS_Handle_t *hsbus, UART_HandleTypeDef *huart) {
  if (hsbus == NULL || huart == NULL) {
    return SBUS_ERROR;
  }

  /* Reset toàn bộ handle */
  memset(hsbus, 0, sizeof(SBUS_Handle_t));

  /* Gán UART handle */
  hsbus->huart = huart;

  /* Giá trị mặc định kênh = mid-point khi chưa có tín hiệu */
  for (uint8_t i = 0; i < SBUS_NUM_CHANNELS; i++) {
    hsbus->channel[i] = SBUS_RAW_MID;
  }

  hsbus->status = SBUS_NOT_INIT;
  hsbus->last_frame_tick = HAL_GetTick();

  /* Khoi tao State Machine */
  hsbus->dma_read_ptr = 0;
  hsbus->parse_state = 0;
  hsbus->parse_idx = 0;

  /*
   * Bat dau nhan DMA Circular (lien tuc vong tron, khong quan tam Idle Line)
   */
  HAL_StatusTypeDef hal_ret = HAL_UART_Receive_DMA(hsbus->huart, hsbus->dma_buf, SBUS_DMA_BUF_SIZE);

  if (hal_ret != HAL_OK) {
    return SBUS_ERROR;
  }

  /* Tat TAT CA cac ngat DMA vi chung ta se chu dong kiem tra NDTR 
   * Tranh viec ngat DMA lam ton tai nguyen CPU vo ich */
  __HAL_DMA_DISABLE_IT(hsbus->huart->hdmarx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(hsbus->huart->hdmarx, DMA_IT_TC);
  __HAL_DMA_DISABLE_IT(hsbus->huart->hdmarx, DMA_IT_TE);

  hsbus->is_initialized = 1U;
  hsbus->status = SBUS_TIMEOUT; /* Chua co frame → timeout */

  return SBUS_OK;
}

/**
 * @brief Xử lý dữ liệu DMA — gọi trong vòng lặp chính (Polling NDTR)
 *
 * Hàm này đọc thanh ghi NDTR của DMA để biết phần cứng đã ghi đến đâu,
 * sau đó đẩy từng byte vào State Machine để tìm và parse frame SBUS.
 * Cach tiep can nay KHONG dung ngat (Interrupt-free), mien nhiem voi loi.
 */
SBUS_Status_t SBUS_Process(SBUS_Handle_t *hsbus) {
  if (!hsbus->is_initialized) {
    return SBUS_NOT_INIT;
  }

  /* Kiểm tra timeout mất tín hiệu (Fail-Safe qua phan mem) */
  if ((HAL_GetTick() - hsbus->last_frame_tick) > SBUS_TIMEOUT_MS) {
    hsbus->status = SBUS_TIMEOUT;
  }

  /* 
   * Tính toán vị trí Write Pointer hien tai cua DMA phan cung.
   * NDTR luu so byte CON LAI. Vị tri hien tai = Kich thuoc Tong - NDTR.
   */
  uint16_t current_dma_ptr = SBUS_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(hsbus->huart->hdmarx);
  
  /* Phat hien va xu ly Wrap-around (khi NDTR = 0 hoac pointer o bien) */
  if (current_dma_ptr >= SBUS_DMA_BUF_SIZE) {
      current_dma_ptr = 0;
  }

  /* Đọc tung byte moi tu Buffer DMA -> dua vao State Machine */
  while (hsbus->dma_read_ptr != current_dma_ptr) {
    uint8_t c = hsbus->dma_buf[hsbus->dma_read_ptr];
    hsbus->dma_read_ptr = (hsbus->dma_read_ptr + 1) % SBUS_DMA_BUF_SIZE;

    if (hsbus->parse_state == 0) {
      /* STATE 0: Doi Byte Bat Dau (Start Byte) */
      if (c == SBUS_START_BYTE) {
        hsbus->parse_buf[0] = c;
        hsbus->parse_idx = 1;
        hsbus->parse_state = 1; /* Chuyen sang nhan du lieu */
      }
    } else {
      /* STATE 1: Gom du lieu Frame */
      hsbus->parse_buf[hsbus->parse_idx++] = c;
      
      /* Khi da nhan du 25 bytes cua 1 frame */
      if (hsbus->parse_idx == SBUS_FRAME_SIZE) {
        hsbus->parse_state = 0; /* Reset state cho frame tiep theo */
        
        if (c == SBUS_END_BYTE) {
          /* Frame hop le! Parse data. */
          sbus_parse_frame(hsbus, hsbus->parse_buf);
        }
      }
    }
  }

  return hsbus->status;
}

/**
 * @brief Lấy giá trị raw 11-bit của một kênh
 */
SBUS_Status_t SBUS_GetChannel(SBUS_Handle_t *hsbus, uint8_t ch,
                              uint16_t *value) {
  if (!hsbus->is_initialized || value == NULL) {
    return SBUS_ERROR;
  }
  if (ch < 1U || ch > SBUS_NUM_CHANNELS) {
    return SBUS_ERROR;
  }
  *value = hsbus->channel[ch - 1U];
  return SBUS_OK;
}

/**
 * @brief Lấy giá trị normalized float [-1.0 .. +1.0]
 *
 * Công thức:
 *   norm = (raw - SBUS_RAW_MID) / (SBUS_RAW_MAX - SBUS_RAW_MID)
 *        = (raw - 992)         / (1811 - 992)
 *        = (raw - 992)         / 819.0f
 */
SBUS_Status_t SBUS_GetChannelNorm(SBUS_Handle_t *hsbus, uint8_t ch,
                                  float *norm) {
  uint16_t raw = 0;
  SBUS_Status_t ret = SBUS_GetChannel(hsbus, ch, &raw);
  if (ret != SBUS_OK) {
    return ret;
  }

  float val = ((float)raw - (float)SBUS_RAW_MID) /
              ((float)SBUS_RAW_MAX - (float)SBUS_RAW_MID);
  *norm = sbus_clampf(val, -1.0f, 1.0f);
  return SBUS_OK;
}

/**
 * @brief Lấy giá trị normalized float [0.0 .. 1.0]
 *
 * Công thức:
 *   norm01 = (raw - SBUS_RAW_MIN) / (SBUS_RAW_MAX - SBUS_RAW_MIN)
 *          = (raw - 172)          / (1811 - 172)
 *          = (raw - 172)          / 1639.0f
 */
SBUS_Status_t SBUS_GetChannelNorm01(SBUS_Handle_t *hsbus, uint8_t ch,
                                    float *norm) {
  uint16_t raw = 0;
  SBUS_Status_t ret = SBUS_GetChannel(hsbus, ch, &raw);
  if (ret != SBUS_OK) {
    return ret;
  }

  float val = ((float)raw - (float)SBUS_RAW_MIN) /
              ((float)SBUS_RAW_MAX - (float)SBUS_RAW_MIN);
  *norm = sbus_clampf(val, 0.0f, 1.0f);
  return SBUS_OK;
}

/**
 * @brief Lấy trạng thái kênh digital CH17
 */
uint8_t SBUS_GetCH17(SBUS_Handle_t *hsbus) { return hsbus->ch17; }

/**
 * @brief Lấy trạng thái kênh digital CH18
 */
uint8_t SBUS_GetCH18(SBUS_Handle_t *hsbus) { return hsbus->ch18; }

/**
 * @brief Lấy trạng thái hiện tại
 */
SBUS_Status_t SBUS_GetStatus(SBUS_Handle_t *hsbus) {
  if (!hsbus->is_initialized) {
    return SBUS_NOT_INIT;
  }
  return hsbus->status;
}

/**
 * @brief Kiểm tra Failsafe
 */
uint8_t SBUS_IsFailsafe(SBUS_Handle_t *hsbus) { return hsbus->failsafe; }

/**
 * @brief Kiểm tra mất tín hiệu
 */
uint8_t SBUS_IsSignalLost(SBUS_Handle_t *hsbus) {
  if (!hsbus->is_initialized) {
    return 1U;
  }
  return ((HAL_GetTick() - hsbus->last_frame_tick) > SBUS_TIMEOUT_MS) ? 1U : 0U;
}

/**
 * @brief Callback từ HAL_UARTEx_RxEventCallback (Khong con su dung)
 *
 * Do he thong da chuyen sang Polling NDTR State Machine, cac ngat Idle Line
 * da bi loai bo. Ham nay duoc de lai duoi dang ham rong de tranh loi bien dich
 * neu user code (main.c) lo goi.
 */
void SBUS_RxEventCallback(SBUS_Handle_t *hsbus, uint16_t size) {
  (void)hsbus;
  (void)size;
  /* Nothing to do */
}

/**
 * @brief Callback từ HAL_UART_RxCpltCallback (Khong con su dung)
 */
void SBUS_RxCpltCallback(SBUS_Handle_t *hsbus) {
  (void)hsbus;
  /* Nothing to do */
}
