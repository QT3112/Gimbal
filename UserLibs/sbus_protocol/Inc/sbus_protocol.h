/**
 ******************************************************************************
 * @file    sbus_protocol.h
 * @brief   Thư viện giải mã giao thức SBUS cho STM32 (HAL + DMA)
 *
 * Giao thức SBUS (Futaba / FrSky):
 *   - Baud Rate  : 100000 bps
 *   - UART Config: 8E2 (8-bit data, Even Parity, 2 Stop bits)
 *   - Frame Size : 25 bytes
 *   - Chu kỳ    : ~14ms (FrSky) hoặc ~7ms (High-Speed)
 *
 * Cấu trúc frame SBUS 25 bytes:
 *   [0]    : Start Byte  = 0x0F
 *   [1..22]: 22 bytes chứa 16 kênh x 11 bit (tổng 176 bit)
 *   [23]   : Flags Byte  (Digital CH17, CH18, Frame Lost, Failsafe)
 *   [24]   : End Byte    = 0x00
 *
 * Tính năng thư viện:
 *   - Nhận frame qua UART DMA (không block CPU)
 *   - Giải mã 16 kênh analog (giá trị 0..2047, midpoint = 1024)
 *   - Đọc 2 kênh digital (CH17, CH18)
 *   - Phát hiện Frame Lost & Failsafe
 *   - Phát hiện timeout mất tín hiệu receiver
 *   - Mapping kênh sang normalized float [-1.0 .. +1.0] hoặc [0.0 .. 1.0]
 *
 * Cách sử dụng:
 *   1. Cấu hình UART: 100000 baud, 8E2, Enable DMA RX Circular
 *   2. Gọi SBUS_Init() với UART handle
 *   3. Gọi SBUS_RxEventCallback() từ HAL_UARTEx_RxEventCallback ISR
 *   4. Gọi SBUS_Process() trong vòng lặp chính
 *   5. Đọc dữ liệu kênh qua SBUS_GetChannel() / SBUS_GetChannelNorm()
 *
 * @note  UART cần được cấu hình ở 100000 baud, 8E2 (NOT 8N1!)
 *        Trong STM32CubeMX: WordLength=8, StopBits=2, Parity=Even
 ******************************************************************************
 */

#ifndef SBUS_PROTOCOL_H
#define SBUS_PROTOCOL_H

#include <stdint.h>
#include <string.h>
#include "stm32g4xx_hal.h"

/* =========================================================================
 * CONSTANTS & CONFIGURATION
 * ========================================================================= */

/** So byte moi frame SBUS */
#define SBUS_FRAME_SIZE         25U

/** Start byte cua frame SBUS */
#define SBUS_START_BYTE         0x0FU

/** End byte cua frame SBUS */
#define SBUS_END_BYTE           0x00U

/** So kenh analog trong mot frame SBUS */
#define SBUS_NUM_CHANNELS       16U

/** Gia tri raw toi thieu / toi da / trung tam cua kenh SBUS (11-bit) */
#define SBUS_RAW_MIN            172U
#define SBUS_RAW_MAX            1811U
#define SBUS_RAW_MID            992U

/** Gia tri thuc te kenh co the la 0..2047 */
#define SBUS_VALUE_MIN          0U
#define SBUS_VALUE_MAX          2047U

/** Bit mask trong Flags byte */
#define SBUS_FLAG_CH17          (1U << 0)   /**< Digital Channel 17     */
#define SBUS_FLAG_CH18          (1U << 1)   /**< Digital Channel 18     */
#define SBUS_FLAG_FRAME_LOST    (1U << 2)   /**< Frame Lost (1 = mat frame) */
#define SBUS_FLAG_FAILSAFE      (1U << 3)   /**< Failsafe Active            */

/** Timeout phat hien mat tin hieu SBUS (ms) */
#define SBUS_TIMEOUT_MS         500U

/** Kich thuoc DMA buffer (2 frame de tranh overwrite khi parse) */
#define SBUS_DMA_BUF_SIZE       (SBUS_FRAME_SIZE * 2U)

/* =========================================================================
 * TYPE DEFINITIONS
 * ========================================================================= */

/**
 * @brief Trang thai cua thu vien SBUS
 */
typedef enum {
    SBUS_OK          = 0,   /**< Hoat dong binh thuong, du lieu hop le */
    SBUS_ERROR       = 1,   /**< Loi chung (UART, DMA)                 */
    SBUS_FRAME_LOST  = 2,   /**< Frame Lost flag duoc set              */
    SBUS_FAILSAFE    = 3,   /**< Failsafe dang active                  */
    SBUS_TIMEOUT     = 4,   /**< Mat tin hieu qua SBUS_TIMEOUT_MS       */
    SBUS_NOT_INIT    = 5,   /**< Chua duoc khoi tao                    */
} SBUS_Status_t;

/**
 * @brief Handle chua toan bo trang thai cua mot ket noi SBUS
 *
 * Khai bao bien kieu nay trong file cua ban, sau do truyen con tro vao
 * tat ca cac ham SBUS_xxx().
 *
 * Vi du:
 * @code
 *   SBUS_Handle_t sbus_rx;
 *   SBUS_Init(&sbus_rx, &huart1);
 * @endcode
 */
typedef struct {
    UART_HandleTypeDef *huart;            /**< Con tro den UART handle cua HAL      */
    
    uint8_t  dma_buf[SBUS_DMA_BUF_SIZE]; /**< DMA receive buffer (ghi boi DMA HW)  */
    uint8_t  frame[SBUS_FRAME_SIZE];     /**< Frame SBUS da duoc xac nhan          */
    uint8_t  parse_buf[SBUS_FRAME_SIZE]; /**< Buffer tam thoi cho State Machine    */

    uint16_t channel[SBUS_NUM_CHANNELS]; /**< Gia tri raw 16 kenh (0..2047)       */
    uint8_t  ch17;                       /**< Digital channel 17 (0 hoac 1)        */
    uint8_t  ch18;                       /**< Digital channel 18 (0 hoac 1)        */
    uint8_t  flags;                      /**< Flags byte nguyen ban tu receiver    */

    uint8_t  frame_lost;                 /**< 1 neu flag Frame Lost dang set       */
    uint8_t  failsafe;                   /**< 1 neu Failsafe dang active           */

    uint32_t last_frame_tick;            /**< Tick (ms) lan nhan frame hop le cuoi */
    uint8_t  is_initialized;             /**< Flag khoi tao thanh cong             */
    uint8_t  new_data;                   /**< 1 = co frame moi chua duoc doc       */

    /* Internal State Machine variables */
    uint16_t dma_read_ptr;               /**< Con tro doc phan mem cho Ring Buffer */
    uint8_t  parse_state;                /**< Trang thai parse (0=Wait, 1=Data)    */
    uint8_t  parse_idx;                  /**< Vi tri byte dang parse               */

    SBUS_Status_t status;               /**< Trang thai hien tai                  */
} SBUS_Handle_t;

/* =========================================================================
 * PUBLIC API
 * ========================================================================= */

/**
 * @brief Khoi tao thu vien SBUS va bat dau nhan DMA Circular
 *
 * @note  UART phai duoc cau hinh truoc: 100000 baud, 9 Bits (incl Parity), Even, 2 Stop.
 *        DMA RX phai la Circular Mode.
 *
 * @param  hsbus  Con tro den SBUS_Handle_t can khoi tao
 * @param  huart  Con tro den UART_HandleTypeDef da duoc HAL khoi tao
 * @retval SBUS_OK neu thanh cong, SBUS_ERROR neu that bai
 */
SBUS_Status_t SBUS_Init(SBUS_Handle_t *hsbus, UART_HandleTypeDef *huart);

/**
 * @brief Xu ly du lieu DMA — can goi dinh ky trong vong lap chinh
 *
 * Ham nay kiem tra DMA buffer, tim frame SBUS hop le, giai ma va cap nhat
 * tat ca cac truong trong SBUS_Handle_t.
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @retval SBUS_OK neu co frame moi hop le
 *         SBUS_FRAME_LOST neu flag frame lost duoc set
 *         SBUS_FAILSAFE neu failsafe dang active
 *         SBUS_TIMEOUT neu mat tin hieu
 */
SBUS_Status_t SBUS_Process(SBUS_Handle_t *hsbus);

/**
 * @brief Lay gia tri raw 11-bit cua mot kenh (0..2047)
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @param  ch     So kenh (1..16)
 * @param  value  Con tro nhan gia tri raw
 * @retval SBUS_OK neu hop le, SBUS_ERROR neu ch ngoai pham vi
 */
SBUS_Status_t SBUS_GetChannel(SBUS_Handle_t *hsbus, uint8_t ch, uint16_t *value);

/**
 * @brief Lay gia tri normalized float cua mot kenh [-1.0 .. +1.0]
 *
 * Mapping: SBUS_RAW_MIN..SBUS_RAW_MAX -> [-1.0 .. +1.0]
 * Gia tri ngoai dai se bi clamp vao [-1.0 .. +1.0].
 * Dung cho: Aileron, Elevator, Rudder, Roll, Pitch, Yaw
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @param  ch     So kenh (1..16)
 * @param  norm   Con tro nhan gia tri float [-1.0 .. +1.0]
 * @retval SBUS_OK neu hop le
 */
SBUS_Status_t SBUS_GetChannelNorm(SBUS_Handle_t *hsbus, uint8_t ch, float *norm);

/**
 * @brief Lay gia tri normalized float [0.0 .. 1.0] (dung cho throttle/gimbal)
 *
 * Mapping: SBUS_RAW_MIN..SBUS_RAW_MAX -> [0.0 .. 1.0]
 * Dung cho: Throttle, Gimbal Tilt
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @param  ch     So kenh (1..16)
 * @param  norm   Con tro nhan gia tri float [0.0 .. 1.0]
 * @retval SBUS_OK neu hop le
 */
SBUS_Status_t SBUS_GetChannelNorm01(SBUS_Handle_t *hsbus, uint8_t ch, float *norm);

/**
 * @brief Lay trang thai kenh digital CH17
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @retval 0 hoac 1
 */
uint8_t SBUS_GetCH17(SBUS_Handle_t *hsbus);

/**
 * @brief Lay trang thai kenh digital CH18
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @retval 0 hoac 1
 */
uint8_t SBUS_GetCH18(SBUS_Handle_t *hsbus);

/**
 * @brief Kiem tra trang thai hien tai cua ket noi SBUS
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @retval SBUS_OK / SBUS_FRAME_LOST / SBUS_FAILSAFE / SBUS_TIMEOUT / SBUS_NOT_INIT
 */
SBUS_Status_t SBUS_GetStatus(SBUS_Handle_t *hsbus);

/**
 * @brief Kiem tra xem receiver co dang trong che do Failsafe khong
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @retval 1 = dang failsafe, 0 = binh thuong
 */
uint8_t SBUS_IsFailsafe(SBUS_Handle_t *hsbus);

/**
 * @brief Kiem tra xem co dang mat tin hieu (timeout) khong
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @retval 1 = mat tin hieu, 0 = binh thuong
 */
uint8_t SBUS_IsSignalLost(SBUS_Handle_t *hsbus);

/**
 * @brief Callback — Goi tu HAL_UARTEx_RxEventCallback khi DMA nhan xong
 *
 * Can goi ham nay trong HAL_UARTEx_RxEventCallback() (stm32g4xx_it.c / main.c).
 *
 * Vi du:
 * @code
 * extern SBUS_Handle_t sbus_rx;
 *
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
 *     if (huart->Instance == USART1) {
 *         SBUS_RxEventCallback(&sbus_rx, Size);
 *     }
 * }
 * @endcode
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 * @param  size   So byte da nhan (tu HAL callback)
 */
void SBUS_RxEventCallback(SBUS_Handle_t *hsbus, uint16_t size);

/**
 * @brief Callback — Goi tu HAL_UART_RxCpltCallback (khi khong dung Idle Line DMA)
 *
 * @param  hsbus  Con tro den SBUS_Handle_t
 */
void SBUS_RxCpltCallback(SBUS_Handle_t *hsbus);

#endif /* SBUS_PROTOCOL_H */
