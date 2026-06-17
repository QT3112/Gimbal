/**
 ******************************************************************************
 * @file    manage_code.h
 * @brief   Quản lý vòng điều khiển FOC chạy qua ngắt TIM6
 *
 * Kiến trúc:
 *   main()      → App_Init()    (1 lần, trước while(1))
 *   TIM6 ISR    → FOC_Loop()    (mỗi 10ms = 100Hz)
 *   while(1)    → đọc foc_log, gọi printf (KHÔNG printf trong ISR)
 ******************************************************************************
 */

#ifndef MANAGE_CODE_H
#define MANAGE_CODE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "as5048a.h"
#include "foc.h"
#include <stdint.h>

/* ===========================================================================
 * Biến chia sẻ (extern) — khai báo ở manage_code.c, dùng trong main.c
 * =========================================================================== */
extern FOC_Handle_t     foc;
extern AS5048A_Handle_t encoder;
extern volatile uint8_t encoder_ready;

/* ===========================================================================
 * Log buffer — ISR ghi, while(1) đọc và printf
 * KHÔNG bao giờ printf trực tiếp trong FOC_Loop()!
 * =========================================================================== */
typedef struct {
    float target_vel;  /* Tốc độ đặt [rad/s] */
    float meas_vel;    /* Tốc độ đo sau LPF [rad/s] */
    float Vq;          /* Điện áp trục q [V] */
    float angle_deg;   /* Góc encoder [°] */
} FOC_LogData_t;

extern volatile uint8_t      foc_log_ready; /* 1 = ISR đã ghi log mới */
extern volatile FOC_LogData_t foc_log;      /* Buffer log từ ISR */

/* ===========================================================================
 * API
 * =========================================================================== */

/**
 * @brief  Khởi tạo phần cứng và thuật toán (PWM, Encoder, FOC, Alignment)
 *         Gọi 1 lần trong USER CODE BEGIN 2 của main.c
 */
void App_Init(void);

/**
 * @brief  Vòng lặp FOC — gọi trong ngắt TIM6 mỗi 10ms
 *
 *   Gọi từ main.c:
 *     void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
 *         if (htim->Instance == TIM6) FOC_Loop();
 *     }
 *
 * QUY TẮC:
 *   - KHÔNG printf()  → gây block ISR, mất timing
 *   - KHÔNG HAL_Delay()
 *   - Thực hiện: đọc encoder SPI + FOC_RunVelocity() + ghi foc_log
 */
void FOC_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* MANAGE_CODE_H */
