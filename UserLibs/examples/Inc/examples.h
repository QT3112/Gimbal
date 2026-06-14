/**
 ******************************************************************************
 * @file    examples.h
 * @brief   Các hàm ví dụ điều khiển BLDC Motor qua FOC + AS5048A Encoder
 *
 * === CÁCH SỬ DỤNG ===
 *
 *  1. Trong main.c, thực hiện đầy đủ các bước khởi tạo:
 *       FOC_Init() → AS5048A_Init() → FOC_AlignD() → FOC_CalibrateAngle()
 *
 *  2. Gọi FOC_Start() để bật output.
 *
 *  3. Gọi một trong các hàm Example trong vòng lặp while(1):
 *       Example_RunConstantVelocity()  — xoay tốc độ cố định
 *       Example_HoldAngle()            — giữ góc cố định
 *
 * === VÍ DỤ (main.c) ===
 *
 *  // Xoay 60 RPM chiều thuận:
 *  Example_RunConstantVelocity(&foc, &encoder, 60.0f * (2*PI / 60.0f));
 *  HAL_Delay(10);
 *
 *  // Giữ tại góc 90 độ:
 *  Example_HoldAngle(&foc, &encoder, 90.0f * (PI / 180.0f));
 *  HAL_Delay(10);
 *
 ******************************************************************************
 */

#ifndef EXAMPLES_H
#define EXAMPLES_H

#ifdef __cplusplus
extern "C" {
#endif

#include "foc.h"
#include "as5048a.h"

/* ===========================================================================
 * Cấu trúc cấu hình cho các hàm Example
 * Cho phép tùy chỉnh PID và LPF mà không cần thay đổi code bên trong hàm.
 * =========================================================================== */
typedef struct {
    /* PID vòng vận tốc (vòng trong - dùng cho cả 2 example) */
    float vel_Kp;        /*!< Proportional gain vòng vận tốc */
    float vel_Ki;        /*!< Integral gain vòng vận tốc */
    float vel_Kd;        /*!< Derivative gain vòng vận tốc */
    float vel_lpf_alpha; /*!< Hệ số LPF cho tốc độ encoder (0~1, gần 1 = lọc mạnh) */

    /* PID vòng vị trí (chỉ dùng cho Example_HoldAngle) */
    float pos_Kp;        /*!< Proportional gain vòng vị trí */
    float pos_Ki;        /*!< Integral gain vòng vị trí */
    float pos_Kd;        /*!< Derivative gain vòng vị trí */
} ExampleConfig_t;

/* ===========================================================================
 * Cấu hình mặc định gợi ý cho motor 12N14P 160KV
 * Có thể dùng làm điểm khởi đầu, sau đó chỉnh thủ công theo phần cứng thực tế.
 * =========================================================================== */
#define EXAMPLE_DEFAULT_CONFIG {    \
    .vel_Kp        = 0.05f,         \
    .vel_Ki        = 0.01f,         \
    .vel_Kd        = 0.0f,          \
    .vel_lpf_alpha = 0.9f,          \
    .pos_Kp        = 5.0f,          \
    .pos_Ki        = 0.0f,          \
    .pos_Kd        = 0.05f,         \
}

/* ===========================================================================
 * EXAMPLE 1: Điều khiển motor xoay tốc độ cố định (Closed-loop Velocity)
 *
 * Motor sẽ được điều khiển để duy trì tốc độ quay mục tiêu, bù lại biến động
 * tải và điện áp nguồn cấp nhờ vòng lặp PID.
 *
 * Sơ đồ điều khiển:
 *   [target_vel] ──►  [PID Velocity] ──► Vq ──► [FOC: InvPark+InvClarke] ──► PWM
 *                             ▲
 *                     [Encoder] → dθ/dt → [LPF] → velocity_mech
 *
 * @param  hfoc         Con trỏ FOC handle (đã FOC_Start())
 * @param  henc         Con trỏ Encoder handle AS5048A
 * @param  target_vel   Tốc độ cơ học mục tiêu [rad/s]
 *                        Dương: chiều thuận | Âm: chiều ngược
 *                        Ví dụ: 60 RPM = 60 * (2*PI/60) = 6.28 rad/s
 *
 * @note  Gọi hàm này liên tục trong while(1) với HAL_Delay(10) phía sau.
 *        Không gọi FOC_SetPID_Vel / FOC_SetLPF_Vel bên trong vòng lặp,
 *        chỉ gọi 1 lần lúc khởi tạo.
 * =========================================================================== */
void Example_RunConstantVelocity(FOC_Handle_t *hfoc,
                                  AS5048A_Handle_t *henc,
                                  float target_vel);

/* ===========================================================================
 * EXAMPLE 2: Điều khiển motor giữ góc cố định (Cascade Position Control)
 *
 * Motor sẽ chủ động kháng cự ngoại lực để duy trì góc mục tiêu.
 * Sử dụng kiến trúc Cascade PID (Vòng vị trí → Vòng vận tốc → FOC).
 *
 * Sơ đồ điều khiển:
 *   [target_angle] ──► [PID Position] ──► target_vel
 *                              ▲                │
 *                   [Encoder angle]      [PID Velocity] ──► Vq ──► FOC ──► PWM
 *                                                ▲
 *                                        [Encoder] → dθ/dt → [LPF]
 *
 * @param  hfoc          Con trỏ FOC handle (đã FOC_Start())
 * @param  henc          Con trỏ Encoder handle AS5048A
 * @param  target_angle  Góc cơ học mục tiêu [rad], range [0, 2π)
 *                         Ví dụ: 90 độ = 90 * (PI / 180.0f) = 1.5708 rad
 *
 * @note  Gọi hàm này liên tục trong while(1) với HAL_Delay(10) phía sau.
 *        Nếu motor rung khi đạt đích → giảm pos_Kp hoặc tăng pos_Kd.
 *        Nếu motor phản hồi chậm → tăng pos_Kp từ từ.
 * =========================================================================== */
void Example_HoldAngle(FOC_Handle_t *hfoc,
                        AS5048A_Handle_t *henc,
                        float target_angle);

/* ===========================================================================
 * Khởi tạo PID và LPF cho các Example theo cấu hình người dùng truyền vào.
 * Gọi hàm này 1 LẦN trong phần Setup (USER CODE BEGIN 2), TRƯỚC FOC_Start().
 *
 * @param  hfoc   Con trỏ FOC handle
 * @param  cfg    Con trỏ ExampleConfig_t chứa thông số PID/LPF mong muốn
 * =========================================================================== */
void Example_Init(FOC_Handle_t *hfoc, const ExampleConfig_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* EXAMPLES_H */
