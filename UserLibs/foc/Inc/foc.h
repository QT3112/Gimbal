/**
 ******************************************************************************
 * @file    foc.h
 * @brief   Thư viện Field Oriented Control (FOC) cho động cơ BLDC/PMSM
 *
 * === KIẾN TRÚC FOC ===
 *
 *  [Encoder]──> angle_mech ──> angle_elec
 *                                    │
 *  [Setpoint] ──> Iq_ref             ▼
 *                          ┌─────────────────┐
 *   Id_ref=0 ──────────────►  Park Inverse   ◄──── [PID_d] ◄── Id (from Park)
 *                          │  (dq → αβ)      ◄──── [PID_q] ◄── Iq (from Park)
 *                          └────────┬────────┘
 *                                   │ Vα, Vβ
 *                          ┌────────▼────────┐
 *                          │ Clarke Inverse  │
 *                          │ (αβ → abc)      │
 *                          └────────┬────────┘
 *                                   │ Ua, Ub, Uc
 *                          ┌────────▼────────┐
 *                          │   TIM PWM       │
 *                          └─────────────────┘
 *
 * === BIẾN ĐỔI ĐƯỢC SỬ DỤNG ===
 *
 *  Clarke:  (Ia, Ib, Ic) → (Iα, Iβ)    [abc → stationary frame]
 *  Park:    (Iα, Iβ)     → (Id, Iq)    [stationary → rotating frame]
 *  InvPark: (Vd, Vq)     → (Vα, Vβ)   [rotating → stationary frame]
 *  InvClarke:(Vα, Vβ)   → (Ua,Ub,Uc)  [stationary → abc]
 *
 * === CHÚ THÍCH PHẦN CỨNG (dự án Gimbal) ===
 *  PWM Timer:   TIM2, Channel 1/2/3
 *  PWM Period:  4249 (ARR)
 *  Encoder:     AS5048A (14-bit, SPI1)
 *  Pole pairs:  Cấu hình tại khởi tạo (mặc định 7)
 ******************************************************************************
 */

#ifndef FOC_H
#define FOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <math.h>

/* ===========================================================================
 * Hằng số toán học
 * =========================================================================== */
#define FOC_PI          3.14159265359f
#define FOC_TWO_PI      6.28318530718f
#define FOC_SQRT3       1.73205080757f
#define FOC_SQRT3_2     0.86602540378f  /* sqrt(3)/2 */
#define FOC_ONE_SQRT3   0.57735026919f  /* 1/sqrt(3) */

/* ===========================================================================
 * Cấu trúc PID Controller
 * =========================================================================== */
typedef struct {
    float Kp;           /*!< Hệ số tỉ lệ */
    float Ki;           /*!< Hệ số tích phân */
    float Kd;           /*!< Hệ số vi phân */

    float integral;     /*!< Tích lũy tích phân */
    float prev_error;   /*!< Sai số lần trước (cho vi phân) */

    float output_min;   /*!< Giới hạn output tối thiểu (anti-windup) */
    float output_max;   /*!< Giới hạn output tối đa (anti-windup) */
} FOC_PID_t;

/* ===========================================================================
 * Cấu trúc trạng thái tọa độ αβ (stationary frame)
 * =========================================================================== */
typedef struct {
    float alpha;   /*!< Thành phần trục α */
    float beta;    /*!< Thành phần trục β */
} FOC_AlphaBeta_t;

/* ===========================================================================
 * Cấu trúc trạng thái tọa độ dq (rotating frame)
 * =========================================================================== */
typedef struct {
    float d;   /*!< Thành phần trục d (flux, cần điều khiển về 0) */
    float q;   /*!< Thành phần trục q (torque, điều khiển moment) */
} FOC_DQ_t;

/* ===========================================================================
 * Cấu trúc chính của FOC Handle
 * =========================================================================== */
typedef struct {
    /* --- Phần cứng PWM --- */
    TIM_HandleTypeDef *htim;     /*!< Con trỏ TIM handle của HAL */
    uint32_t ch_a;               /*!< TIM Channel pha A (TIM_CHANNEL_1) */
    uint32_t ch_b;               /*!< TIM Channel pha B (TIM_CHANNEL_2) */
    uint32_t ch_c;               /*!< TIM Channel pha C (TIM_CHANNEL_3) */
    float    pwm_period;         /*!< Giá trị ARR của Timer (ví dụ: 4249.0f) */

    /* --- Thông số động cơ --- */
    uint8_t  pole_pairs;         /*!< Số cặp cực (pole pairs). Ví dụ: 7 */
    float    voltage_supply;     /*!< Điện áp cấp nguồn [V] (dùng để normalize) */
    float    voltage_limit;      /*!< Điện áp tối đa cho phép [V] (bảo vệ motor) */

    /* --- Góc điện --- */
    float angle_mech;            /*!< Góc cơ học từ encoder [rad], 0 đến 2π */
    float angle_elec;            /*!< Góc điện = angle_mech * pole_pairs [rad] */
    float angle_offset;          /*!< Offset hiệu chỉnh góc 0 (zero angle) [rad] */

    /* --- Setpoint điều khiển --- */
    float Vd_ref;                /*!< Điện áp trục d mong muốn [V] (thường = 0) */
    float Vq_ref;                /*!< Điện áp trục q mong muốn [V] (= torque) */

    /* --- Trạng thái biến đổi tọa độ --- */
    FOC_AlphaBeta_t V_ab;        /*!< Điện áp αβ sau Park Inverse */
    FOC_DQ_t        V_dq;        /*!< Điện áp dq (output của PID) */

    /* --- PID Controllers --- */
    FOC_PID_t pid_d;             /*!< PID trục d (điều khiển từ thông) */
    FOC_PID_t pid_q;             /*!< PID trục q (điều khiển moment) */

    /* --- Trạng thái vòng lặp --- */
    float Ts;                    /*!< Chu kỳ lấy mẫu [s] */
    uint8_t enabled;             /*!< 1 = FOC đang chạy, 0 = dừng (PWM = 50%) */
} FOC_Handle_t;

/* ===========================================================================
 * API khởi tạo
 * =========================================================================== */

/**
 * @brief  Khởi tạo FOC handle với cấu hình phần cứng
 * @param  hfoc         Con trỏ đến FOC_Handle_t
 * @param  htim         Con trỏ đến TIM handle (đã Start PWM)
 * @param  ch_a/b/c     TIM_CHANNEL_x của 3 pha
 * @param  pwm_period   Giá trị ARR (ví dụ: 4249.0f)
 * @param  pole_pairs   Số cặp cực của motor
 * @param  voltage_lim  Giới hạn điện áp [V]
 * @param  Ts           Chu kỳ điều khiển [s] (ví dụ: 0.0001f = 100µs)
 */
void FOC_Init(FOC_Handle_t *hfoc,
              TIM_HandleTypeDef *htim,
              uint32_t ch_a, uint32_t ch_b, uint32_t ch_c,
              float pwm_period,
              uint8_t pole_pairs,
              float voltage_lim,
              float Ts);

/**
 * @brief  Cấu hình PID trục D (điều khiển từ thông, thường Id_ref = 0)
 */
void FOC_SetPID_D(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max);

/**
 * @brief  Cấu hình PID trục Q (điều khiển moment/tốc độ)
 */
void FOC_SetPID_Q(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max);

/* ===========================================================================
 * API điều khiển chính
 * =========================================================================== */

/**
 * @brief  Cập nhật góc cơ học từ encoder (gọi trước FOC_Update)
 * @param  hfoc         Con trỏ FOC_Handle_t
 * @param  angle_mech_rad  Góc cơ học [rad], range 0 đến 2π
 */
void FOC_SetAngle(FOC_Handle_t *hfoc, float angle_mech_rad);

/**
 * @brief  Đặt điện áp trực tiếp theo tọa độ dq (không qua PID)
 *         Dùng cho open-loop hoặc torque feedforward
 * @param  Vd  Điện áp trục d (thường = 0)
 * @param  Vq  Điện áp trục q (tỉ lệ với moment)
 */
void FOC_SetVoltage(FOC_Handle_t *hfoc, float Vd, float Vq);

/**
 * @brief  Hàm chạy 1 chu kỳ FOC (gọi trong timer interrupt hoặc vòng lặp chính)
 *
 * Thực hiện toàn bộ pipeline:
 *   angle_elec → sin/cos → InvPark(Vd,Vq→Vα,Vβ) → InvClarke(Vα,Vβ→Ua,Ub,Uc) → PWM
 *
 * @param  hfoc  Con trỏ FOC_Handle_t
 */
void FOC_Update(FOC_Handle_t *hfoc);

/**
 * @brief  Tắt output PWM về 50% duty (trạng thái thả nổi an toàn)
 */
void FOC_Stop(FOC_Handle_t *hfoc);

/**
 * @brief  Bật FOC output
 */
void FOC_Start(FOC_Handle_t *hfoc);

/**
 * @brief  Hiệu chỉnh góc zero (align encoder với rotor)
 *         Gọi khi motor đang ở vị trí cố định với Vd được áp
 * @param  hfoc  Con trỏ FOC_Handle_t
 * @param  current_angle_mech  Góc encoder hiện tại [rad]
 */
void FOC_CalibrateAngle(FOC_Handle_t *hfoc, float current_angle_mech);

/* ===========================================================================
 * API biến đổi tọa độ (cấp thấp, có thể dùng độc lập)
 * =========================================================================== */

/**
 * @brief  Biến đổi Clarke thuận: (a, b, c) → (α, β)
 *         Giả sử ia + ib + ic = 0
 */
FOC_AlphaBeta_t FOC_Clarke(float ia, float ib, float ic);

/**
 * @brief  Biến đổi Park thuận: (α, β) → (d, q)
 * @param  theta_e  Góc điện [rad]
 */
FOC_DQ_t FOC_Park(FOC_AlphaBeta_t ab, float theta_e);

/**
 * @brief  Biến đổi Park ngược: (d, q) → (α, β)
 * @param  theta_e  Góc điện [rad]
 */
FOC_AlphaBeta_t FOC_InvPark(FOC_DQ_t dq, float theta_e);

/**
 * @brief  Biến đổi Clarke ngược (3-phase modulation): (α, β) → (a, b, c)
 *         Output là duty cycle normalize [0.0, 1.0]
 */
void FOC_InvClarke(FOC_AlphaBeta_t ab, float *ua, float *ub, float *uc);

/**
 * @brief  Tính PID một bước
 * @param  pid    Con trỏ FOC_PID_t
 * @param  error  Sai số (setpoint - feedback)
 * @param  Ts     Chu kỳ lấy mẫu [s]
 * @retval Output của PID (đã clamp trong [output_min, output_max])
 */
float FOC_PID_Update(FOC_PID_t *pid, float error, float Ts);

/**
 * @brief  Reset tích phân PID về 0 (dùng khi disable/enable)
 */
void FOC_PID_Reset(FOC_PID_t *pid);

/**
 * @brief  Chạy open-loop velocity: quét góc điện liên tục không cần encoder
 *
 * Đây là chế độ dùng để:
 *   1. Test motor lần đầu (không cần encoder hoạt động)
 *   2. Làm điểm khởi đầu trước khi chuyển sang closed-loop
 *
 * Nguyên lý:
 *   - Tự tăng theta_elec với tốc độ velocity_elec_rad_s mỗi chu kỳ Ts
 *   - Áp vector điện áp (Vd=0, Vq) theo góc đó → tạo từ trường quay
 *   - Motor sẽ đồng bộ theo từ trường (như stepper motor)
 *
 * @param  hfoc              Con trỏ FOC_Handle_t
 * @param  velocity_elec_rad_s  Tốc độ góc ĐIỆN [rad/s]
 *                              = tốc độ cơ học [rad/s] × pole_pairs
 *                              Ví dụ: 10.0f rad/s điện ≈ 1.4 rad/s cơ (7 cặp cực)
 * @param  Vq                Điện áp trục q [V] (tỉ lệ với lực kéo)
 *                           Bắt đầu nhỏ (0.3–0.5V), tăng dần nếu motor trượt bước
 */
void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
