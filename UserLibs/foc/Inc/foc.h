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
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
} FOC_PID_t;

/* ===========================================================================
 * Cấu trúc Low-Pass Filter (LPF) — lọc nhiễu tốc độ encoder
 *
 * Công thức: y[n] = α * y[n-1] + (1-α) * x[n]
 *   α gần 1 = lọc nhiều, chậm theo (dùng khi nhiễu lớn)
 *   α gần 0 = lọc ít, nhanh theo (dùng khi muốn đáp ứng nhanh)
 *   Khuyến nghị: 0.85 – 0.95 cho bước lấy mẫu 10ms
 * =========================================================================== */
typedef struct {
    float alpha;   /*!< Hệ số lọc: 0 < α < 1 */
    float output;  /*!< Giá trị đã lọc (khởi tạo = 0) */
} FOC_LPF_t;

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
    TIM_HandleTypeDef *htim;
    uint32_t ch_a;
    uint32_t ch_b;
    uint32_t ch_c;
    float    pwm_period;

    /* --- Thông số động cơ --- */
    uint8_t  pole_pairs;
    float    voltage_supply;
    float    voltage_limit;

    /* --- Góc điện --- */
    float angle_mech;
    float angle_elec;
    float angle_offset;

    /* --- Ước lượng tốc độ từ encoder --- */
    float prev_angle_mech;   /*!< Góc cơ lần trước [rad] — dùng tính vi phân */
    float velocity_mech;     /*!< Tốc độ cơ học đã lọc LPF [rad/s] */
    FOC_LPF_t lpf_vel;       /*!< Bộ lọc LPF cho tín hiệu tốc độ */

    /* --- Setpoint điều khiển --- */
    float Vd_ref;
    float Vq_ref;

    /* --- Trạng thái biến đổi tọa độ --- */
    FOC_AlphaBeta_t V_ab;
    FOC_DQ_t        V_dq;

    /* --- PID Controllers --- */
    FOC_PID_t pid_d;    /*!< PID trục d (từ thông, thường Kp=0) */
    FOC_PID_t pid_q;    /*!< PID trục q (moment) */
    FOC_PID_t pid_vel;  /*!< PID vòng tốc độ: vel_error → Vq */

    /* --- Trạng thái vòng lặp --- */
    float Ts;
    uint8_t enabled;
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
 * @brief  Căn chỉnh góc encoder-rotor (Alignment sequence)
 *
 * ĐÂY LÀ BƯỚC BẮT BUỘC trước khi chạy closed-loop FOC.
 * Không có alignment → angle_elec sai → FOC áp lực sai hướng → rung + nóng.
 *
 * Nguyên lý:
 *   - Áp Vd tại theta_elec = 0 (KHÔNG dùng encoder)
 *   - Vd kéo rotor về vị trí D-axis tuyệt đối (phụ thuộc vật lý motor)
 *   - Sau khi rotor ổn định, đọc encoder → đó chính là góc "D-axis"
 *   - Gọi FOC_CalibrateAngle() với góc đó để lưu offset
 *
 * Quy trình sử dụng trong main.c:
 *   1. Gọi FOC_AlignD() lặp lại trong ~500ms (rotor kéo về vị trí)
 *   2. Đọc encoder.angle_rad
 *   3. Gọi FOC_CalibrateAngle(&foc, encoder.angle_rad)
 *   4. Khởi động velocity control bình thường
 *
 * @param  hfoc  Con trỏ FOC_Handle_t
 * @param  Vd    Điện áp căn chỉnh [V] — dùng 0.3×voltage_limit đến voltage_limit
 *               Đủ lớn để kéo rotor nhưng không gây quá nhiệt
 */
void FOC_AlignD(FOC_Handle_t *hfoc, float Vd);

/**
 * @brief  Chạy open-loop velocity (giữ lại để test ban đầu)
 */
void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq);

/* ===========================================================================
 * API Closed-loop Velocity Control (bộ lọc tốc độ + PID)
 * =========================================================================== */

/**
 * @brief  Tính 1 bước LPF
 * @param  lpf    Con trỏ FOC_LPF_t
 * @param  input  Giá trị đầu vào thô (tốc độ chưa lọc [rad/s])
 * @retval Giá trị đã lọc [rad/s]
 */
float FOC_LPF_Update(FOC_LPF_t *lpf, float input);

/**
 * @brief  Cài hệ số LPF cho bộ lọc tốc độ
 * @param  alpha  Hệ số [0.0, 1.0]:
 *                0.9 = lọc mạnh (dùng khi nhiễu lớn, encoder rung)
 *                0.7 = lọc vừa (đáp ứng nhanh hơn)
 */
void FOC_SetLPF_Vel(FOC_Handle_t *hfoc, float alpha);

/**
 * @brief  Cấu hình PID vòng tốc độ (velocity loop)
 * @param  Kp/Ki/Kd    Hệ số PID
 * @param  out_min/max Giới hạn Vq output [V] — thường = ±voltage_limit
 *
 * Điểm khởi đầu điều chỉnh:
 *   Kp = 0.05 ~ 0.3  (tăng nếu đáp ứng chậm, giảm nếu dao động)
 *   Ki = 0.01 ~ 0.1  (tăng để triệt sai số xác lập, giảm nếu rung)
 *   Kd = 0            (thường không cần, thêm nếu overshoot nhiều)
 */
void FOC_SetPID_Vel(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                    float out_min, float out_max);

/**
 * @brief  Chạy 1 chu kỳ Closed-loop Velocity Control
 *
 * Pipeline đầy đủ:
 *
 *   [Encoder angle] ──> dθ/dt ──> LPF ──> velocity_mech
 *                                              │
 *                              (target - velocity_mech)
 *                                              │
 *                                           PID_vel
 *                                              │ Vq
 *                                         InvPark(Vd=0,Vq)
 *                                              │ Vα,Vβ
 *                                         InvClarke
 *                                              │ Ua,Ub,Uc
 *                                            PWM
 *
 * @param  hfoc              Con trỏ FOC_Handle_t (đã FOC_Start())
 * @param  angle_mech_rad    Góc cơ học từ encoder [rad], range [0, 2π)
 * @param  target_vel_rad_s  Tốc độ cơ học mục tiêu [rad/s]
 *                           Dương = chiều thuận, Âm = ngược chiều
 *                           Ví dụ: 2*PI ≈ 1 vòng/giây
 */
void FOC_RunVelocity(FOC_Handle_t *hfoc, float angle_mech_rad,
                     float target_vel_rad_s);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
