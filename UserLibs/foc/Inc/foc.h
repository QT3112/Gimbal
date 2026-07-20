/**
 ******************************************************************************
 * @file    foc.h
 * @brief   Thư viện Field Oriented Control (FOC) cho động cơ BLDC/PMSM
 *
 * === KIẾN TRÚC FOC (Cascade 3 vòng) ===
 *
 *  [Encoder] ──► angle_mech ──► angle_elec
 *
 *  [Vòng Ngoài — 1kHz, TIM6 ISR]
 *   pos_error ──► PID_pos ──► target_vel ──► PID_vel ──► Iq_ref
 *                                                          │
 *  [Vòng Trong — ~8kHz, ADC Injected ISR]                 │
 *   [SO1,SO2] ──► Ia, Ib                                  │
 *       │                                                  │
 *       ▼                                                  ▼
 *   Clarke ──► Iα,Iβ ──► Park ──► Id_meas, Iq_meas
 *                                       │
 *                         (Id_ref=0) ─► PID_d ──► Vd
 *                              Iq_ref ─► PID_q ──► Vq
 *                                                   │
 *                              InvPark(Vd,Vq,θe) ──► Vα,Vβ
 *                                                   │
 *                              InvClarke (SVPWM) ──► Ua,Ub,Uc
 *                                                   │
 *                                                 PWM
 *
 * === BIẾN ĐỔI ĐƯỢC SỬ DỤNG ===
 *
 *  Clarke:    (Ia, Ib, Ic) → (Iα, Iβ)   [abc → stationary frame]
 *  Park:      (Iα, Iβ)    → (Id, Iq)    [stationary → rotating frame]
 *  InvPark:   (Vd, Vq)    → (Vα, Vβ)   [rotating → stationary frame]
 *  InvClarke: (Vα, Vβ)   → (Ua,Ub,Uc)  [stationary → abc, SVPWM]
 *
 * === CHÚ THÍCH PHẦN CỨNG (dự án Gimbal) ===
 *  PWM Timer:   TIM1, Channel 1/2/3
 *  PWM Period:  4249 (ARR), Center-Aligned → tần số ngắt ADC = tần số PWM
 *  Encoder:     AS5048A (14-bit, SPI1)
 *  Current Sense: DRV8302 SO1 (ADC1_IN1), SO2 (ADC2_IN2)
 *  Pole pairs:  Cấu hình tại khởi tạo (mặc định 14)
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
#include "pid_lib.h"

/* ===========================================================================
 * Hằng số toán học
 * =========================================================================== */
#define FOC_PI          3.14159265359f
#define FOC_TWO_PI      6.28318530718f
#define FOC_SQRT3       1.73205080757f
#define FOC_SQRT3_2     0.86602540378f  /* sqrt(3)/2 */
#define FOC_ONE_SQRT3   0.57735026919f  /* 1/sqrt(3) */

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
    float d;   /*!< Thành phần trục d (flux, điều khiển về 0) */
    float q;   /*!< Thành phần trục q (torque) */
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
    float Vd_ref;            /*!< Điện áp trục d [V] — do Current PID tính */
    float Vq_ref;            /*!< Điện áp trục q [V] — do Current PID tính */

    /* =======================================================================
     * Current Loop (Closed-Loop Current Control)
     * ======================================================================= */

    /* Dòng điện đo được (raw từ ADC, đã quy đổi sang [A]) */
    float Ia;                /*!< Dòng pha A đo từ SO1 [A] */
    float Ib;                /*!< Dòng pha B đo từ SO2 [A] */
    float Ic;                /*!< Dòng pha C = -(Ia+Ib) [A] */

    /* Dòng sau biến đổi Park thuận (feedback của Current Loop) */
    float Id_meas;           /*!< Thành phần dòng từ thông (d-axis) [A] */
    float Iq_meas;           /*!< Thành phần dòng momen (q-axis) [A] */

    /* Setpoint dòng điện (đầu ra của Velocity PID) */
    float Id_ref;            /*!< Dòng từ thông mục tiêu — luôn = 0 với SPMSM [A] */
    float Iq_ref;            /*!< Dòng momen mục tiêu từ velocity PID [A] */

    /* Giới hạn bảo vệ */
    float current_limit;     /*!< Giới hạn dòng tối đa |Iq_ref| [A] */

    /* Chu kỳ Current Loop (thường = 1/f_PWM) */
    float Ts_current;        /*!< Chu kỳ lấy mẫu của Current Loop [s] */

    /* Flag bật/tắt Current Loop */
    uint8_t current_loop_enabled; /*!< 1 = True FOC, 0 = Voltage-Mode fallback */

    /* --- Trạng thái biến đổi tọa độ --- */
    FOC_AlphaBeta_t V_ab;
    FOC_DQ_t        V_dq;

    /* --- PID Controllers --- */
    PID_Handle_t pid_d;    /*!< PID trục d (điều khiển Id → 0) — chạy trong Current Loop */
    PID_Handle_t pid_q;    /*!< PID trục q (điều khiển Iq → Iq_ref) — chạy trong Current Loop */
    PID_Handle_t pid_vel;  /*!< PID vòng tốc độ: vel_error → Iq_ref — chạy trong Velocity Loop */
    PID_Handle_t pid_pos;  /*!< PID vòng vị trí: pos_error → target_vel — chạy trong Position Loop */

    /* --- Trạng thái vòng lặp --- */
    float Ts;            /*!< Chu kỳ Velocity Loop [s] */
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
 * @param  Ts           Chu kỳ Velocity Loop [s] (ví dụ: 0.001f = 1ms)
 */
void FOC_Init(FOC_Handle_t *hfoc,
              TIM_HandleTypeDef *htim,
              uint32_t ch_a, uint32_t ch_b, uint32_t ch_c,
              float pwm_period,
              uint8_t pole_pairs,
              float voltage_lim,
              float Ts);

/**
 * @brief  Cấu hình PID trục D (điều khiển từ thông, Id_ref = 0)
 *
 * Điểm khởi đầu: Kp = 0.5~2.0, Ki = 50~500, Kd = 0
 * out_min/max = ±voltage_limit
 */
void FOC_SetPID_D(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max);

/**
 * @brief  Cấu hình PID trục Q (điều khiển moment, Iq → Iq_ref)
 *
 * Điểm khởi đầu: Kp = 0.5~2.0, Ki = 50~500, Kd = 0
 * out_min/max = ±voltage_limit
 */
void FOC_SetPID_Q(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max);

/**
 * @brief  Bật chế độ Current Loop (True FOC)
 *
 * Cần gọi sau FOC_Init(). Nếu không gọi hàm này, FOC hoạt động ở
 * chế độ Voltage-Mode (tương thích ngược với code cũ).
 *
 * @param  hfoc       Con trỏ FOC_Handle_t
 * @param  Ts_current Chu kỳ Current Loop = 1/f_PWM [s]
 *                    Ví dụ: f_PWM=8.5kHz (Center-Aligned, ARR=4999) → Ts = 1/8500
 */
void FOC_EnableCurrentLoop(FOC_Handle_t *hfoc, float Ts_current);

/**
 * @brief  Cấu hình giới hạn dòng điện bảo vệ
 *
 * Giới hạn |Iq_ref| để bảo vệ driver và motor.
 * @param  limit_A  Dòng tối đa [A]. Ví dụ: 3.0f cho gimbal nhỏ.
 */
void FOC_SetCurrentLimit(FOC_Handle_t *hfoc, float limit_A);

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
 * @brief  Hàm cập nhật FOC 1 chu kỳ ở Voltage-Mode (không có Current Loop)
 *
 * Chỉ dùng khi current_loop_enabled = 0 (Voltage-Mode fallback).
 * Thực hiện: InvPark(Vd,Vq) → InvClarke (SVPWM) → PWM
 *
 * @param  hfoc  Con trỏ FOC_Handle_t
 */
void FOC_Update(FOC_Handle_t *hfoc);

/**
 * @brief  Hàm chạy Current Loop — GỌI TRONG NGẮT ADC INJECTED
 *
 * Đây là trái tim của True FOC. Hàm này phải được gọi tại tần số
 * cao (bằng tần số PWM, thông qua trigger từ Timer).
 *
 * Pipeline:
 *   Ia, Ib ──► Clarke ──► Park ──► Id_meas, Iq_meas
 *                                       │
 *           PID_d(Id_ref=0 - Id_meas) ──► Vd
 *           PID_q(Iq_ref   - Iq_meas) ──► Vq
 *                                       │
 *                        InvPark(Vd,Vq) ──► Vα,Vβ
 *                        InvClarke(SVPWM) ──► PWM
 *
 * @param  hfoc  Con trỏ FOC_Handle_t (đã FOC_Start() và EnableCurrentLoop())
 * @param  Ia    Dòng pha A đo được từ SO1 [A]
 * @param  Ib    Dòng pha B đo được từ SO2 [A]
 *
 * @note   Góc điện (angle_elec) phải được cập nhật bởi FOC_RunVelocity()
 *         trước khi hàm này chạy. Trong thực tế, do FOC_RunVelocity chạy
 *         ở 1kHz và Current Loop ở ~8kHz, góc điện có thể cũ hơn vài µs
 *         — điều này là chấp nhận được với gimbal tốc độ thấp.
 */
void FOC_UpdateCurrentLoop(FOC_Handle_t *hfoc, float Ia, float Ib);

/**
 * @brief  Tắt output PWM về 50% duty (trạng thái thả nổi an toàn)
 */
void FOC_Stop(FOC_Handle_t *hfoc);

/**
 * @brief  Bật FOC output — PHẢI truyền góc encoder hiện tại để tránh velocity spike
 * @param  hfoc          Con trỏ FOC_Handle_t
 * @param  current_angle Góc cơ học hiện tại từ encoder [rad]
 */
void FOC_Start(FOC_Handle_t *hfoc, float current_angle);

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
 * @brief  Biến đổi Clarke ngược + SVPWM centering: (α, β) → (a, b, c)
 *         Output là điện áp centered quanh 0 (chưa normalize về duty cycle).
 */
void FOC_InvClarke(FOC_AlphaBeta_t ab, float *ua, float *ub, float *uc);

/**
 * @brief  Căn chỉnh góc encoder-rotor (Alignment sequence)
 *
 * ĐÂY LÀ BƯỚC BẮT BUỘC trước khi chạy closed-loop FOC.
 * Không có alignment → angle_elec sai → FOC áp lực sai hướng → rung + nóng.
 *
 * Quy trình sử dụng trong main.c:
 *   1. Gọi FOC_AlignD() lặp lại trong ~500ms (rotor kéo về vị trí)
 *   2. Đọc encoder.angle_rad
 *   3. Gọi FOC_CalibrateAngle(&foc, encoder.angle_rad)
 *   4. Gọi FOC_EnableCurrentLoop() nếu dùng True FOC
 *   5. Khởi động velocity control bình thường
 *
 * @param  hfoc  Con trỏ FOC_Handle_t
 * @param  Vd    Điện áp căn chỉnh [V] — dùng 0.3×voltage_limit đến voltage_limit
 */
void FOC_AlignD(FOC_Handle_t *hfoc, float Vd);

/**
 * @brief  Chạy open-loop velocity (giữ lại để test ban đầu)
 */
void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq);

/* ===========================================================================
 * API Closed-loop Velocity Control
 * =========================================================================== */

/**
 * @brief  Tính 1 bước LPF
 */
float FOC_LPF_Update(FOC_LPF_t *lpf, float input);

/**
 * @brief  Cài hệ số LPF cho bộ lọc tốc độ
 */
void FOC_SetLPF_Vel(FOC_Handle_t *hfoc, float alpha);

/**
 * @brief  Cấu hình PID vòng tốc độ (velocity loop)
 *
 * Khi current_loop_enabled = 1: out_min/max là giới hạn Iq [A]
 * Khi current_loop_enabled = 0: out_min/max là giới hạn Vq [V]
 */
void FOC_SetPID_Vel(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                    float out_min, float out_max);

/**
 * @brief  Chạy 1 chu kỳ Velocity Control
 *
 * Khi current_loop_enabled = 0 (Voltage-Mode):
 *   → Pipeline cũ: vel_error → PID_vel → Vq → InvPark → SVPWM → PWM
 *
 * Khi current_loop_enabled = 1 (True FOC):
 *   → Chỉ tính: vel_error → PID_vel → Iq_ref
 *   → KHÔNG xuất PWM — PWM được điều khiển bởi FOC_UpdateCurrentLoop()
 *
 * @param  hfoc              Con trỏ FOC_Handle_t (đã FOC_Start())
 * @param  angle_mech_rad    Góc cơ học từ encoder [rad], range [0, 2π)
 * @param  target_vel_rad_s  Tốc độ cơ học mục tiêu [rad/s]
 */
void FOC_RunVelocity(FOC_Handle_t *hfoc, float angle_mech_rad, float target_vel_rad_s);

/* ===========================================================================
 * API Closed-loop Position Control
 * =========================================================================== */

/**
 * @brief  Cấu hình PID vòng vị trí (position loop)
 *
 * out_min/max là giới hạn tốc độ [rad/s]
 */
void FOC_SetPID_Pos(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                    float out_min, float out_max);

/**
 * @brief  Chạy 1 chu kỳ Position Control
 *
 * Tính: pos_error → PID_pos → target_vel
 * Sau đó tự động gọi FOC_RunVelocity() để tính target_vel → Iq_ref.
 *
 * @param  hfoc              Con trỏ FOC_Handle_t
 * @param  angle_mech_rad    Góc cơ học hiện tại từ encoder [rad]
 * @param  target_angle_rad  Góc cơ học mục tiêu [rad]
 */
void FOC_RunPosition(FOC_Handle_t *hfoc, float angle_mech_rad, float target_angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* FOC_H */
