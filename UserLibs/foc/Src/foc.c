/**
 ******************************************************************************
 * @file    foc.c
 * @brief   Field Oriented Control (FOC) implementation — Cascade 3-Loop
 *
 * === KIẾN TRÚC CASCADE ===
 *
 *  [Vòng Ngoài — 1kHz, TIM6 ISR]
 *   FOC_RunVelocity():
 *     angle_mech → dθ/dt → LPF → velocity_mech
 *     vel_error  → PID_vel → Iq_ref   (chỉ tính, KHÔNG xuất PWM)
 *
 *  [Vòng Trong — ~8kHz, ADC Injected ISR]
 *   FOC_UpdateCurrentLoop():
 *     Ia, Ib → Clarke → Park → Id_meas, Iq_meas
 *     PID_d(Id_ref=0 − Id_meas) → Vd
 *     PID_q(Iq_ref  − Iq_meas) → Vq
 *     InvPark(Vd,Vq,θe) → Vα,Vβ
 *     InvClarke (SVPWM) → Ua,Ub,Uc → PWM
 *
 * === VOLTAGE-MODE FALLBACK ===
 *  Khi current_loop_enabled = 0, FOC_RunVelocity() xuất PWM trực tiếp
 *  (hành vi tương thích với phiên bản cũ).
 ******************************************************************************
 */

#include "foc.h"
#include <string.h>
#include <stdio.h>

/* ===========================================================================
 * Hàm nội bộ
 * =========================================================================== */

/**
 * @brief  Clamp giá trị float trong khoảng [min, max]
 */
static inline float _clamp(float val, float mn, float mx)
{
    if (val < mn) return mn;
    if (val > mx) return mx;
    return val;
}

/**
 * @brief  Chuẩn hóa góc về phạm vi [0, 2π)
 */
static inline float _normalize_angle(float angle)
{
    float a = fmodf(angle, FOC_TWO_PI);
    return (a < 0.0f) ? (a + FOC_TWO_PI) : a;
}

/**
 * @brief  Áp duty cycle chuẩn hóa [0.0, 1.0] vào 3 kênh PWM
 */
static void _apply_pwm(FOC_Handle_t *hfoc, float ua, float ub, float uc)
{
    /* Clamp vào [0, 1] trước khi nhân với ARR */
    ua = _clamp(ua, 0.0f, 1.0f);
    ub = _clamp(ub, 0.0f, 1.0f);
    uc = _clamp(uc, 0.0f, 1.0f);

    uint16_t ccrA = (uint16_t)(ua * hfoc->pwm_period);
    uint16_t ccrB = (uint16_t)(ub * hfoc->pwm_period);
    uint16_t ccrC = (uint16_t)(uc * hfoc->pwm_period);

    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_a, ccrA);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_b, ccrB);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_c, ccrC);
}

/**
 * @brief  Pipeline chung: (Vd, Vq, θe) → PWM
 *         Dùng chung cho cả Voltage-Mode và Current-Mode để tránh trùng code.
 */
static void _vdq_to_pwm(FOC_Handle_t *hfoc, float Vd, float Vq, float theta_e)
{
    FOC_DQ_t dq_ref = { Vd, Vq };
    hfoc->V_ab = FOC_InvPark(dq_ref, theta_e);

    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    /* Normalize điện áp centered về duty cycle [0, 1] */
    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    _apply_pwm(hfoc, ua, ub, uc);
}

/* ===========================================================================
 * Biến đổi tọa độ (Clarke & Park)
 * =========================================================================== */

/**
 * @brief  Clarke thuận: (ia, ib, ic) → (α, β)
 *
 * Công thức (amplitude invariant, giả sử ia + ib + ic = 0):
 *   Iα =  ia
 *   Iβ = (ia + 2*ib) / sqrt(3)
 */
FOC_AlphaBeta_t FOC_Clarke(float ia, float ib, float ic)
{
    FOC_AlphaBeta_t ab;
    (void)ic; /* ic = -(ia+ib), không cần thiết với công thức 2-biến */
    ab.alpha = ia;
    ab.beta  = (ia + 2.0f * ib) * FOC_ONE_SQRT3;
    return ab;
}

/**
 * @brief  Park thuận: (α, β, θe) → (d, q)
 *
 * [Id]   [ cos(θe)   sin(θe)] [Iα]
 * [Iq] = [-sin(θe)   cos(θe)] [Iβ]
 */
FOC_DQ_t FOC_Park(FOC_AlphaBeta_t ab, float theta_e)
{
    float cos_e = cosf(theta_e);
    float sin_e = sinf(theta_e);
    FOC_DQ_t dq;
    dq.d =  ab.alpha * cos_e + ab.beta * sin_e;
    dq.q = -ab.alpha * sin_e + ab.beta * cos_e;
    return dq;
}

/**
 * @brief  Park ngược: (d, q, θe) → (α, β)
 *
 * [Vα]   [cos(θe)  -sin(θe)] [Vd]
 * [Vβ] = [sin(θe)   cos(θe)] [Vq]
 */
FOC_AlphaBeta_t FOC_InvPark(FOC_DQ_t dq, float theta_e)
{
    float cos_e = cosf(theta_e);
    float sin_e = sinf(theta_e);
    FOC_AlphaBeta_t ab;
    ab.alpha = dq.d * cos_e - dq.q * sin_e;
    ab.beta  = dq.d * sin_e + dq.q * cos_e;
    return ab;
}

/**
 * @brief  Clarke ngược + SVPWM centering: (α, β) → (ua, ub, uc) centered
 *
 * Bước 1 — Tính sóng sin 3-pha chuẩn (SPWM):
 *   Ua =  Vα
 *   Ub = -Vα/2 + Vβ*√3/2
 *   Uc = -Vα/2 - Vβ*√3/2
 *
 * Bước 2 — SVPWM Min-Max Centering (Zero-Sequence Injection):
 *   Thêm thành phần common-mode = -(Vmax + Vmin) / 2
 *   Tăng biên độ tối đa thêm 15.5% so với SPWM thuần.
 *
 * Output: điện áp centered quanh 0, cần normalize và +0.5 trước khi áp PWM.
 */
void FOC_InvClarke(FOC_AlphaBeta_t ab, float *ua, float *ub, float *uc)
{
    /* Bước 1: Clarke ngược chuẩn → sóng 3-pha */
    float a =  ab.alpha;
    float b = -ab.alpha * 0.5f + ab.beta * FOC_SQRT3_2;
    float c = -ab.alpha * 0.5f - ab.beta * FOC_SQRT3_2;

    /* Bước 2: SVPWM — tiêm sóng common-mode (min-max centering) */
    float v_min = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
    float v_max = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
    float v_center = (v_min + v_max) * 0.5f;

    *ua = a - v_center;
    *ub = b - v_center;
    *uc = c - v_center;
}

/* ===========================================================================
 * API công khai — Khởi tạo
 * =========================================================================== */

void FOC_Init(FOC_Handle_t *hfoc,
              TIM_HandleTypeDef *htim,
              uint32_t ch_a, uint32_t ch_b, uint32_t ch_c,
              float pwm_period,
              uint8_t pole_pairs,
              float voltage_lim,
              float Ts)
{
    memset(hfoc, 0, sizeof(FOC_Handle_t));

    hfoc->htim          = htim;
    hfoc->ch_a          = ch_a;
    hfoc->ch_b          = ch_b;
    hfoc->ch_c          = ch_c;
    hfoc->pwm_period    = pwm_period;
    hfoc->pole_pairs    = pole_pairs;
    hfoc->voltage_limit = voltage_lim;
    hfoc->Ts            = Ts;
    hfoc->enabled       = 0;

    /* Mặc định: Voltage-Mode (tương thích ngược) */
    hfoc->current_loop_enabled = 0;
    hfoc->current_limit        = 5.0f;  /* 5A mặc định an toàn */
    hfoc->Ts_current           = Ts;    /* Sẽ được ghi đè bởi FOC_EnableCurrentLoop() */

    hfoc->Vd_ref = 0.0f;
    hfoc->Vq_ref = 0.0f;
    hfoc->Id_ref = 0.0f;
    hfoc->Iq_ref = 0.0f;

    /* Chỉ đặt PWM về 50% khi htim hợp lệ.
     * FOC_Stop() nên được gọi lại sau khi PWM đã được Start trong main(). */
    if (hfoc->htim != NULL) {
        FOC_Stop(hfoc);
    }
}

void FOC_SetPID_D(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    PID_Init(&hfoc->pid_d, Kp, Ki, Kd, out_min, out_max);
}

void FOC_SetPID_Q(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    PID_Init(&hfoc->pid_q, Kp, Ki, Kd, out_min, out_max);
}

void FOC_EnableCurrentLoop(FOC_Handle_t *hfoc, float Ts_current)
{
    hfoc->Ts_current           = Ts_current;
    hfoc->current_loop_enabled = 1;
    PID_Reset(&hfoc->pid_d);
    PID_Reset(&hfoc->pid_q);
}

void FOC_SetCurrentLimit(FOC_Handle_t *hfoc, float limit_A)
{
    hfoc->current_limit = (limit_A > 0.0f) ? limit_A : 0.0f;
}

/* ===========================================================================
 * API điều khiển
 * =========================================================================== */

void FOC_SetAngle(FOC_Handle_t *hfoc, float angle_mech_rad)
{
    hfoc->angle_mech = _normalize_angle(angle_mech_rad);

    /* Tính góc điện và trừ offset hiệu chỉnh */
    float elec = hfoc->angle_mech * (float)hfoc->pole_pairs - hfoc->angle_offset;
    hfoc->angle_elec = _normalize_angle(elec);
}

void FOC_SetVoltage(FOC_Handle_t *hfoc, float Vd, float Vq)
{
    hfoc->Vd_ref = _clamp(Vd, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vq_ref = _clamp(Vq, -hfoc->voltage_limit, hfoc->voltage_limit);
}

/**
 * @brief  Hàm cập nhật FOC 1 chu kỳ ở Voltage-Mode
 *         Chỉ dùng khi current_loop_enabled = 0
 */
void FOC_Update(FOC_Handle_t *hfoc)
{
    if (!hfoc->enabled) return;
    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
}

/**
 * @brief  Current Loop — Trái tim của True FOC
 *         GỌI TRONG NGẮT ADC INJECTED (tần số cao, = tần số PWM)
 */
void FOC_UpdateCurrentLoop(FOC_Handle_t *hfoc, float Ia, float Ib)
{
    if (!hfoc->enabled) return;

    /* Lưu dòng điện thực tế */
    hfoc->Ia = Ia;
    hfoc->Ib = Ib;
    hfoc->Ic = -(Ia + Ib);

    /* === Bước 1: Clarke thuận → tọa độ αβ === */
    FOC_AlphaBeta_t I_ab = FOC_Clarke(hfoc->Ia, hfoc->Ib, hfoc->Ic);

    /* === Bước 2: Park thuận → tọa độ dq (dùng góc điện hiện tại) === */
    FOC_DQ_t I_dq = FOC_Park(I_ab, hfoc->angle_elec);
    hfoc->Id_meas = I_dq.d;
    hfoc->Iq_meas = I_dq.q;

    /* === Bước 3: PID Current Loop ===
     * Trục D: Id_ref = 0 (không kích từ thêm với SPMSM)
     * Trục Q: Iq_ref được đặt bởi Velocity PID
     */
    if (!hfoc->current_loop_enabled) return;

    float err_d = hfoc->Id_ref - hfoc->Id_meas;  /* Id_ref luôn = 0 */
    float err_q = hfoc->Iq_ref - hfoc->Iq_meas;

    hfoc->Vd_ref = PID_Update(&hfoc->pid_d, err_d, hfoc->Ts_current);
    hfoc->Vq_ref = PID_Update(&hfoc->pid_q, err_q, hfoc->Ts_current);

    /* === Bước 4: InvPark + SVPWM → PWM === */
    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
}

void FOC_Stop(FOC_Handle_t *hfoc)
{
    hfoc->enabled = 0;
    PID_Reset(&hfoc->pid_d);
    PID_Reset(&hfoc->pid_q);
    PID_Reset(&hfoc->pid_vel);

    /* Reset tất cả setpoint về 0 để tránh xung điện áp bất ngờ khi Start lại */
    hfoc->Vd_ref = 0.0f;
    hfoc->Vq_ref = 0.0f;
    hfoc->Iq_ref = 0.0f;
    hfoc->Id_ref = 0.0f;

    /* Đặt tất cả PWM về 50% duty = không dòng (floating) */
    if (hfoc->htim == NULL) return;
    uint16_t mid = (uint16_t)(hfoc->pwm_period * 0.5f);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_a, mid);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_b, mid);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_c, mid);
}

void FOC_Start(FOC_Handle_t *hfoc, float current_angle)
{
    PID_Reset(&hfoc->pid_d);
    PID_Reset(&hfoc->pid_q);
    PID_Reset(&hfoc->pid_vel);
    hfoc->lpf_vel.output   = 0.0f;
    hfoc->velocity_mech    = 0.0f;
    hfoc->prev_angle_mech  = current_angle;  /* Fix velocity spike */
    hfoc->Iq_ref           = 0.0f;
    hfoc->Id_ref           = 0.0f;
    hfoc->enabled          = 1;
}

void FOC_AlignD(FOC_Handle_t *hfoc, float Vd)
{
    if (!hfoc->enabled) return;

    /* Áp điện áp vào trục D tại góc điện = 0 để kéo rotor về vị trí chuẩn */
    hfoc->Vd_ref    = _clamp(Vd, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vq_ref    = 0.0f;
    hfoc->angle_elec = 0.0f;

    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
}

void FOC_CalibrateAngle(FOC_Handle_t *hfoc, float current_angle_mech)
{
    /* Khi rotor đang được giữ ở điện áp Vd và Iq=0,
     * góc điện thực tế nên là 0.
     * Offset = (góc cơ hiện tại * pole_pairs) mod 2π */
    hfoc->angle_offset = _normalize_angle(
        current_angle_mech * (float)hfoc->pole_pairs
    );
}

/* ===========================================================================
 * Open-loop
 * =========================================================================== */

void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq)
{
    if (!hfoc->enabled) return;

    hfoc->angle_elec += velocity_elec_rad_s * hfoc->Ts_current;
    hfoc->angle_elec  = _normalize_angle(hfoc->angle_elec);

    hfoc->Vq_ref = _clamp(Vq, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vd_ref = 0.0f;

    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
}

/* ===========================================================================
 * Closed-loop Velocity Control
 * =========================================================================== */

float FOC_LPF_Update(FOC_LPF_t *lpf, float input)
{
    lpf->output = lpf->alpha * lpf->output + (1.0f - lpf->alpha) * input;
    return lpf->output;
}

void FOC_SetLPF_Vel(FOC_Handle_t *hfoc, float alpha)
{
    hfoc->lpf_vel.alpha  = _clamp(alpha, 0.0f, 0.9999f);
    hfoc->lpf_vel.output = 0.0f;
}

void FOC_SetPID_Vel(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                    float out_min, float out_max)
{
    PID_Init(&hfoc->pid_vel, Kp, Ki, Kd, out_min, out_max);
}

/**
 * @brief  Closed-loop Velocity Control — 1 chu kỳ
 *
 * Hành vi phụ thuộc vào current_loop_enabled:
 *
 * [Voltage-Mode, current_loop_enabled = 0]:
 *   vel_error → PID_vel → Vq → InvPark → SVPWM → PWM
 *   (Tương thích ngược với code cũ)
 *
 * [True FOC, current_loop_enabled = 1]:
 *   vel_error → PID_vel → Iq_ref  (KHÔNG xuất PWM)
 *   PWM được quản lý bởi FOC_UpdateCurrentLoop() trong ngắt ADC
 */
void FOC_RunVelocity(FOC_Handle_t *hfoc, float angle_mech_rad,
                     float target_vel_rad_s)
{
    if (!hfoc->enabled) return;

    /* --- Bước 1: Tính tốc độ cơ học thô (vi phân góc) --- */
    float d_angle = angle_mech_rad - hfoc->prev_angle_mech;

    /* Xử lý wrap-around (encoder nhảy qua 0/2π) */
    if (d_angle >  FOC_PI)  d_angle -= FOC_TWO_PI;
    if (d_angle < -FOC_PI)  d_angle += FOC_TWO_PI;

    float vel_raw = d_angle / hfoc->Ts;  /* [rad/s] cơ học */
    hfoc->prev_angle_mech = angle_mech_rad;

    /* --- Bước 2: Lọc LPF --- */
    hfoc->velocity_mech = FOC_LPF_Update(&hfoc->lpf_vel, vel_raw);

    /* --- Bước 3: Cập nhật góc điện từ encoder --- */
    hfoc->angle_mech = angle_mech_rad;
    float elec = angle_mech_rad * (float)hfoc->pole_pairs - hfoc->angle_offset;
    hfoc->angle_elec = _normalize_angle(elec);

    /* --- Bước 4: PID vòng tốc độ --- */
    float vel_error = target_vel_rad_s - hfoc->velocity_mech;
    float pid_out   = PID_Update(&hfoc->pid_vel, vel_error, hfoc->Ts);

    if (hfoc->current_loop_enabled) {
        /* ============================================================
         * TRUE FOC MODE: PID_vel xuất Iq_ref [A]
         * PWM sẽ được xử lý bởi FOC_UpdateCurrentLoop() trong ISR ADC
         * ============================================================ */
        hfoc->Iq_ref = _clamp(pid_out, -hfoc->current_limit, hfoc->current_limit);
        hfoc->Id_ref = 0.0f;
        /* KHÔNG gọi _vdq_to_pwm() ở đây */
    } else {
        /* ============================================================
         * VOLTAGE-MODE: PID_vel xuất Vq [V] (fallback, tương thích cũ)
         * ============================================================ */
        hfoc->Vq_ref = pid_out;  /* đã clamp trong PID */
        hfoc->Vd_ref = 0.0f;
        _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
    }
}
