/**
 ******************************************************************************
 * @file    foc.c
 * @brief   Field Oriented Control (FOC) implementation
 *
 * === LUỒNG THỰC THI FOC_Update() ===
 *
 *  1. Tính góc điện: θe = (θ_mech - offset) * pole_pairs
 *  2. Tính sin/cos của θe
 *  3. Park Inverse: (Vd, Vq) → (Vα, Vβ) dùng sin/cos
 *  4. Clarke Inverse: (Vα, Vβ) → (Ua, Ub, Uc) normalized [0.0, 1.0]
 *  5. Áp duty cycle vào TIM PWM
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

/* ===========================================================================
 * Biến đổi tọa độ (Clarke & Park)
 * =========================================================================== */

/**
 * @brief  Clarke thuận: (ia, ib, ic) → (α, β)
 *
 * Công thức (giả sử ia + ib + ic = 0):
 *   Iα =  ia
 *   Iβ = (ia + 2*ib) / sqrt(3)
 *
 * Dạng ma trận chuẩn (amplitude invariant):
 *   [Iα]   [ 1       0    ] [ia]
 *   [Iβ] = [ 1/√3   2/√3  ] [ib]
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
 *   Điều này tái căn giữa các sóng điện áp quanh điểm 0, cho phép biên độ
 *   tối đa tăng thêm 15.5% so với SPWM thuần mà không bị méo dạng.
 *   Đây là kỹ thuật tương đương Space Vector PWM trong miền thời gian.
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
 * PID Controller
 * =========================================================================== */

/**
 * @brief  Tính một bước PID với anti-windup (clamping)
 *
 * output = Kp*e + Ki*∫e*dt + Kd*de/dt
 *
 * Anti-windup: tích phân bị clamp trong [output_min, output_max].
 * Lưu ý về Derivative Kick: công thức vi phân trên ERROR (d(error)/dt)
 * sẽ tạo spike khi setpoint thay đổi đột ngột. Để triệt tiêu hoàn toàn
 * Derivative Kick, cần vi phân trên MEASUREMENT: -Kd * d(measurement)/dt.
 * Với Gimbal, setpoint thay đổi mượt nên cách hiện tại là chấp nhận được.
 */
float FOC_PID_Update(FOC_PID_t *pid, float error, float Ts)
{
    /* Thành phần tỉ lệ */
    float p_term = pid->Kp * error;

    /* Thành phần tích phân với anti-windup clamp */
    pid->integral += pid->Ki * error * Ts;
    pid->integral  = _clamp(pid->integral, pid->output_min, pid->output_max);

    /* Thành phần vi phân (d(error)/dt) */
    float d_term = (Ts > 1e-9f) ? (pid->Kd * (error - pid->prev_error) / Ts) : 0.0f;
    pid->prev_error = error;

    /* Tổng và clamp */
    float output = p_term + pid->integral + d_term;
    return _clamp(output, pid->output_min, pid->output_max);
}

void FOC_PID_Reset(FOC_PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

/* ===========================================================================
 * API công khai
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

    /* Mặc định: Id_ref = 0 (không từ hóa thêm), Iq_ref = 0 (dừng) */
    hfoc->Vd_ref = 0.0f;
    hfoc->Vq_ref = 0.0f;

    /* Đặt PWM về 50% (trạng thái an toàn = không dòng) */
    FOC_Stop(hfoc);
}

void FOC_SetPID_D(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    hfoc->pid_d.Kp         = Kp;
    hfoc->pid_d.Ki         = Ki;
    hfoc->pid_d.Kd         = Kd;
    hfoc->pid_d.output_min = out_min;
    hfoc->pid_d.output_max = out_max;
    FOC_PID_Reset(&hfoc->pid_d);
}

void FOC_SetPID_Q(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    hfoc->pid_q.Kp         = Kp;
    hfoc->pid_q.Ki         = Ki;
    hfoc->pid_q.Kd         = Kd;
    hfoc->pid_q.output_min = out_min;
    hfoc->pid_q.output_max = out_max;
    FOC_PID_Reset(&hfoc->pid_q);
}

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
 * @brief  Hàm cập nhật FOC 1 chu kỳ (core loop)
 *
 * Thực hiện:
 *   1. Park Inverse:   (Vd, Vq) → (Vα, Vβ)  dùng góc điện hiện tại
 *   2. Clarke Inverse: (Vα, Vβ) → (Ua, Ub, Uc) centered [−1, +1]
 *   3. Normalize về [0, 1] và áp vào PWM
 */
void FOC_Update(FOC_Handle_t *hfoc)
{
    if (!hfoc->enabled) return;

    float theta_e = hfoc->angle_elec;

    /* Bước 1: Park Inverse → tọa độ αβ */
    FOC_DQ_t dq_ref = { hfoc->Vd_ref, hfoc->Vq_ref };
    hfoc->V_ab = FOC_InvPark(dq_ref, theta_e);

    /* Bước 2: Clarke Inverse + SVPWM → 3 pha centered quanh 0 */
    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    /* Bước 3: Normalize về duty cycle [0, 1]
     * SVPWM tận dụng được đến 1/sqrt(3) ≈ 0.577 × Vdc (so với SPWM là 0.5 × Vdc)
     * Dùng voltage_limit làm mốc scale để giữ tương thích với cài đặt hiện tại */
    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    /* Bước 4: Áp vào PWM */
    _apply_pwm(hfoc, ua, ub, uc);
}

void FOC_Stop(FOC_Handle_t *hfoc)
{
    hfoc->enabled = 0;
    FOC_PID_Reset(&hfoc->pid_d);
    FOC_PID_Reset(&hfoc->pid_q);

    /* Đặt tất cả PWM về 50% duty = không dòng (floating) */
    uint16_t mid = (uint16_t)(hfoc->pwm_period * 0.5f);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_a, mid);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_b, mid);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_c, mid);
}

/**
 * @brief  Bật FOC output — PHẢI truyền góc encoder hiện tại
 *
 * Lý do cần current_angle: khi bắt đầu, prev_angle_mech = 0.
 * Nếu encoder thực tế đang ở 3.14 rad, chu kỳ đầu tiên của
 * FOC_RunVelocity sẽ tính d_angle = 3.14 / Ts → velocity spike 314 rad/s.
 * Bằng cách khởi tạo prev_angle_mech với góc hiện tại, d_angle ≈ 0 và
 * PID tốc độ khởi động êm ái.
 *
 * @param  hfoc          Con trỏ FOC_Handle_t
 * @param  current_angle Góc cơ học hiện tại từ encoder [rad]
 */
void FOC_Start(FOC_Handle_t *hfoc, float current_angle)
{
    FOC_PID_Reset(&hfoc->pid_d);
    FOC_PID_Reset(&hfoc->pid_q);
    FOC_PID_Reset(&hfoc->pid_vel);
    hfoc->lpf_vel.output   = 0.0f;
    hfoc->velocity_mech    = 0.0f;
    hfoc->prev_angle_mech  = current_angle;  /* <-- Fix velocity spike */
    hfoc->enabled = 1;
}

void FOC_AlignD(FOC_Handle_t *hfoc, float Vd)
{
    if (!hfoc->enabled) return;

    /* Áp điện áp vào trục D (từ thông) và Vq = 0 (không tạo torque quay ngoài)
     * Góc điện = 0 (D-axis absolute) */
    hfoc->Vd_ref = _clamp(Vd, 0.0f, hfoc->voltage_limit);
    hfoc->Vq_ref = 0.0f;
    hfoc->angle_elec = 0.0f;

    FOC_DQ_t dq_ref = { hfoc->Vd_ref, hfoc->Vq_ref };
    hfoc->V_ab = FOC_InvPark(dq_ref, hfoc->angle_elec);

    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    _apply_pwm(hfoc, ua, ub, uc);
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

/**
 * @brief  Open-loop velocity: quét góc điện liên tục để motor quay
 *
 * Nguyên lý: mỗi lần gọi, tự cộng thêm (velocity * Ts) vào angle_elec,
 * sau đó áp vector điện áp (Vd=0, Vq) theo góc đó.
 * Motor sẽ đồng bộ theo từ trường quay, giống như stepper motor.
 *
 * Lưu ý: Hàm này KHÔNG dùng encoder. angle_elec được quản lý nội bộ.
 */
void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq)
{
    if (!hfoc->enabled) return;

    hfoc->angle_elec += velocity_elec_rad_s * hfoc->Ts;
    hfoc->angle_elec  = _normalize_angle(hfoc->angle_elec);

    hfoc->Vq_ref = _clamp(Vq, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vd_ref = 0.0f;

    FOC_DQ_t dq_ref = { hfoc->Vd_ref, hfoc->Vq_ref };
    hfoc->V_ab = FOC_InvPark(dq_ref, hfoc->angle_elec);

    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    _apply_pwm(hfoc, ua, ub, uc);
}

/* ===========================================================================
 * Closed-loop Velocity Control
 * =========================================================================== */

/**
 * @brief  Tính 1 bước LPF bậc 1
 *         y[n] = α * y[n-1] + (1-α) * x[n]
 */
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
    hfoc->pid_vel.Kp         = Kp;
    hfoc->pid_vel.Ki         = Ki;
    hfoc->pid_vel.Kd         = Kd;
    hfoc->pid_vel.output_min = out_min;
    hfoc->pid_vel.output_max = out_max;
    FOC_PID_Reset(&hfoc->pid_vel);
}

/**
 * @brief  Closed-loop velocity control — 1 chu kỳ điều khiển
 *
 * Bước 1: Ước lượng tốc độ cơ học từ encoder (vi phân góc)
 *   vel_raw = (angle_now - angle_prev) / Ts
 *   Xử lý wrap-around: nếu ∆θ > π thì ∆θ -= 2π (và ngược lại)
 *
 * Bước 2: Lọc nhiễu qua LPF
 *   vel_filtered = α * vel_filtered_prev + (1-α) * vel_raw
 *
 * Bước 3: PID vòng tốc độ → Vq
 *   error = target_vel - vel_filtered
 *   Vq = Kp*error + Ki*∫error*dt + Kd*d(error)/dt
 *
 * Bước 4: FOC transforms → PWM (dùng góc encoder thực)
 *   angle_elec = angle_mech * pole_pairs - offset
 *   InvPark(Vd=0, Vq) → Vα,Vβ → InvClarke → Ua,Ub,Uc → PWM
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

    /* --- Bước 3: PID vòng tốc độ → Vq --- */
    float vel_error = target_vel_rad_s - hfoc->velocity_mech;
    float Vq = FOC_PID_Update(&hfoc->pid_vel, vel_error, hfoc->Ts);

    hfoc->Vq_ref = Vq;  /* đã clamp trong PID */
    hfoc->Vd_ref = 0.0f;

    /* --- Bước 4: Cập nhật góc điện từ encoder --- */
    float elec = angle_mech_rad * (float)hfoc->pole_pairs - hfoc->angle_offset;
    hfoc->angle_mech = angle_mech_rad;
    hfoc->angle_elec = _normalize_angle(elec);

    /* --- Bước 5: FOC transforms → PWM (dùng SVPWM qua FOC_InvClarke) --- */
    FOC_DQ_t dq_ref = { hfoc->Vd_ref, hfoc->Vq_ref };
    hfoc->V_ab = FOC_InvPark(dq_ref, hfoc->angle_elec);

    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    _apply_pwm(hfoc, ua, ub, uc);
}
