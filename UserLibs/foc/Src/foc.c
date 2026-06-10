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
 * @brief  Clarke ngược: (α, β) → (ua, ub, uc) chuẩn hóa [0.0, 1.0]
 *
 * Công thức 3-phase từ tọa độ αβ:
 *   Ua_centered =  Vα
 *   Ub_centered = -Vα/2 + Vβ*√3/2
 *   Uc_centered = -Vα/2 - Vβ*√3/2
 *
 * Sau đó thêm offset 0.5 để đưa về [0, 1] (centered modulation):
 *   Ua = 0.5 + Ua_centered / voltage_limit
 */
void FOC_InvClarke(FOC_AlphaBeta_t ab, float *ua, float *ub, float *uc)
{
    *ua =  ab.alpha;
    *ub = -ab.alpha * 0.5f + ab.beta * FOC_SQRT3_2;
    *uc = -ab.alpha * 0.5f - ab.beta * FOC_SQRT3_2;
}

/* ===========================================================================
 * PID Controller
 * =========================================================================== */

/**
 * @brief  Tính một bước PID với anti-windup (clamping)
 *
 * output = Kp*e + Ki*∫e*dt + Kd*de/dt
 * Anti-windup: không tích phân nếu output đã bị bão hòa
 */
float FOC_PID_Update(FOC_PID_t *pid, float error, float Ts)
{
    /* Thành phần tỉ lệ */
    float p_term = pid->Kp * error;

    /* Thành phần tích phân (chỉ update khi chưa bão hòa - anti-windup) */
    pid->integral += pid->Ki * error * Ts;
    pid->integral  = _clamp(pid->integral, pid->output_min, pid->output_max);

    /* Thành phần vi phân (trên sai số, không phải output để tránh derivative kick) */
    float d_term = pid->Kd * (error - pid->prev_error) / Ts;
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

    /* Bước 2: Clarke Inverse → 3 pha centered (centered around 0) */
    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    /* Bước 3: Normalize điện áp về duty cycle [0, 1]
     * ua_c, ub_c, uc_c nằm trong [-voltage_limit, +voltage_limit]
     * Chia cho voltage_limit để về [-1, 1], rồi thêm 0.5 để về [0, 1] */
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

void FOC_Start(FOC_Handle_t *hfoc)
{
    FOC_PID_Reset(&hfoc->pid_d);
    FOC_PID_Reset(&hfoc->pid_q);
    hfoc->enabled = 1;
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

    /* Tự tăng góc điện theo vận tốc yêu cầu */
    hfoc->angle_elec += velocity_elec_rad_s * hfoc->Ts;
    hfoc->angle_elec  = _normalize_angle(hfoc->angle_elec);

    /* Cập nhật Vq (clamp trong giới hạn an toàn) */
    hfoc->Vq_ref = _clamp(Vq, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vd_ref = 0.0f;

    /* Áp vector điện áp theo góc điện hiện tại */
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
