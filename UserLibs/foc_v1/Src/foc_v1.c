#include "foc_v1.h"
#include <string.h>
#include <stdio.h>



static inline float _clamp(float val, float mn, float mx)
{
    if (val < mn) return mn;
    if (val > mx) return mx;
    return val;
}



static inline float _normalize_angle(float angle)
{
    float a = fmodf(angle, FOC_TWO_PI);
    return (a < 0.0f) ? (a + FOC_TWO_PI) : a;
}


static void _apply_pwm(FOC_Handle_t *hfoc, float ua, float ub, float uc)
{
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


static void _vdq_to_pwm(FOC_Handle_t *hfoc, float Vd, float Vq, float theta_e)
{
    FOC_Park_t dq_ref = { Vd, Vq };
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


FOC_Clarke_t FOC_Clarke(float ia, float ib, float ic)
{
    FOC_Clarke_t ab;
    (void)ic;
    ab.alpha = ia;
    ab.beta  = (ia + 2.0f * ib) * FOC_ONE_SQRT3;
    return ab;
}


FOC_Park_t FOC_Park(FOC_Clarke_t ab, float theta_e)
{
    float cos_e = cosf(theta_e);
    float sin_e = sinf(theta_e);
    FOC_Park_t dq;
    dq.d =  ab.alpha * cos_e + ab.beta * sin_e;
    dq.q = -ab.alpha * sin_e + ab.beta * cos_e;
    return dq;
}


FOC_Clarke_t FOC_InvPark(FOC_Park_t dq, float theta_e)
{
    float cos_e = cosf(theta_e);
    float sin_e = sinf(theta_e);
    FOC_Clarke_t ab;
    ab.alpha = dq.d * cos_e - dq.q * sin_e;
    ab.beta  = dq.d * sin_e + dq.q * cos_e;
    return ab;
}


void FOC_InvClarke(FOC_Clarke_t ab, float *ua, float *ub, float *uc)
{
    *ua =  ab.alpha;
    *ub = -ab.alpha * 0.5f + ab.beta * FOC_SQRT3_2;
    *uc = -ab.alpha * 0.5f - ab.beta * FOC_SQRT3_2;
}


void FOC_Update(FOC_Handle_t *hfoc)
{
    if (!hfoc->enabled) return;
    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
}


void FOC_Stop(FOC_Handle_t *hfoc)
{
    hfoc->enabled = 0;
    hfoc->position_loop_enabled = 0;
    hfoc->velocity_loop_enabled = 0;
    hfoc->current_loop_enabled  = 0;
    PID_Reset(&hfoc->pid_d);
    PID_Reset(&hfoc->pid_q);
    PID_Reset(&hfoc->pid_pos);
    PID_Reset(&hfoc->pid_vel);

    hfoc->Vd_ref = 0.0f;
    hfoc->Vq_ref = 0.0f;
    hfoc->Iq_ref = 0.0f;
    hfoc->Id_ref = 0.0f;

    uint16_t mid = (uint16_t)(hfoc->pwm_period * 0.5f);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_a, mid);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_b, mid);
    __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_c, mid);
}


void FOC_Start(FOC_Handle_t *hfoc, float current_angle)
{
    PID_Reset(&hfoc->pid_d);
    PID_Reset(&hfoc->pid_q);
    PID_Reset(&hfoc->pid_pos);
    PID_Reset(&hfoc->pid_vel);
    hfoc->enabled = 1;
    hfoc->position_loop_enabled = 1;
    hfoc->velocity_loop_enabled = 1;
    hfoc->current_loop_enabled  = 1;

    hfoc->lpf_vel.output   = 0.0f;
    hfoc->velocity_mech    = 0.0f;
    hfoc->prev_angle_mech  = current_angle;  /* Fix velocity spike */
    hfoc->Iq_ref           = 0.0f;
    hfoc->Id_ref           = 0.0f;
}


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
    hfoc->angle_offset = _normalize_angle(current_angle_mech * (float)hfoc->pole_pairs);
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

void FOC_SetPID_POS(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    PID_Init(&hfoc->pid_pos, Kp, Ki, Kd, out_min, out_max);
}

void FOC_SetPID_VEL(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd,
                  float out_min, float out_max)
{
    PID_Init(&hfoc->pid_vel, Kp, Ki, Kd, out_min, out_max);
}



void FOC_Init(FOC_Handle_t *hfoc,
              TIM_HandleTypeDef *htim,
              uint32_t ch_a, uint32_t ch_b, uint32_t ch_c,
              float pwm_period,
              uint8_t pole_pairs,
              float voltage_lim, float current_lim,
              float Ts, float Ts_current)
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
    hfoc->position_loop_enabled = 0;
    hfoc->velocity_loop_enabled = 0;
    hfoc->current_loop_enabled  = 0;
    hfoc->sensor_direction = 1; /* Mặc định là thuận chiều */

    /* Mặc định: Voltage-Mode (tương thích ngược) */
    hfoc->current_loop_enabled = 0;
    hfoc->current_limit        = (current_lim > 0.0f) ? current_lim : 0.0f;
    hfoc->Ts_current           = Ts_current;    /* Sẽ được ghi đè bởi FOC_EnableCurrentLoop() */

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


void FOC_CurrentLoop(FOC_Handle_t *hfoc, float Ia, float Ib)
{
    if ((!hfoc->enabled) || (!hfoc->current_loop_enabled)) return;

    /* Lưu dòng điện thực tế */
    hfoc->Ia = Ia;
    hfoc->Ib = Ib;
    hfoc->Ic = -(Ia + Ib);

    /* === Bước 1: Clarke thuận → tọa độ αβ === */
    hfoc->I_ab = FOC_Clarke(hfoc->Ia, hfoc->Ib, hfoc->Ic);

    /* === Bước 2: Park thuận → tọa độ dq (dùng góc điện hiện tại) === */
    hfoc->I_dq = FOC_Park(hfoc->I_ab, hfoc->angle_elec);
    
    float err_d = hfoc->Id_ref - hfoc->Idq.d;
    float err_q = hfoc->Iq_ref - hfoc->Idq.q;

    hfoc->Vd_ref = PID_Update(&hfoc->pid_d, err_d, hfoc->Ts_current);
    hfoc->Vq_ref = PID_Update(&hfoc->pid_q, err_q, hfoc->Ts_current);

    hfoc->Vdq.d = hfoc->Vd_ref;
    hfoc->Vdq.q = hfoc->Vq_ref;

    hfoc->V_ab = FOC_InvPark(hfoc->Vdq, hfoc->angle_elec);
    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    _apply_pwm(hfoc, ua, ub, uc);
}



void FOC_VelocityLoop(FOC_Handle_t *hfoc, float angle_mech_rad,
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

    /* --- Bước 3: Cập nhật góc cơ học từ encoder --- */
    hfoc->angle_mech = angle_mech_rad;
    if (hfoc->current_loop_enabled) {
        hfoc->angle_mech = angle_mech_rad;
    } else {
        hfoc->angle_mech = angle_mech_rad;
        float true_elec = angle_mech_rad * hfoc->pole_pairs - hfoc->angle_offset;
        true_elec = fmodf(true_elec, FOC_TWO_PI);
        if (true_elec < 0.0f) true_elec += FOC_TWO_PI;
    
        float phase_err = true_elec - hfoc->angle_elec;
        if (phase_err > FOC_PI) phase_err -= FOC_TWO_PI;
        if (phase_err < -FOC_PI) phase_err += FOC_TWO_PI;
        
        hfoc->angle_elec += phase_err * 0.1f + (hfoc->velocity_mech * hfoc->pole_pairs * (1.0f / 2000.0f));
        
        if (hfoc->angle_elec >= FOC_TWO_PI) hfoc->angle_elec -= FOC_TWO_PI;
        else if (hfoc->angle_elec < 0.0f) hfoc->angle_elec += FOC_TWO_PI;
    }
    /* KHÔNG cập nhật hfoc->angle_elec ở đây để tránh Race Condition với ngắt ADC (20kHz).
     * Góc điện (angle_elec) giờ đây đã được quản lý và nội suy mượt mà bằng thuật toán PLL
     * trong ISR ADC (FOC_UpdateCurrentLoop). */

    /* --- Bước 4: PID vòng tốc độ --- */
    float vel_error = target_vel_rad_s - hfoc->velocity_mech;
    float pid_out   = PID_Update(&hfoc->pid_vel, vel_error, hfoc->Ts);

    if (hfoc->current_loop_enabled) {
        hfoc->Iq_ref = _clamp(pid_out, -hfoc->current_limit, hfoc->current_limit);
        hfoc->Id_ref = 0.0f;
    } else {
        hfoc->Vq_ref = pid_out;  /* đã clamp trong PID */
        hfoc->Vd_ref = 0.0f;
        hfoc->Vdq.d = hfoc->Vd_ref;
        hfoc->Vdq.q = hfoc->Vq_ref;

        hfoc->V_ab = FOC_InvPark(hfoc->Vdq, hfoc->angle_elec);
        float ua_c, ub_c, uc_c;
        FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

        float inv_vmax = 1.0f / hfoc->voltage_limit;
        float ua = 0.5f + ua_c * inv_vmax * 0.5f;
        float ub = 0.5f + ub_c * inv_vmax * 0.5f;
        float uc = 0.5f + uc_c * inv_vmax * 0.5f;

        _apply_pwm(hfoc, ua, ub, uc);
    }
}



void FOC_PositionLoop(FOC_Handle_t *hfoc, float angle_mech_rad, float target_angle_rad)
{
    if (!hfoc->enabled) return;

    /* Tính sai số vị trí với wrap-around (chọn đường ngắn nhất) */
    float pos_error = target_angle_rad - angle_mech_rad;
    
    if (pos_error >  FOC_PI) pos_error -= FOC_TWO_PI;
    if (pos_error < -FOC_PI) pos_error += FOC_TWO_PI;

    /* Tính PID vòng vị trí -> Vận tốc mục tiêu (rad/s) */
    float target_vel = PID_Update(&hfoc->pid_pos, pos_error, hfoc->Ts);

    /* Gọi vòng lặp tốc độ để thực thi vận tốc mục tiêu */
    FOC_RunVelocity(hfoc, angle_mech_rad, target_vel);
}




void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq)
{
    if (!hfoc->enabled) return;

    hfoc->angle_elec += velocity_elec_rad_s * hfoc->Ts_current;
    hfoc->angle_elec  = _normalize_angle(hfoc->angle_elec);

    hfoc->Vq_ref = _clamp(Vq, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vd_ref = 0.0f;

    hfoc->Vdq.d = hfoc->Vd_ref;
    hfoc->Vdq.q = hfoc->Vq_ref;

    hfoc->V_ab = FOC_InvPark(hfoc->Vdq, hfoc->angle_elec);
    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax * 0.5f;
    float ub = 0.5f + ub_c * inv_vmax * 0.5f;
    float uc = 0.5f + uc_c * inv_vmax * 0.5f;

    _apply_pwm(hfoc, ua, ub, uc);
}