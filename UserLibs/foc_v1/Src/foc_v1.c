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


void FOC_SVPWM(FOC_Handle_t *hfoc, float Vd, float Vq, float theta_e)
{
    /* 1. Giới hạn biên độ Vector điện áp Vdq <= voltage_limit */
    float v_mag_sq = Vd * Vd + Vq * Vq;
    float v_lim_sq = hfoc->voltage_limit * hfoc->voltage_limit;
    if (v_mag_sq > v_lim_sq && v_mag_sq > 1e-9f) {
        float scale = sqrtf(v_lim_sq / v_mag_sq);
        Vd *= scale;
        Vq *= scale;
    }

    hfoc->Vd_ref = Vd;
    hfoc->Vq_ref = Vq;
    hfoc->V_dq.d = Vd;
    hfoc->V_dq.q = Vq;

    /* 2. Biến đổi Park ngược (dq -> alpha/beta) */
    FOC_Park_t dq_ref = { Vd, Vq };
    hfoc->V_ab = FOC_InvPark(dq_ref, theta_e);

    /* 3. Biến đổi Clarke ngược (alpha/beta -> ua, ub, uc) */
    float ua_c, ub_c, uc_c;
    FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

    /* 4. Space Vector PWM (Midpoint Centered Injection)
     * V_mid = (V_min + V_max) / 2
     * Căn giữa dạng sóng điện áp trong chu kỳ PWM, tăng hiệu suất bus DC thêm 15.5% */
    float v_min = ua_c;
    if (ub_c < v_min) v_min = ub_c;
    if (uc_c < v_min) v_min = uc_c;

    float v_max = ua_c;
    if (ub_c > v_max) v_max = ub_c;
    if (uc_c > v_max) v_max = uc_c;

    float v_mid = 0.5f * (v_min + v_max);

    ua_c -= v_mid;
    ub_c -= v_mid;
    uc_c -= v_mid;

    /* 5. Quy đổi điện áp pha về Duty Cycle [0.0, 1.0] */
    float inv_vmax = 1.0f / hfoc->voltage_limit;
    float ua = 0.5f + ua_c * inv_vmax;
    float ub = 0.5f + ub_c * inv_vmax;
    float uc = 0.5f + uc_c * inv_vmax;

    _apply_pwm(hfoc, ua, ub, uc);
}

static inline void _vdq_to_pwm(FOC_Handle_t *hfoc, float Vd, float Vq, float theta_e)
{
    FOC_SVPWM(hfoc, Vd, Vq, theta_e);
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
    /* Khi rotor đang được giữ ở điện áp Vd (Vq=0) tại góc điện = 0 (AlignD):
     * Offset = (sensor_direction * góc cơ hiện tại * pole_pairs) mod 2π */
    float elec_aligned = (float)hfoc->sensor_direction * current_angle_mech * (float)hfoc->pole_pairs;
    hfoc->angle_offset = _normalize_angle(elec_aligned);
}


void FOC_SetAngle(FOC_Handle_t *hfoc, float angle_mech_rad)
{
    hfoc->angle_mech = _normalize_angle(angle_mech_rad);

    /* Tính góc điện dựa trên sensor_direction và trừ offset hiệu chỉnh */
    float elec = (float)hfoc->sensor_direction * hfoc->angle_mech * (float)hfoc->pole_pairs - hfoc->angle_offset;
    hfoc->angle_elec = _normalize_angle(elec);
}

void FOC_SetSensorDirection(FOC_Handle_t *hfoc, int8_t direction)
{
    hfoc->sensor_direction = (direction >= 0) ? 1 : -1;
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

    /* Mặc định offset zero-current cho ADC 12-bit (2048) */
    hfoc->adc_offset_a       = 2048;
    hfoc->adc_offset_b       = 2048;
    hfoc->current_scale      = 1.0f;
    hfoc->current_calibrated = 0;

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


void FOC_ConfigureCurrentSense(FOC_Handle_t *hfoc, float shunt_res, float gain_drv, float vref_adc)
{
    if (shunt_res <= 0.0f || gain_drv <= 0.0f) return;
    if (vref_adc <= 0.0f) vref_adc = 3.3f;

    /* Current [A] = (ADC_raw - ADC_offset) * Vref / (4095.0 * gain_drv * shunt_res) */
    hfoc->current_scale = vref_adc / (4095.0f * gain_drv * shunt_res);
}

void FOC_SetCurrentOffset(FOC_Handle_t *hfoc, uint32_t offset_a, uint32_t offset_b)
{
    hfoc->adc_offset_a = offset_a;
    hfoc->adc_offset_b = offset_b;
    hfoc->current_calibrated = 1;
}

void FOC_CalibrateCurrentOffset(FOC_Handle_t *hfoc, uint32_t sum_raw_a, uint32_t sum_raw_b, uint32_t num_samples)
{
    if (num_samples == 0) return;
    hfoc->adc_offset_a = sum_raw_a / num_samples;
    hfoc->adc_offset_b = sum_raw_b / num_samples;
    hfoc->current_calibrated = 1;
}

void FOC_UpdateCurrentLoopADC(FOC_Handle_t *hfoc, uint32_t raw_adc_a, uint32_t raw_adc_b)
{
    if ((!hfoc->enabled) || (!hfoc->current_loop_enabled)) return;

    /* Trừ zero-current offset và quy đổi sang Amperes */
    float ia = ((float)raw_adc_a - (float)hfoc->adc_offset_a) * hfoc->current_scale;
    float ib = ((float)raw_adc_b - (float)hfoc->adc_offset_b) * hfoc->current_scale;

    /* Extrapolation: Nội suy góc điện ở tần số 20kHz giữa 2 chu kỳ lấy mẫu encoder (1kHz) */
    float d_elec = (float)hfoc->sensor_direction * hfoc->velocity_mech * (float)hfoc->pole_pairs * hfoc->Ts_current;
    hfoc->angle_elec = _normalize_angle(hfoc->angle_elec + d_elec);

    FOC_CurrentLoop(hfoc, ia, ib);
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

    float err_d = hfoc->Id_ref - hfoc->I_dq.d;
    float err_q = hfoc->Iq_ref - hfoc->I_dq.q;
    
    /* === Bước 3: Đặt ưu tiên điện áp trục D (từ thông Id -> 0) === */
    hfoc->pid_d.out_min = -hfoc->voltage_limit;
    hfoc->pid_d.out_max =  hfoc->voltage_limit;
    hfoc->Vd_ref = PID_Update(&hfoc->pid_d, err_d, hfoc->Ts_current);

    /* === Bước 4: Tính giới hạn điện áp tối đa còn lại cho trục Q (momen Iq) ===
     * Vq_max = sqrt(V_limit^2 - Vd^2) */
    float v_d_sq = hfoc->Vd_ref * hfoc->Vd_ref;
    float v_lim_sq = hfoc->voltage_limit * hfoc->voltage_limit;
    float v_q_max = 0.0f;
    if (v_lim_sq > v_d_sq) {
        v_q_max = sqrtf(v_lim_sq - v_d_sq);
    }

    /* === Bước 5: Cập nhật giới hạn động cho PID trục Q === */
    hfoc->pid_q.out_min = -v_q_max;
    hfoc->pid_q.out_max =  v_q_max;
    hfoc->Vq_ref = PID_Update(&hfoc->pid_q, err_q, hfoc->Ts_current);

    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
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

    /* --- Bước 3: Đồng bộ góc cơ học & góc điện mới từ encoder --- */
    FOC_SetAngle(hfoc, angle_mech_rad);

    /* --- Bước 4: PID vòng tốc độ --- */
    float vel_error = target_vel_rad_s - hfoc->velocity_mech;
    float pid_out   = PID_Update(&hfoc->pid_vel, vel_error, hfoc->Ts);

    if (hfoc->current_loop_enabled) {
        hfoc->Iq_ref = _clamp(pid_out, -hfoc->current_limit, hfoc->current_limit);
        hfoc->Id_ref = 0.0f;
    } else {
        hfoc->Vq_ref = pid_out;  /* đã clamp trong PID */
        hfoc->Vd_ref = 0.0f;
        _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
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
    FOC_VelocityLoop(hfoc, angle_mech_rad, target_vel);
}




void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq)
{
    if (!hfoc->enabled) return;

    hfoc->angle_elec += velocity_elec_rad_s * hfoc->Ts_current;
    hfoc->angle_elec  = _normalize_angle(hfoc->angle_elec);

    hfoc->Vq_ref = _clamp(Vq, -hfoc->voltage_limit, hfoc->voltage_limit);
    hfoc->Vd_ref = 0.0f;

    _vdq_to_pwm(hfoc, hfoc->Vd_ref, hfoc->Vq_ref, hfoc->angle_elec);
}