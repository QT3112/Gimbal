// #include "foc_v1.h"
// #include <string.h>
// #include <stdio.h>

// /* ===========================================================================
//  * Hàm nội bộ
//  * =========================================================================== */

// /**
//  * @brief  Clamp giá trị float trong khoảng [min, max]
//  */
// static inline float _clamp(float val, float mn, float mx)
// {
//     if (val < mn) return mn;
//     if (val > mx) return mx;
//     return val;
// }

// /**
//  * @brief  Chuẩn hóa góc về phạm vi [0, 2π)
//  */
// static inline float _normalize_angle(float angle)
// {
//     float a = fmodf(angle, FOC_TWO_PI);
//     return (a < 0.0f) ? (a + FOC_TWO_PI) : a;
// }

// /**
//  * @brief  Áp duty cycle chuẩn hóa [0.0, 1.0] vào 3 kênh PWM
//  */
// static void _apply_pwm(FOC_Handle_t *hfoc, float ua, float ub, float uc)
// {
//     /* Clamp vào [0, 1] trước khi nhân với ARR */
//     ua = _clamp(ua, 0.0f, 1.0f);
//     ub = _clamp(ub, 0.0f, 1.0f);
//     uc = _clamp(uc, 0.0f, 1.0f);

//     uint16_t ccrA = (uint16_t)(ua * hfoc->pwm_period);
//     uint16_t ccrB = (uint16_t)(ub * hfoc->pwm_period);
//     uint16_t ccrC = (uint16_t)(uc * hfoc->pwm_period);

//     __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_a, ccrA);
//     __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_b, ccrB);
//     __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_c, ccrC);
// }

// /* ===========================================================================
//  * Biến đổi tọa độ (Clarke & Park)
//  * =========================================================================== */

// /**
//  * @brief  Clarke thuận: (ia, ib, ic) → (α, β)
//  *
//  * Công thức (giả sử ia + ib + ic = 0):
//  *   Iα =  ia
//  *   Iβ = (ia + 2*ib) / sqrt(3)
//  *
//  * Dạng ma trận chuẩn (amplitude invariant):
//  *   [Iα]   [ 1       0    ] [ia]
//  *   [Iβ] = [ 1/√3   2/√3  ] [ib]
//  */
// FOC_Clarke_t FOC_Clarke(float ia, float ib, float ic)
// {
//     FOC_Clarke_t ab;
//     (void)ic; /* ic = -(ia+ib), không cần thiết với công thức 2-biến */
//     ab.alpha = ia;
//     ab.beta  = (ia + 2.0f * ib) * FOC_ONE_SQRT3;
//     return ab;
// }

// /**
//  * @brief  Park thuận: (α, β, θe) → (d, q)
//  *
//  * [Id]   [ cos(θe)   sin(θe)] [Iα]
//  * [Iq] = [-sin(θe)   cos(θe)] [Iβ]
//  */
// FOC_Park_t FOC_Park(FOC_Clarke_t ab, float theta_e)
// {
//     float cos_e = cosf(theta_e);
//     float sin_e = sinf(theta_e);
//     FOC_Park_t dq;
//     dq.d =  ab.alpha * cos_e + ab.beta * sin_e;
//     dq.q = -ab.alpha * sin_e + ab.beta * cos_e;
//     return dq;
// }


// /**
//  * @brief  Park ngược: (d, q, θe) → (α, β)
//  *
//  * [Vα]   [cos(θe)  -sin(θe)] [Vd]
//  * [Vβ] = [sin(θe)   cos(θe)] [Vq]
//  */
// FOC_Clarke_t FOC_InvPark(FOC_Park_t dq, float theta_e)
// {
//     float cos_e = cosf(theta_e);
//     float sin_e = sinf(theta_e);
//     FOC_Clarke_t ab;
//     ab.alpha = dq.d * cos_e - dq.q * sin_e;
//     ab.beta  = dq.d * sin_e + dq.q * cos_e;
//     return ab;
// }


// /**
//  * @brief  Clarke ngược: (α, β) → (ua, ub, uc) chuẩn hóa [0.0, 1.0]
//  *
//  * Công thức 3-phase từ tọa độ αβ:
//  *   Ua_centered =  Vα
//  *   Ub_centered = -Vα/2 + Vβ*√3/2
//  *   Uc_centered = -Vα/2 - Vβ*√3/2
//  *
//  * Sau đó thêm offset 0.5 để đưa về [0, 1] (centered modulation):
//  *   Ua = 0.5 + Ua_centered / voltage_limit
//  */
// void FOC_InvClarke(FOC_Clarke_t ab, float *ua, float *ub, float *uc)
// {
//     *ua =  ab.alpha;
//     *ub = -ab.alpha * 0.5f + ab.beta * FOC_SQRT3_2;
//     *uc = -ab.alpha * 0.5f - ab.beta * FOC_SQRT3_2;
// }


// /**
//  * @brief  Hàm cập nhật FOC 1 chu kỳ (core loop)
//  *
//  * Thực hiện:
//  *   1. Park Inverse:   (Vd, Vq) → (Vα, Vβ)  dùng góc điện hiện tại
//  *   2. Clarke Inverse: (Vα, Vβ) → (Ua, Ub, Uc) centered [−1, +1]
//  *   3. Normalize về [0, 1] và áp vào PWM
//  */
// void FOC_Update(FOC_Handle_t *hfoc)
// {
//     if (!hfoc->enabled) return;

//     float theta_e = hfoc->angle_elec;

//     /* Bước 1: Park Inverse → tọa độ αβ */
//     FOC_Park_t dq_ref = { hfoc->Vd_ref, hfoc->Vq_ref };
//     hfoc->V_ab = FOC_InvPark(dq_ref, theta_e);

//     /* Bước 2: Clarke Inverse → 3 pha centered (centered around 0) */
//     float ua_c, ub_c, uc_c;
//     FOC_InvClarke(hfoc->V_ab, &ua_c, &ub_c, &uc_c);

//     /* Bước 3: Normalize điện áp về duty cycle [0, 1]
//      * ua_c, ub_c, uc_c nằm trong [-voltage_limit, +voltage_limit]
//      * Chia cho voltage_limit để về [-1, 1], rồi thêm 0.5 để về [0, 1] */
//     float inv_vmax = 1.0f / hfoc->voltage_limit;
//     float ua = 0.5f + ua_c * inv_vmax * 0.5f;
//     float ub = 0.5f + ub_c * inv_vmax * 0.5f;
//     float uc = 0.5f + uc_c * inv_vmax * 0.5f;

//     /* Bước 4: Áp vào PWM */
//     _apply_pwm(hfoc, ua, ub, uc);
// }

// void FOC_Stop(FOC_Handle_t *hfoc)
// {
//     hfoc->enabled = 0;
//     PID_Reset(&hfoc->pid_d);
//     PID_Reset(&hfoc->pid_q);

//     /* Đặt tất cả PWM về 50% duty = không dòng (floating) */
//     uint16_t mid = (uint16_t)(hfoc->pwm_period * 0.5f);
//     __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_a, mid);
//     __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_b, mid);
//     __HAL_TIM_SET_COMPARE(hfoc->htim, hfoc->ch_c, mid);
// }

// void FOC_Start(FOC_Handle_t *hfoc)
// {
//     PID_Reset(&hfoc->pid_d);
//     PID_Reset(&hfoc->pid_q);
//     hfoc->enabled = 1;
// }