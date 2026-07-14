// #ifndef FOC_V1_H
// #define FOC_V1_H

// #ifdef __cplusplus
// extern "C" {
// #endif

// #include "stm32g4xx_hal.h"
// #include "pid_lib.h"
// #include <stdint.h>
// #include <math.h>

// /* ===========================================================================
//  * Hằng số toán học
//  * =========================================================================== */
// #define FOC_PI          3.14159265359f
// #define FOC_TWO_PI      6.28318530718f
// #define FOC_SQRT3       1.73205080757f
// #define FOC_SQRT3_2     0.86602540378f  /* sqrt(3)/2 */
// #define FOC_ONE_SQRT3   0.57735026919f  /* 1/sqrt(3) */


// /* ===========================================================================
//  * Cấu trúc trạng thái tọa độ αβ (stationary frame)
//  * =========================================================================== */
// typedef struct {
//     float alpha;   /*!< Thành phần trục α */
//     float beta;    /*!< Thành phần trục β */
// } FOC_Clarke_t;

// /* ===========================================================================
//  * Cấu trúc trạng thái tọa độ dq (rotating frame)
//  * =========================================================================== */
// typedef struct {
//     float d;   /*!< Thành phần trục d (flux, cần điều khiển về 0) */
//     float q;   /*!< Thành phần trục q (torque, điều khiển moment) */
// } FOC_Park_t;

// /* ===========================================================================
//  * Cấu trúc Low-Pass Filter (LPF) — lọc nhiễu tốc độ encoder
//  *
//  * Công thức: y[n] = α * y[n-1] + (1-α) * x[n]
//  *   α gần 1 = lọc nhiều, chậm theo (dùng khi nhiễu lớn)
//  *   α gần 0 = lọc ít, nhanh theo (dùng khi muốn đáp ứng nhanh)
//  *   Khuyến nghị: 0.85 – 0.95 cho bước lấy mẫu 10ms
//  * =========================================================================== */
// typedef struct {
//     float alpha;   /*!< Hệ số lọc: 0 < α < 1 */
//     float output;  /*!< Giá trị đã lọc (khởi tạo = 0) */
// } FOC_LPF_t;

// /* ===========================================================================
//  * Cấu trúc chính của FOC Handle
//  * =========================================================================== */
// typedef struct {
//     /* --- Phần cứng PWM --- */
//     TIM_HandleTypeDef *htim;
//     uint32_t ch_a;
//     uint32_t ch_b;
//     uint32_t ch_c;
//     float    pwm_period;

//     /* --- Thông số động cơ --- */
//     uint8_t  pole_pairs;
//     float    voltage_supply;
//     float    voltage_limit;

//     /* --- Góc điện --- */
//     float angle_mech;
//     float angle_elec;
//     float angle_offset;

//     /* --- Ước lượng tốc độ từ encoder --- */
//     float prev_angle_mech;   /*!< Góc cơ lần trước [rad] — dùng tính vi phân */
//     float velocity_mech;     /*!< Tốc độ cơ học đã lọc LPF [rad/s] */
//     FOC_LPF_t lpf_vel;       /*!< Bộ lọc LPF cho tín hiệu tốc độ */

//     /* --- Setpoint điều khiển --- */
//     float Vd_ref;
//     float Vq_ref;

//     /* --- Trạng thái biến đổi tọa độ --- */
//     FOC_Clarke_t V_ab;
//     FOC_Park_t   V_dq;

//     /* --- PID Controllers --- */
//     PID_Handle_t pid_d;    /*!< PID trục d (từ thông, thường Kp=0) */
//     PID_Handle_t pid_q;    /*!< PID trục q (moment) */
//     PID_Handle_t pid_vel;  /*!< PID vòng tốc độ: vel_error → Vq */

//     /* --- Trạng thái vòng lặp --- */
//     float Ts;
//     uint8_t enabled;
// } FOC_Handle_t;


// #ifdef __cplusplus
// }
// #endif
// #endif //FOC_V1_H