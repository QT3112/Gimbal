#ifndef FOC_V1_H
#define FOC_V1_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32g4xx_hal.h"
#include "pid_lib.h"
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


typedef struct {
    float alpha;   /*!< Thành phần trục α */
    float beta;    /*!< Thành phần trục β */
} FOC_Clarke_t;

typedef struct {
    float d;   /*!< Thành phần trục d (flux, cần điều khiển về 0) */
    float q;   /*!< Thành phần trục q (torque, điều khiển moment) */
} FOC_Park_t;

typedef struct {
    float alpha;   /*!< Hệ số lọc: 0 < α < 1 */
    float output;  /*!< Giá trị đã lọc (khởi tạo = 0) */
} FOC_LPF_t;


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
    float    current_limit;

    /* --- Góc điện --- */
    float angle_mech;
    float angle_elec;
    float angle_offset;
    int8_t sensor_direction;
    
    /* --- Ước lượng tốc độ từ encoder --- */
    float prev_angle_mech;   /*!< Góc cơ lần trước [rad] — dùng tính vi phân */
    float velocity_mech_raw; /*!< Tốc độ cơ học thô (chưa qua bộ lọc) [rad/s] */
    float prev_velocity_mech;/*!< Tốc độ cơ học vòng lặp trước [rad/s] */
    float velocity_mech;     /*!< Tốc độ cơ học hiện tại đã lọc LPF [rad/s] */
    FOC_LPF_t lpf_vel;       /*!< Bộ lọc LPF cho tín hiệu tốc độ */

    /* Setpoint dòng điện (đầu ra của Velocity PID) */
    float Id_ref;            /*!< Dòng từ thông mục tiêu — luôn = 0 với SPMSM [A] */
    float Iq_ref;            /*!< Dòng momen mục tiêu từ velocity PID [A] */

    /* Dòng điện đo được (raw từ ADC, đã quy đổi sang [A]) */
    float Ia;                /*!< Dòng pha A đo từ SO1 [A] */
    float Ib;                /*!< Dòng pha B đo từ SO2 [A] */
    float Ic;                /*!< Dòng pha C = -(Ia+Ib) [A] */

    /* --- Cấu hình & Hiệu chỉnh ADC Current Sense --- */
    uint32_t adc_offset_a;   /*!< Zero-current raw offset ADC Pha A (thường ~2048) */
    uint32_t adc_offset_b;   /*!< Zero-current raw offset ADC Pha B (thường ~2048) */
    float    current_scale;  /*!< Hệ số chuyển đổi (ADC_raw - offset) -> Amperes [A/count] */
    uint8_t  current_calibrated; /*!< Flag = 1 khi đã hoàn thành hiệu chỉnh offset */

    /* --- Trạng thái biến đổi tọa độ --- */
    FOC_Clarke_t I_ab;
    FOC_Park_t   I_dq;

    /* --- Setpoint điều khiển --- */
    float Vd_ref;
    float Vq_ref;

    /* --- Setpoint điện áp đầu ra --- */
    FOC_Clarke_t V_ab;
    FOC_Park_t   V_dq;

    /* --- PID Controllers --- */
    PID_Handle_t pid_d;    /*!< PID trục d (từ thông, thường Kp=0) */
    PID_Handle_t pid_q;    /*!< PID trục q (moment) */
    PID_Handle_t pid_pos; 
    PID_Handle_t pid_vel;  

    /* --- Trạng thái vòng lặp --- */
    float Ts;
    float Ts_current;  
    uint8_t enabled;
    uint8_t position_loop_enabled;
    uint8_t velocity_loop_enabled;
    uint8_t current_loop_enabled;
} FOC_Handle_t;




FOC_Clarke_t FOC_Clarke(float ia, float ib, float ic);
FOC_Park_t FOC_Park(FOC_Clarke_t ab, float theta_e);
FOC_Clarke_t FOC_InvPark(FOC_Park_t dq, float theta_e);
void FOC_InvClarke(FOC_Clarke_t ab, float *ua, float *ub, float *uc);
void FOC_SVPWM(FOC_Handle_t *hfoc, float Vd, float Vq, float theta_e);
void FOC_Update(FOC_Handle_t *hfoc);
void FOC_Stop(FOC_Handle_t *hfoc);
void FOC_Start(FOC_Handle_t *hfoc, float current_angle);
float FOC_LPF_Update(FOC_LPF_t *lpf, float input);
void FOC_SetLPF_Vel(FOC_Handle_t *hfoc, float alpha);
void FOC_AlignD(FOC_Handle_t *hfoc, float Vd);
void FOC_CalibrateAngle(FOC_Handle_t *hfoc, float current_angle_mech);
void FOC_SetAngle(FOC_Handle_t *hfoc, float angle_mech_rad);
void FOC_SetSensorDirection(FOC_Handle_t *hfoc, int8_t direction);
void FOC_SetVoltage(FOC_Handle_t *hfoc, float Vd, float Vq);
void FOC_SetPID_D(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_SetPID_Q(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_SetPID_POS(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_SetPID_VEL(FOC_Handle_t *hfoc, float Kp, float Ki, float Kd, float out_min, float out_max);
void FOC_Init(FOC_Handle_t *hfoc,
              TIM_HandleTypeDef *htim,
              uint32_t ch_a, uint32_t ch_b, uint32_t ch_c,
              float pwm_period,
              uint8_t pole_pairs,
              float voltage_supply,
              float voltage_lim, float current_lim,
              float Ts, float Ts_current);

/* --- APIs Cấu hình & Xử lý Dòng ADC --- */
void FOC_ConfigureCurrentSense(FOC_Handle_t *hfoc, float shunt_res, float gain_drv, float vref_adc);
void FOC_SetCurrentOffset(FOC_Handle_t *hfoc, uint32_t offset_a, uint32_t offset_b);
void FOC_CalibrateCurrentOffset(FOC_Handle_t *hfoc, uint32_t sum_raw_a, uint32_t sum_raw_b, uint32_t num_samples);
void FOC_UpdateCurrentLoopADC(FOC_Handle_t *hfoc, uint32_t raw_adc_a, uint32_t raw_adc_b);

void FOC_CurrentLoop(FOC_Handle_t *hfoc, float Ia, float Ib);
void FOC_VelocityLoop(FOC_Handle_t *hfoc, float angle_mech_rad, float target_vel_rad_s);
void FOC_PositionLoop(FOC_Handle_t *hfoc, float angle_mech_rad, float target_angle_rad);
void FOC_RunOpenLoop(FOC_Handle_t *hfoc, float velocity_elec_rad_s, float Vq);

/* --- IMU-based Control Loops (dùng Gyro làm phản hồi vận tốc) --- */
void FOC_VelocityLoop_IMU(FOC_Handle_t *hfoc, float angle_mech_rad,
                          float imu_vel_rad_s, float target_vel_rad_s);
void FOC_PositionLoop_IMU(FOC_Handle_t *hfoc, float angle_mech_rad,
                          float imu_angle_rad, float imu_vel_rad_s,
                          float target_angle_rad);

#ifdef __cplusplus
}
#endif
#endif //FOC_V1_H