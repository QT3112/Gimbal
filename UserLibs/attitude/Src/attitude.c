/**
 ******************************************************************************
 * @file    attitude.c
 * @brief   Attitude Estimator cho Gimbal 2-IMU — Triển khai
 ******************************************************************************
 */

#include "attitude.h"
#include <math.h>

/* ===========================================================================
 * Hằng số nội bộ
 * =========================================================================== */
#define ATT_TWO_PI   (6.28318530718f)
#define ATT_DEG2RAD  (0.01745329252f)   /* π/180 */

/* ===========================================================================
 * Hàm nội bộ
 * =========================================================================== */

/**
 * @brief  Chuẩn hóa góc về [0, 2π]
 */
static float _normalize_angle(float angle)
{
    while (angle < 0.0f)        angle += ATT_TWO_PI;
    while (angle >= ATT_TWO_PI) angle -= ATT_TWO_PI;
    return angle;
}

/* ===========================================================================
 * Attitude_Init
 * =========================================================================== */
void Attitude_Init(Attitude_Handle_t *hatt, float Kp, float Ki)
{
    /* Khởi tạo 2 bộ lọc Mahony với cùng thông số */
    Mahony_Init(&hatt->mahony_frame,   Kp, Ki);
    Mahony_Init(&hatt->mahony_payload, Kp, Ki);

    /* Reset trạng thái đầu ra */
    hatt->frame_roll    = 0.0f;
    hatt->frame_pitch   = 0.0f;
    hatt->frame_yaw     = 0.0f;

    hatt->payload_roll  = 0.0f;
    hatt->payload_pitch = 0.0f;
    hatt->payload_yaw   = 0.0f;

    hatt->frame_rate_x   = 0.0f;
    hatt->frame_rate_y   = 0.0f;
    hatt->frame_rate_z   = 0.0f;

    hatt->payload_rate_x = 0.0f;
    hatt->payload_rate_y = 0.0f;
    hatt->payload_rate_z = 0.0f;

    hatt->initialized = 1;
}

/* ===========================================================================
 * Attitude_Update
 * =========================================================================== */
void Attitude_Update(Attitude_Handle_t *hatt,
                     float f_gx, float f_gy, float f_gz,
                     float f_ax, float f_ay, float f_az,
                     float p_gx, float p_gy, float p_gz,
                     float p_ax, float p_ay, float p_az,
                     float dt)
{
    if (!hatt->initialized) return;

    /* --- Cập nhật IMU khung drone --- */
    Mahony_Update(&hatt->mahony_frame,
                  f_gx, f_gy, f_gz,
                  f_ax, f_ay, f_az,
                  dt);

    hatt->frame_roll  = hatt->mahony_frame.roll;
    hatt->frame_pitch = hatt->mahony_frame.pitch;
    hatt->frame_yaw   = hatt->mahony_frame.yaw;

    /* Lưu tốc độ góc raw của frame (dùng cho feedforward) */
    hatt->frame_rate_x = f_gx;
    hatt->frame_rate_y = f_gy;
    hatt->frame_rate_z = f_gz;

    /* --- Cập nhật IMU camera/payload --- */
    Mahony_Update(&hatt->mahony_payload,
                  p_gx, p_gy, p_gz,
                  p_ax, p_ay, p_az,
                  dt);

    hatt->payload_roll  = hatt->mahony_payload.roll;
    hatt->payload_pitch = hatt->mahony_payload.pitch;
    hatt->payload_yaw   = hatt->mahony_payload.yaw;

    /* Lưu tốc độ góc raw của payload (dùng cho inner velocity PID) */
    hatt->payload_rate_x = p_gx;
    hatt->payload_rate_y = p_gy;
    hatt->payload_rate_z = p_gz;
}

/* ===========================================================================
 * Getters — Payload (camera)
 * =========================================================================== */
float Attitude_GetPayloadPitch(const Attitude_Handle_t *hatt)
{
    return hatt->payload_pitch;
}

float Attitude_GetPayloadRoll(const Attitude_Handle_t *hatt)
{
    return hatt->payload_roll;
}

float Attitude_GetPayloadYaw(const Attitude_Handle_t *hatt)
{
    return hatt->payload_yaw;
}

/* ===========================================================================
 * Getters — Frame rate (feedforward)
 * =========================================================================== */
float Attitude_GetFramePitchRate(const Attitude_Handle_t *hatt)
{
    /*
     * Trục nào là "pitch" phụ thuộc vào cách gắn IMU trên drone.
     * Mặc định: Y-axis của IMU_frame là trục pitch.
     * Thay đổi nếu IMU được gắn theo hướng khác.
     */
    return hatt->frame_rate_y;
}

float Attitude_GetFrameRollRate(const Attitude_Handle_t *hatt)
{
    return hatt->frame_rate_x;
}

float Attitude_GetFrameYawRate(const Attitude_Handle_t *hatt)
{
    return hatt->frame_rate_z;
}

/* ===========================================================================
 * Getters — Payload rate (inner velocity PID)
 * =========================================================================== */
float Attitude_GetPayloadPitchRate(const Attitude_Handle_t *hatt)
{
    return hatt->payload_rate_y;
}

float Attitude_GetPayloadRollRate(const Attitude_Handle_t *hatt)
{
    return hatt->payload_rate_x;
}

/* ===========================================================================
 * Attitude_GetElecAngle
 *
 * Tính góc điện cho FOC từ góc pitch của camera.
 *
 * Nguyên lý:
 *   Trong gimbal 1 trục pitch, camera chuyển động ↔ motor shaft quay.
 *   → camera_pitch_mech ≡ motor_shaft_angle_mech
 *   → angle_elec = camera_pitch_mech × pole_pairs - offset
 *
 * Lưu ý: Góc Mahony trả về [-π, +π]. Nhân với pole_pairs trước khi normalize.
 * =========================================================================== */
float Attitude_GetElecAngle(const Attitude_Handle_t *hatt,
                            uint8_t pole_pairs,
                            float offset)
{
    float mech_angle  = hatt->payload_pitch;   /* [−π, +π] */
    float elec_angle  = mech_angle * (float)pole_pairs - offset;
    return _normalize_angle(elec_angle);
}
