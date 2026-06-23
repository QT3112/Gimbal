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
#define ATT_PI       (3.14159265359f)
#define ATT_DEG2RAD  (0.01745329252f)   /* π/180 */

/* Hệ số hiệu chỉnh Yaw cho Payload Mahony mỗi chu kỳ Update.
 * Giá trị nhỏ (0.01 → 0.05) = hiệu chỉnh chậm, mượt, không gây jitter.
 * Tăng lên nếu Yaw drift quá nhanh (hiếm gặp với gimbal cố định). */
#define ATT_YAW_LOCK_GAIN  (0.02f)

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

/**
 * @brief  Tính Quaternion Error:  q_err = conj(q_a) ⊗ q_b
 *
 * Ý nghĩa: "Từ góc nhìn của q_a, q_b đang lệch bao nhiêu?"
 * Áp dụng: q_a = q_frame, q_b = q_payload
 *          → q_err biểu diễn góc cơ học thực tế của khớp motor.
 *
 * Đầu vào:  q_a[4], q_b[4]  theo thứ tự [w, x, y, z]
 * Đầu ra:   q_out[4]        theo thứ tự [w, x, y, z]
 */
static void _quat_mul_conj(const float q_a[4], const float q_b[4], float q_out[4])
{
    /* Liên hợp của q_a: conj(q_a) = [w, -x, -y, -z] */
    float aw =  q_a[0];
    float ax = -q_a[1];
    float ay = -q_a[2];
    float az = -q_a[3];

    float bw = q_b[0];
    float bx = q_b[1];
    float by = q_b[2];
    float bz = q_b[3];

    /* Hamilton product: conj(q_a) ⊗ q_b */
    q_out[0] = aw*bw - ax*bx - ay*by - az*bz;  /* w */
    q_out[1] = aw*bx + ax*bw + ay*bz - az*by;  /* x */
    q_out[2] = aw*by - ax*bz + ay*bw + az*bx;  /* y */
    q_out[3] = aw*bz + ax*by - ay*bx + az*bw;  /* z */
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

    /* Reset Quaternion Error pipeline */
    hatt->q_error[0] = 1.0f;   /* w = 1 → identity quaternion */
    hatt->q_error[1] = 0.0f;
    hatt->q_error[2] = 0.0f;
    hatt->q_error[3] = 0.0f;
    hatt->relative_pitch = 0.0f;
    hatt->relative_roll  = 0.0f;

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

    /* -----------------------------------------------------------------------
     * QUATERNION ERROR PIPELINE
     * q_error = conj(q_frame) ⊗ q_payload
     *
     * Lấy quaternion trực tiếp từ bộ lọc Mahony (q0..q3 = [w, x, y, z]).
     * q_error biểu diễn góc lệch CƠ HỌC của camera so với khung tay cầm.
     * ----------------------------------------------------------------------- */
    {
        float qf[4] = {
            hatt->mahony_frame.q0,
            hatt->mahony_frame.q1,
            hatt->mahony_frame.q2,
            hatt->mahony_frame.q3
        };
        float qp[4] = {
            hatt->mahony_payload.q0,
            hatt->mahony_payload.q1,
            hatt->mahony_payload.q2,
            hatt->mahony_payload.q3
        };

        _quat_mul_conj(qf, qp, hatt->q_error);

        /* -------------------------------------------------------------------
         * Chuyển q_error → Góc Euler tương đối (Pitch, Roll, Yaw)
         *
         * Công thức chuẩn ZYX Euler từ Quaternion [w, x, y, z]:
         *   roll  = atan2(2*(w*x + y*z), 1 - 2*(x²+y²))
         *   pitch = asin (2*(w*y - z*x))            [clamp ±1]
         *   yaw   = atan2(2*(w*z + x*y), 1 - 2*(y²+z²))
         * ------------------------------------------------------------------- */
        float qw = hatt->q_error[0];
        float qx = hatt->q_error[1];
        float qy = hatt->q_error[2];
        float qz = hatt->q_error[3];

        /* Relative Roll — trục X (motor Roll) */
        float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        hatt->relative_roll = atan2f(sinr_cosp, cosr_cosp);

        /* Relative Pitch — trục Y (motor Pitch) */
        float sinp = 2.0f * (qw * qy - qz * qx);
        if (sinp >  1.0f) sinp =  1.0f;
        if (sinp < -1.0f) sinp = -1.0f;
        hatt->relative_pitch = asinf(sinp);

        /* Relative Yaw — trục Z (dùng cho Yaw Lock, không dùng cho PID) */
        float siny_cosp = 2.0f * (qw * qz + qx * qy);
        float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        float relative_yaw = atan2f(siny_cosp, cosy_cosp);  /* [-π, +π] */

        /* -------------------------------------------------------------------
         * BƯỚC 2: YAW LOCK — Đồng bộ hóa Yaw của Payload về Frame
         *
         * Vấn đề: 2 bộ Mahony chạy độc lập, Yaw của cả 2 sẽ trôi dần theo
         *         thời gian với tốc độ khác nhau (vì không có Magnetometer).
         *         Khi Yaw_frame ≠ Yaw_payload, q_error tạo ra lỗi ảo làm
         *         sai lệch relative_pitch và relative_roll.
         *
         * Giải pháp (Soft Yaw Injection):
         *   Nếu Yaw lệch nhau (relative_yaw ≠ 0), ta hiệu chỉnh nhẹ
         *   quaternion của Payload Mahony theo hướng làm giảm lệch đó.
         *
         *   Cụ thể: Tạo một quaternion xoay nhỏ quanh trục Z (trục Yaw)
         *   với góc = -relative_yaw * gain, rồi cộng vào q_payload.
         *
         * Lưu ý: Dùng xấp xỉ tuyến tính (không dùng slerp) vì gain rất nhỏ.
         *         Sau khi cộng phải normalize lại để giữ đơn vị quaternion.
         * ------------------------------------------------------------------- */
        if (relative_yaw > ATT_PI)  relative_yaw -= ATT_TWO_PI;
        if (relative_yaw < -ATT_PI) relative_yaw += ATT_TWO_PI;

        /* Correction: xoay nhỏ quanh Z để kéo Payload Yaw về Frame Yaw */
        float half_angle = -relative_yaw * ATT_YAW_LOCK_GAIN * 0.5f;
        /* Quaternion xoay quanh Z: [cos(a/2), 0, 0, sin(a/2)] */
        float dqw =  1.0f;      /* cos(half_angle) ≈ 1 vì angle rất nhỏ */
        float dqz =  half_angle; /* sin(half_angle) ≈ half_angle */

        /* Áp dụng hiệu chỉnh vào q_payload của bộ Mahony */
        float *pq = &hatt->mahony_payload.q0;   /* q0=w, q1=x, q2=y, q3=z */
        float new_w = pq[0]*dqw - pq[3]*dqz;
        float new_x = pq[1]*dqw + pq[2]*dqz;  /* nhân Hamilton nhỏ gọn */
        float new_y = pq[2]*dqw - pq[1]*dqz;
        float new_z = pq[3]*dqw + pq[0]*dqz;

        /* Chuẩn hóa quaternion sau khi hiệu chỉnh */
        float norm = 1.0f / sqrtf(new_w*new_w + new_x*new_x +
                                  new_y*new_y + new_z*new_z);
        pq[0] = new_w * norm;
        pq[1] = new_x * norm;
        pq[2] = new_y * norm;
        pq[3] = new_z * norm;
    }
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
 * Getters — Góc cơ học tương đối (từ Quaternion Error Pipeline)
 * =========================================================================== */

float Attitude_GetRelativePitch(const Attitude_Handle_t *hatt)
{
    return hatt->relative_pitch;
}

float Attitude_GetRelativeRoll(const Attitude_Handle_t *hatt)
{
    return hatt->relative_roll;
}

/* ===========================================================================
 * FOC Electrical Angle — Phiên bản TƯƠNG ĐỐI (dùng q_error, chính xác hơn)
 * =========================================================================== */

/**
 * Attitude_GetElecAnglePitchRel
 * angle_elec = relative_pitch × pole_pairs - offset
 */
float Attitude_GetElecAnglePitchRel(const Attitude_Handle_t *hatt,
                                    uint8_t pole_pairs,
                                    float offset)
{
    float elec_angle = hatt->relative_pitch * (float)pole_pairs - offset;
    return _normalize_angle(elec_angle);
}

/**
 * Attitude_GetElecAngleRollRel
 * angle_elec = relative_roll × pole_pairs - offset
 */
float Attitude_GetElecAngleRollRel(const Attitude_Handle_t *hatt,
                                   uint8_t pole_pairs,
                                   float offset)
{
    float elec_angle = hatt->relative_roll * (float)pole_pairs - offset;
    return _normalize_angle(elec_angle);
}

/* ===========================================================================
 * FOC Electrical Angle — [Legacy] Phiên bản TUYỆT ĐỐI (payload Euler)
 * Khuyến nghị dùng *Rel() phía trên thay thế.
 * =========================================================================== */
float Attitude_GetElecAngle(const Attitude_Handle_t *hatt,
                            uint8_t pole_pairs,
                            float offset)
{
    float elec_angle = hatt->payload_pitch * (float)pole_pairs - offset;
    return _normalize_angle(elec_angle);
}

float Attitude_GetElecAngleRoll(const Attitude_Handle_t *hatt,
                                uint8_t pole_pairs,
                                float offset)
{
    float elec_angle = hatt->payload_roll * (float)pole_pairs - offset;
    return _normalize_angle(elec_angle);
}
