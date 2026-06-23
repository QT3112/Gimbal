/**
 ******************************************************************************
 * @file    attitude.h
 * @brief   Attitude Estimator cho Gimbal 2-IMU
 *
 * Thư viện này đóng gói toàn bộ pipeline ước lượng tư thế cho hệ thống
 * gimbal dùng 2 IMU (không cần encoder), bao gồm:
 *
 *  - Attitude_Init():   Khởi tạo 2 bộ lọc Mahony (frame + payload)
 *  - Attitude_Update(): Cập nhật tư thế từ raw IMU data mỗi chu kỳ Ts
 *  - Attitude_GetPayloadAngle(): Góc camera (dùng cho feedback PID)
 *  - Attitude_GetFrameRate():   Tốc độ góc khung (dùng cho feedforward)
 *
 * Kiến trúc 2-IMU Gimbal:
 *
 *   IMU_frame (trên drone)   → ước lượng tư thế khung
 *                              → cung cấp disturbance rate (feedforward)
 *
 *   IMU_payload (trên camera)→ ước lượng tư thế camera tuyệt đối
 *                              → cung cấp feedback angle cho PID
 *
 *   Electrical angle cho FOC:
 *     angle_elec = payload_pitch_rad * pole_pairs
 *     (thay thế hoàn toàn encoder)
 *
 * Cách sử dụng:
 * @code
 *   Attitude_Handle_t att;
 *   Attitude_Init(&att, 2.0f, 0.005f);   // Kp=2.0, Ki=0.005
 *
 *   // Mỗi Ts (ví dụ 10ms):
 *   Attitude_Update(&att,
 *       frame_gyro, frame_accel,          // IMU trên drone
 *       payload_gyro, payload_accel,      // IMU trên camera
 *       0.01f);                           // dt = 10ms
 *
 *   float cam_pitch   = Attitude_GetPayloadPitch(&att);  // feedback
 *   float frame_rate  = Attitude_GetFramePitchRate(&att); // feedforward
 * @endcode
 ******************************************************************************
 */

#ifndef ATTITUDE_H
#define ATTITUDE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "imu_filter.h"
#include <stdint.h>

/* ===========================================================================
 * Cấu trúc vector 3D dùng nội bộ
 * =========================================================================== */
typedef struct {
    float x;
    float y;
    float z;
} Att_Vec3_t;

/* ===========================================================================
 * Attitude_Handle_t — trạng thái đầy đủ của bộ ước lượng tư thế
 * =========================================================================== */
typedef struct {

    /* --- Bộ lọc Mahony cho từng IMU --- */
    MahonyFilter_t mahony_frame;    /**< Bộ lọc cho IMU trên khung drone */
    MahonyFilter_t mahony_payload;  /**< Bộ lọc cho IMU trên camera/payload */

    /* --- Góc Euler đầu ra của frame (Radian) --- */
    float frame_roll;   /**< Roll của khung drone  [rad] */
    float frame_pitch;  /**< Pitch của khung drone [rad] */
    float frame_yaw;    /**< Yaw của khung drone   [rad] */

    /* --- Góc Euler đầu ra của payload/camera (Radian) --- */
    float payload_roll;   /**< Roll của camera  [rad] */
    float payload_pitch;  /**< Pitch của camera [rad] */
    float payload_yaw;    /**< Yaw của camera   [rad] */

    /* --- Tốc độ góc raw từ IMU frame (rad/s) — dùng cho feedforward --- */
    float frame_rate_x;  /**< Angular rate trục X của khung [rad/s] */
    float frame_rate_y;  /**< Angular rate trục Y của khung [rad/s] */
    float frame_rate_z;  /**< Angular rate trục Z của khung [rad/s] */

    /* --- Tốc độ góc raw từ IMU payload (rad/s) — dùng cho inner velocity loop --- */
    float payload_rate_x;  /**< Angular rate trục X của camera [rad/s] */
    float payload_rate_y;  /**< Angular rate trục Y của camera [rad/s] */
    float payload_rate_z;  /**< Angular rate trục Z của camera [rad/s] */

    /* --- Trạng thái khởi tạo --- */
    uint8_t initialized;    /**< 1 nếu đã init thành công */

} Attitude_Handle_t;

/* ===========================================================================
 * API
 * =========================================================================== */

/**
 * @brief  Khởi tạo bộ ước lượng tư thế cho 2 IMU
 *
 * @param  hatt  Con trỏ đến Attitude_Handle_t
 * @param  Kp    Mahony proportional gain (hội tụ góc nhanh/chậm)
 *               Gợi ý: 1.0 – 2.5
 * @param  Ki    Mahony integral gain (bù drift gyro lâu dài)
 *               Gợi ý: 0.001 – 0.01 (0.0 nếu không cần bù drift)
 */
void Attitude_Init(Attitude_Handle_t *hatt, float Kp, float Ki);

/**
 * @brief  Cập nhật tư thế từ raw IMU data
 *
 * Hàm này phải được gọi đúng chu kỳ Ts (thường trong TIM6 ISR hoặc while(1)).
 * Thực hiện:
 *   1. Normalize accel vectors
 *   2. Mahony_Update cho cả 2 IMU
 *   3. Trích xuất góc Euler và lưu tốc độ góc raw
 *
 * @param  hatt      Con trỏ đến Attitude_Handle_t
 * @param  f_gx/y/z  Gyro của IMU khung drone   [rad/s]
 * @param  f_ax/y/z  Accel của IMU khung drone  [g hoặc m/s²]
 * @param  p_gx/y/z  Gyro của IMU camera        [rad/s]
 * @param  p_ax/y/z  Accel của IMU camera       [g hoặc m/s²]
 * @param  dt        Thời gian lấy mẫu          [s]
 */
void Attitude_Update(Attitude_Handle_t *hatt,
                     float f_gx, float f_gy, float f_gz,
                     float f_ax, float f_ay, float f_az,
                     float p_gx, float p_gy, float p_gz,
                     float p_ax, float p_ay, float p_az,
                     float dt);

/* ---------------------------------------------------------------------------
 * Getter: Góc của payload/camera (dùng làm feedback cho outer PID angle)
 * --------------------------------------------------------------------------- */

/** @return Góc Pitch của camera [rad] */
float Attitude_GetPayloadPitch(const Attitude_Handle_t *hatt);

/** @return Góc Roll của camera [rad] */
float Attitude_GetPayloadRoll(const Attitude_Handle_t *hatt);

/** @return Góc Yaw của camera [rad] */
float Attitude_GetPayloadYaw(const Attitude_Handle_t *hatt);

/* ---------------------------------------------------------------------------
 * Getter: Tốc độ góc của frame (dùng làm feedforward chống nhiễu)
 * --------------------------------------------------------------------------- */

/** @return Pitch rate của khung drone [rad/s] — axis phụ thuộc vào cách đặt IMU */
float Attitude_GetFramePitchRate(const Attitude_Handle_t *hatt);

/** @return Roll rate của khung drone [rad/s] */
float Attitude_GetFrameRollRate(const Attitude_Handle_t *hatt);

/** @return Yaw rate của khung drone [rad/s] */
float Attitude_GetFrameYawRate(const Attitude_Handle_t *hatt);

/* ---------------------------------------------------------------------------
 * Getter: Tốc độ góc của payload (dùng cho inner velocity PID)
 * --------------------------------------------------------------------------- */

/** @return Pitch rate của camera [rad/s] */
float Attitude_GetPayloadPitchRate(const Attitude_Handle_t *hatt);

/** @return Roll rate của camera [rad/s] */
float Attitude_GetPayloadRollRate(const Attitude_Handle_t *hatt);

/* ---------------------------------------------------------------------------
 * Utility
 * --------------------------------------------------------------------------- */

/**
 * @brief  Tính góc điện để đưa vào FOC InvPark (thay encoder)
 *
 * Trong gimbal 1 trục pitch: motor shaft angle ≈ payload_pitch_mech
 * → angle_elec = payload_pitch_mech × pole_pairs
 *
 * @param  hatt        Con trỏ đến Attitude_Handle_t
 * @param  pole_pairs  Số cặp cực của motor
 * @param  offset      Offset góc điện tại thời điểm khởi động [rad]
 * @return Góc điện [rad] đã chuẩn hóa về [0, 2π]
 */
float Attitude_GetElecAngle(const Attitude_Handle_t *hatt,
                            uint8_t pole_pairs,
                            float offset);

/**
 * @brief  Tính góc điện cho FOC trục Roll từ góc roll của camera.
 *
 * @param  hatt        Con trỏ đến Attitude_Handle_t
 * @param  pole_pairs  Số cặp cực của motor Roll
 * @param  offset      Offset góc điện tại thời điểm khởi động [rad]
 * @return Góc điện [rad] đã chuẩn hóa về [0, 2π]
 */
float Attitude_GetElecAngleRoll(const Attitude_Handle_t *hatt,
                                uint8_t pole_pairs,
                                float offset);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
