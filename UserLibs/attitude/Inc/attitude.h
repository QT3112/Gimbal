/**
 ******************************************************************************
 * @file    attitude.h
 * @brief   Attitude Estimator cho Gimbal 2-IMU
 *
 * Thư viện này đóng gói toàn bộ pipeline ước lượng tư thế cho hệ thống
 * gimbal dùng 2 IMU (không cần encoder), bao gồm:
 *
 *  - Attitude_Init():              Khởi tạo 2 bộ lọc Mahony (frame + payload)
 *  - Attitude_Update():            Cập nhật tư thế từ raw IMU data mỗi chu kỳ Ts
 *  - Attitude_GetPayloadAngle():   Góc tuyệt đối camera (dùng cho feedback PID)
 *  - Attitude_GetFrameRate():      Tốc độ góc khung (dùng cho feedforward)
 *  - Attitude_GetRelativePitch():  Góc cơ học motor Pitch [= q_err → Euler]
 *  - Attitude_GetRelativeRoll():   Góc cơ học motor Roll  [= q_err → Euler]
 *
 * Kiến trúc 2-IMU Gimbal:
 *
 *   IMU_frame (trên drone)   → ước lượng tư thế khung  → q_frame
 *                              → cung cấp disturbance rate (feedforward)
 *
 *   IMU_payload (trên camera)→ ước lượng tư thế camera → q_payload
 *                              → cung cấp feedback angle cho PID
 *
 *   Quaternion Error Pipeline:
 *     q_error     = q_frame* ⊗ q_payload    (Conjugate của frame × payload)
 *     rel_pitch   = q_error → Euler Pitch   (Góc cơ học motor Pitch)
 *     rel_roll    = q_error → Euler Roll    (Góc cơ học motor Roll)
 *
 *   Electrical angle cho FOC (thay thế hoàn toàn encoder vật lý):
 *     angle_elec_pitch = rel_pitch * pole_pairs
 *     angle_elec_roll  = rel_roll  * pole_pairs
 *
 * Cách sử dụng:
 * @code
 *   Attitude_Handle_t att;
 *   Attitude_Init(&att, 2.0f, 0.005f);   // Kp=2.0, Ki=0.005
 *
 *   // Mỗi Ts (ví dụ 1ms trong TIM ISR):
 *   Attitude_Update(&att,
 *       f_gx, f_gy, f_gz, f_ax, f_ay, f_az,   // IMU trên drone
 *       p_gx, p_gy, p_gz, p_ax, p_ay, p_az,   // IMU trên camera
 *       0.001f);                                // dt = 1ms
 *
 *   // Feedback cho PID Angle (góc tuyệt đối camera so với mặt đất)
 *   float cam_pitch  = Attitude_GetPayloadPitch(&att);
 *
 *   // Góc cơ học motor (thay encoder) — dùng để tính electrical angle FOC
 *   float rel_pitch  = Attitude_GetRelativePitch(&att);
 *   float rel_roll   = Attitude_GetRelativeRoll(&att);
 *
 *   // Feedforward từ IMU khung
 *   float frame_rate = Attitude_GetFramePitchRate(&att);
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

    /* =========================================================================
     * QUATERNION ERROR PIPELINE
     * =========================================================================
     * q_error = conjugate(q_frame) ⊗ q_payload
     *
     * Biểu diễn "từ góc nhìn của tay cầm, camera đang lệch bao nhiêu".
     * Đây là góc cơ học thực tế của khớp motor, thay thế hoàn toàn encoder.
     * ========================================================================= */

    float q_error[4];       /**< Sai số Quaternion [w, x, y, z] giữa frame và payload */

    float relative_pitch;   /**< Góc cơ học motor Pitch [rad] — tách từ q_error */
    float relative_roll;    /**< Góc cơ học motor Roll  [rad] — tách từ q_error */

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
 * Getter: Góc cơ học tương đối của motor (Quaternion Error → Euler)
 *
 * Đây là kết quả của pipeline:  q_error = q_frame* ⊗ q_payload
 * Các giá trị này là góc thực tế của khớp cơ khí, thay thế encoder vật lý.
 * Dùng để:
 *   1. Tính góc điện cho FOC (nhân với pole_pairs)
 *   2. Dùng làm feedback cho outer angle PID
 * --------------------------------------------------------------------------- */

/** @return Góc cơ học motor Pitch [rad] — tách từ q_error */
float Attitude_GetRelativePitch(const Attitude_Handle_t *hatt);

/** @return Góc cơ học motor Roll [rad] — tách từ q_error */
float Attitude_GetRelativeRoll(const Attitude_Handle_t *hatt);

/**
 * @brief  Trả về sai số Pitch theo Quaternion Error (dùng cho Outer PID).
 *
 * Thay vì dùng relative_pitch (ZYX Euler) vốn bị coupling với Roll,
 * dùng trực tiếp thành phần ảo q_error[2] (qy):
 *
 *   pitch_err ≈ 2 × q_error[2]
 *
 * Ở các góc nhỏ (´45° trong vùng hoạt động của gimbal), thành phần này
 * chủ yếu biểu diễn sai số Pitch và độc lập với Roll.
 *
 * @return Sai số Pitch tưᨏng đối [rad] — dùng cho outer Angle PID
 */
float Attitude_GetQErrorPitch(const Attitude_Handle_t *hatt);

/**
 * @brief  Trả về sai số Roll theo Quaternion Error (dùng cho Outer PID).
 *
 *   roll_err ≈ 2 × q_error[1]
 *
 * @return Sai số Roll tưᨏng đối [rad] — dùng cho outer Angle PID
 */
float Attitude_GetQErrorRoll(const Attitude_Handle_t *hatt);

/* ---------------------------------------------------------------------------
 * Utility: Tính góc điện cho FOC
 * --------------------------------------------------------------------------- */

/**
 * @brief  Tính góc điện cho FOC motor Pitch dựa trên góc TƯƠNG ĐỐI (q_error).
 *
 * Đây là phiên bản chính xác hơn Attitude_GetElecAngle() vì nó dùng góc
 * cơ học thực của khớp motor thay vì góc tuyệt đối của camera.
 *
 *   angle_elec = relative_pitch × pole_pairs - offset
 *
 * @param  hatt        Con trỏ đến Attitude_Handle_t
 * @param  pole_pairs  Số cặp cực của motor (Ví dụ: 7 cho motor 14P)
 * @param  offset      Offset góc điện lúc Homing [rad]
 * @return Góc điện [rad] đã chuẩn hóa về [0, 2π]
 */
float Attitude_GetElecAnglePitchRel(const Attitude_Handle_t *hatt,
                                    uint8_t pole_pairs,
                                    float offset);

/**
 * @brief  Tính góc điện cho FOC motor Roll dựa trên góc TƯƠNG ĐỐI (q_error).
 *
 *   angle_elec = relative_roll × pole_pairs - offset
 *
 * @param  hatt        Con trỏ đến Attitude_Handle_t
 * @param  pole_pairs  Số cặp cực của motor Roll (Ví dụ: 7 cho motor 14P)
 * @param  offset      Offset góc điện lúc Homing [rad]
 * @return Góc điện [rad] đã chuẩn hóa về [0, 2π]
 */
float Attitude_GetElecAngleRollRel(const Attitude_Handle_t *hatt,
                                   uint8_t pole_pairs,
                                   float offset);

/**
 * @brief  [Legacy] Tính góc điện từ góc TUYỆT ĐỐI payload Pitch.
 * @note   Khuyến nghị dùng Attitude_GetElecAnglePitchRel() thay thế.
 */
float Attitude_GetElecAngle(const Attitude_Handle_t *hatt,
                            uint8_t pole_pairs,
                            float offset);

/**
 * @brief  [Legacy] Tính góc điện từ góc TUYỆT ĐỐI payload Roll.
 * @note   Khuyến nghị dùng Attitude_GetElecAngleRollRel() thay thế.
 */
float Attitude_GetElecAngleRoll(const Attitude_Handle_t *hatt,
                                uint8_t pole_pairs,
                                float offset);

#ifdef __cplusplus
}
#endif

#endif /* ATTITUDE_H */
