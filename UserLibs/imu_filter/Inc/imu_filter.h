#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>
#include <math.h>
#include "icm42688.h"

/* =================================================================================
 * COMPLEMENTARY FILTER (Bộ lọc bù - 1 Trục)
 * Ưu điểm: Đơn giản, cực kỳ nhẹ (chỉ tốn vài phép tính), tốc độ hội tụ nhanh.
 * Nhược điểm: Phản hồi có thể bị trễ nhẹ nếu alpha quá lớn.
 * Phù hợp: Gimbal 1 trục, cân bằng xe 2 bánh, hoặc khi vi điều khiển yếu.
 * ================================================================================= */
typedef struct {
    float alpha;    // Hệ số tin cậy vào Gyro (thường là 0.96 -> 0.99)
    float angle;    // Góc đầu ra hiện tại (độ hoặc radian tùy vào đầu vào)
} ComplementaryFilter_t;

/**
 * @brief Khởi tạo bộ lọc Complementary
 */
void CompFilter_Init(ComplementaryFilter_t *filter, float alpha);

/**
 * @brief Cập nhật góc từ gia tốc và vận tốc góc
 * @param accel_angle: Góc tính toán được từ Accelerometer (qua hàm atan2)
 * @param gyro_rate: Vận tốc góc đo được từ Gyroscope
 * @param dt: Thời gian lấy mẫu (giây)
 * @return Góc đã lọc
 */
float CompFilter_Update(ComplementaryFilter_t *filter, float accel_angle, float gyro_rate, float dt);

/* =================================================================================
 * KALMAN FILTER (Bộ lọc Kalman - 1 Trục)
 * Ưu điểm: Khử nhiễu rất tốt, tự động tính toán bù nhiễu trôi (drift/bias) của gyro.
 * Nhược điểm: Cần nhiều phép toán ma trận hơn Complementary, cần tinh chỉnh Q và R.
 * Phù hợp: Hệ thống yêu cầu độ mịn cao trên 1 trục (Pitch hoặc Roll).
 * ================================================================================= */
typedef struct {
    float Q_angle;   // Process noise variance for the accelerometer
    float Q_bias;    // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance (nhiễu của accel)

    float angle;     // Góc ước lượng
    float bias;      // Độ trôi (bias) ước lượng của gyro
    float rate;      // Vận tốc góc sau khi đã khử bias

    float P[2][2];   // Error covariance matrix
} KalmanFilter_t;

/**
 * @brief Khởi tạo bộ lọc Kalman (thiết lập thông số Q, R mặc định)
 */
void Kalman_Init(KalmanFilter_t *kf);

/**
 * @brief Cập nhật và tính toán góc theo thuật toán Kalman
 * @param newAngle: Góc tính từ Accelerometer (atan2)
 * @param newRate: Tốc độ góc từ Gyroscope
 * @param dt: Thời gian lấy mẫu
 * @return Góc đã lọc
 */
float Kalman_Update(KalmanFilter_t *kf, float newAngle, float newRate, float dt);

/* =================================================================================
 * MAHONY AHRS FILTER (Bộ lọc Mahony - 3 Trục Không Gian)
 * Ưu điểm: Dùng Quaternion tránh Gimbal Lock, tính toán được cả 3 trục không gian
 *          nhẹ hơn Kalman 3 trục rất nhiều. Bù drift cực kỳ hiệu quả.
 * Nhược điểm: Cần chỉnh Kp, Ki. Cần cung cấp đầy đủ data cả 3 trục.
 * Phù hợp: Drone, Gimbal 3 trục, MPU6050, hệ thống Navigation.
 * ================================================================================= */
typedef struct {
    float Kp;            // Proportional gain (tốc độ hội tụ về lực hấp dẫn)
    float Ki;            // Integral gain (tốc độ khử bias trôi của Gyro)
    
    float q0, q1, q2, q3; // Quaternion biểu diễn hướng không gian
    float eInt_x, eInt_y, eInt_z; // Các thành phần tích phân bù sai số
    
    float roll, pitch, yaw; // Góc Euler đầu ra (Radian)
} MahonyFilter_t;

/**
 * @brief Khởi tạo Mahony (đặt quaternion về mặc định)
 * @param Kp: Mặc định thường dùng 0.5 - 2.0
 * @param Ki: Mặc định thường dùng 0.001 - 0.01 (hoặc 0 nếu không cần bù drift chậm)
 */
void Mahony_Init(MahonyFilter_t *mahony, float Kp, float Ki);

/**
 * @brief Cập nhật Quaternion và tính ra Roll/Pitch/Yaw
 * @param gx, gy, gz: Tốc độ góc từ Gyro (Đơn vị: Radian/s)
 * @param ax, ay, az: Gia tốc từ Accel (Đơn vị: G hoặc m/s2 đều được vì sẽ tự normalize)
 * @param dt: Thời gian lấy mẫu (giây)
 */
void Mahony_Update(MahonyFilter_t *mahony, float gx, float gy, float gz, float ax, float ay, float az, float dt);

/**
 * @brief Cập nhật Quaternion và tính ra Roll/Pitch/Yaw trực tiếp từ cấu trúc dữ liệu ICM42688
 * @param mahony: Con trỏ đến cấu trúc MahonyFilter_t
 * @param imu: Con trỏ đến cấu trúc ICM42688_Handle_t chứa dữ liệu thô và đã đổi từ cảm biến
 * @param dt: Thời gian lấy mẫu (giây)
 */
void Mahony_Update_ICM42688(MahonyFilter_t *mahony, const ICM42688_Handle_t *imu, float dt);

/* =================================================================================
 * QUATERNION MATH & 3D ORIENTATION (Chống Gimbal Lock)
 * ================================================================================= */
typedef struct {
    float q0; /* Scalar (w) */
    float q1; /* Vector X */
    float q2; /* Vector Y */
    float q3; /* Vector Z */
} Quaternion_t;

/**
 * @brief Nhân hai Quaternion: result = q1 * q2
 */
void Quaternion_Multiply(const Quaternion_t *q1, const Quaternion_t *q2, Quaternion_t *result);

/**
 * @brief Nghịch đảo / Liên hợp Quaternion: result = q* (cho Quaternion đơn vị)
 */
void Quaternion_Conjugate(const Quaternion_t *q, Quaternion_t *result);

/**
 * @brief Chuyển đổi Quaternion sang góc Euler (Roll, Pitch, Yaw tính theo Radian)
 */
void Quaternion_ToEuler(const Quaternion_t *q, float *roll, float *pitch, float *yaw);

/**
 * @brief Tạo Quaternion từ góc Euler (Roll, Pitch, Yaw tính theo Radian)
 */
void Quaternion_FromEuler(float roll, float pitch, float yaw, Quaternion_t *q);

/**
 * @brief Tính toán Vector sai số góc quay 3D (3D Rotation Error Vector) giữa Target & Measured
 *        dùng cho vòng lặp điều khiển FOC PID không dính Gimbal Lock.
 * @param q_target Quaternion mục tiêu
 * @param q_meas Quaternion đo được hiện tại từ AHRS
 * @param e_rot Mang 3 phần tử [e_x, e_y, e_z] nhận vector sai số góc (Radian)
 */
void Quaternion_ComputeError(const Quaternion_t *q_target, const Quaternion_t *q_meas, float e_rot[3]);

#endif /* IMU_FILTER_H */

