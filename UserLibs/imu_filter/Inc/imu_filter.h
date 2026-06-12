#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include <stdint.h>
#include <math.h>

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

#endif /* IMU_FILTER_H */
