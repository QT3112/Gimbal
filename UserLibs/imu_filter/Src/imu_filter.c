#include "imu_filter.h"

/* =================================================================================
 * COMPLEMENTARY FILTER (1-Axis)
 * ================================================================================= */
void CompFilter_Init(ComplementaryFilter_t *filter, float alpha) {
    filter->alpha = alpha;
    filter->angle = 0.0f;
}

float CompFilter_Update(ComplementaryFilter_t *filter, float accel_angle, float gyro_rate, float dt) {
    // Thuật toán cốt lõi: Kết hợp góc ước lượng từ Gyro và góc thô từ Accel
    filter->angle = filter->alpha * (filter->angle + gyro_rate * dt) + (1.0f - filter->alpha) * accel_angle;
    return filter->angle;
}

/* =================================================================================
 * KALMAN FILTER (1-Axis)
 * ================================================================================= */
void Kalman_Init(KalmanFilter_t *kf) {
    // Các thông số mặc định (có thể điều chỉnh dựa trên cảm biến thực tế)
    kf->Q_angle = 0.001f;
    kf->Q_bias = 0.003f;
    kf->R_measure = 0.03f; // Giả sử nhiễu Accel tương đối thấp

    kf->angle = 0.0f;
    kf->bias = 0.0f;
    kf->rate = 0.0f;

    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
}

float Kalman_Update(KalmanFilter_t *kf, float newAngle, float newRate, float dt) {
    // --- Bước 1: Dự đoán (Predict) ---
    // Loại bỏ bias ra khỏi vận tốc góc
    kf->rate = newRate - kf->bias;
    kf->angle += dt * kf->rate;

    // Cập nhật ma trận sai số hiệp phương sai P
    kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
    kf->P[0][1] -= dt * kf->P[1][1];
    kf->P[1][0] -= dt * kf->P[1][1];
    kf->P[1][1] += kf->Q_bias * dt;

    // --- Bước 2: Cập nhật / Đo lường (Update) ---
    // Tính toán sai số (Innovation) giữa giá trị đo được (Accel) và dự đoán
    float y = newAngle - kf->angle;
    
    // Đo lường hiệp phương sai
    float S = kf->P[0][0] + kf->R_measure;
    
    // Tính hệ số Kalman (Kalman Gain)
    float K[2];
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Cập nhật lại góc và bias
    kf->angle += K[0] * y;
    kf->bias += K[1] * y;

    // Cập nhật lại ma trận P
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];

    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;

    return kf->angle;
}

/* =================================================================================
 * MAHONY AHRS FILTER (3-Axis)
 * ================================================================================= */

// Hàm tính nghịch đảo căn bậc hai (Fast Inverse Square Root) rất nhanh
static float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    int32_t i = *(int32_t*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(MahonyFilter_t *mahony, float Kp, float Ki) {
    mahony->Kp = Kp;  
    mahony->Ki = Ki;  
    
    // Quaternion ở tư thế cân bằng ban đầu
    mahony->q0 = 1.0f;
    mahony->q1 = 0.0f;
    mahony->q2 = 0.0f;
    mahony->q3 = 0.0f;
    
    mahony->eInt_x = 0.0f;
    mahony->eInt_y = 0.0f;
    mahony->eInt_z = 0.0f;
}

void Mahony_Update(MahonyFilter_t *mahony, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;

    // Bỏ qua tính toán nếu Accel không hợp lệ (đang rơi tự do hoặc lỗi cảm biến)
    if(ax == 0.0f && ay == 0.0f && az == 0.0f) return;

    // Chuẩn hóa vector Accel (để đưa về đơn vị vector độ dài 1)
    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Ước lượng hướng của trọng lực dựa trên quaternion hiện tại
    vx = 2.0f * (mahony->q1 * mahony->q3 - mahony->q0 * mahony->q2);
    vy = 2.0f * (mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3);
    vz = mahony->q0 * mahony->q0 - mahony->q1 * mahony->q1 - mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3;

    // Tính toán sai số (Dùng Cross Product giữa trọng lực đo được và trọng lực ước lượng)
    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    // Tính và bù sai số tích phân (để khử bias drift lâu dài của gyro)
    if(mahony->Ki > 0.0f) {
        mahony->eInt_x += ex * dt;
        mahony->eInt_y += ey * dt;
        mahony->eInt_z += ez * dt;
        
        gx += mahony->Ki * mahony->eInt_x;
        gy += mahony->Ki * mahony->eInt_y;
        gz += mahony->Ki * mahony->eInt_z;
    }

    // Bù sai số tỉ lệ (để giúp góc hội tụ nhanh về hướng của Accel)
    gx += mahony->Kp * ex;
    gy += mahony->Kp * ey;
    gz += mahony->Kp * ez;

    // Cập nhật Quaternion dựa trên Gyro đã được hiệu chỉnh
    float q0_dot = 0.5f * (-mahony->q1 * gx - mahony->q2 * gy - mahony->q3 * gz);
    float q1_dot = 0.5f * ( mahony->q0 * gx + mahony->q2 * gz - mahony->q3 * gy);
    float q2_dot = 0.5f * ( mahony->q0 * gy - mahony->q1 * gz + mahony->q3 * gx);
    float q3_dot = 0.5f * ( mahony->q0 * gz + mahony->q1 * gy - mahony->q2 * gx);

    mahony->q0 += q0_dot * dt;
    mahony->q1 += q1_dot * dt;
    mahony->q2 += q2_dot * dt;
    mahony->q3 += q3_dot * dt;

    // Chuẩn hóa lại Quaternion
    norm = invSqrt(mahony->q0 * mahony->q0 + mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3);
    mahony->q0 *= norm;
    mahony->q1 *= norm;
    mahony->q2 *= norm;
    mahony->q3 *= norm;

    // Trích xuất góc Euler (Roll, Pitch, Yaw)
    // Các góc đầu ra được tính bằng Radian.
    mahony->roll  = atan2f(2.0f * (mahony->q0 * mahony->q1 + mahony->q2 * mahony->q3), 1.0f - 2.0f * (mahony->q1 * mahony->q1 + mahony->q2 * mahony->q2));
    mahony->pitch = asinf(2.0f * (mahony->q0 * mahony->q2 - mahony->q3 * mahony->q1));
    mahony->yaw   = atan2f(2.0f * (mahony->q0 * mahony->q3 + mahony->q1 * mahony->q2), 1.0f - 2.0f * (mahony->q2 * mahony->q2 + mahony->q3 * mahony->q3));
}
