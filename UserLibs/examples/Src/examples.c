/**
 ******************************************************************************
 * @file    examples.c
 * @brief   Triển khai các hàm ví dụ điều khiển BLDC Motor qua FOC
 ******************************************************************************
 */

#include "examples.h"
#include <math.h>

/* ===========================================================================
 * Biến nội bộ (private) — chỉ dùng trong file này
 * ===========================================================================
 */

/* PID vòng vị trí dùng riêng cho Example_HoldAngle */
static FOC_PID_t s_pid_pos;

/* Cờ để biết Example_Init đã được gọi chưa */
static uint8_t s_initialized = 0;

/* ===========================================================================
 * Example_Init
 * ===========================================================================
 */
void Example_Init(FOC_Handle_t *hfoc, const ExampleConfig_t *cfg) {
  /* --- Cấu hình LPF và PID vòng vận tốc (vòng trong, dùng chung) --- */
  FOC_SetLPF_Vel(hfoc, cfg->vel_lpf_alpha);
  FOC_SetPID_Vel(hfoc, cfg->vel_Kp, cfg->vel_Ki, cfg->vel_Kd,
                 -hfoc->voltage_limit, hfoc->voltage_limit);

  /* --- Cấu hình PID vòng vị trí (chỉ dùng cho HoldAngle) --- */
  s_pid_pos.Kp = cfg->pos_Kp;
  s_pid_pos.Ki = cfg->pos_Ki;
  s_pid_pos.Kd = cfg->pos_Kd;
  /* Giới hạn đầu ra vòng vị trí = tốc độ góc tối đa cho phép [rad/s].
   * 10 rad/s ≈ 1.6 vòng/giây — an toàn cho gimbal.
   * Tăng nếu muốn motor phản ứng tốc độ cao hơn. */
  s_pid_pos.output_min = -10.0f;
  s_pid_pos.output_max = 10.0f;
  FOC_PID_Reset(&s_pid_pos);

  s_initialized = 1;
}

/* ===========================================================================
 * EXAMPLE 1: Xoay tốc độ cố định
 * ===========================================================================
 */
void Example_RunConstantVelocity(FOC_Handle_t *hfoc, AS5048A_Handle_t *henc,
                                 float target_vel) {
  /* Bảo vệ: chỉ chạy nếu đã khởi tạo và FOC đang bật */
  if (!s_initialized || !hfoc->enabled)
    return;

  /* Đọc góc encoder.
   * Nếu đọc lỗi (SPI fail, magnet lỗi...) thì bỏ qua chu kỳ này
   * để tránh nạp giá trị rác vào FOC. */
  if (AS5048A_ReadAngle(henc) != AS5048A_OK)
    return;

  /* FOC_RunVelocity thực hiện toàn bộ pipeline:
   *   dθ/dt → LPF → PID velocity → Vq → InvPark → InvClarke → PWM
   * Chỉ cần truyền vào góc encoder thực và tốc độ mục tiêu. */
  FOC_RunVelocity(hfoc, henc->angle_rad, target_vel);
}

/* ===========================================================================
 * EXAMPLE 2: Giữ góc cố định (Cascade Position Control)
 * ===========================================================================
 */
void Example_HoldAngle(FOC_Handle_t *hfoc, AS5048A_Handle_t *henc,
                       float target_angle) {
  if (!s_initialized || !hfoc->enabled)
    return;

  /* Đọc góc hiện tại từ encoder */
  if (AS5048A_ReadAngle(henc) != AS5048A_OK)
    return;

  float current_angle = henc->angle_rad; /* Góc cơ học thực tế [0, 2π) */

  /* -----------------------------------------------------------------------
   * VÒNG NGOÀI: PID Vị trí → Vận tốc mục tiêu
   *
   * Tính sai số góc có xử lý wrap-around (encoder nhảy qua 0/2π).
   * Ví dụ: target=0.1rad, current=6.2rad → sai số thực = 0.1-6.2+2π ≈ 0.18
   * Không xử lý sẽ bị sai số giả ~6 rad khiến motor quay điên cuồng.
   * ----------------------------------------------------------------------- */
  float angle_error = target_angle - current_angle;

  /* Chuẩn hóa sai số về [-π, +π] để luôn chọn đường đi ngắn nhất */
  while (angle_error > (float)M_PI)
    angle_error -= 2.0f * (float)M_PI;
  while (angle_error < -(float)M_PI)
    angle_error += 2.0f * (float)M_PI;

  /* PID vòng vị trí: Đầu ra là vận tốc góc mục tiêu [rad/s] */
  float target_vel = FOC_PID_Update(&s_pid_pos, angle_error, hfoc->Ts);

  /* -----------------------------------------------------------------------
   * VÒNG TRONG: FOC Velocity Control → PWM
   *
   * FOC_RunVelocity nhận vận tốc mục tiêu từ vòng vị trí và điều khiển
   * motor để đạt được vận tốc đó (PID vận tốc + FOC transforms).
   * ----------------------------------------------------------------------- */
  FOC_RunVelocity(hfoc, current_angle, target_vel);
}
