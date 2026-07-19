/**
 * *****************************************************************************
 * @file    example_position_control.c
 * @brief   Ví dụ điều khiển vị trí góc dùng VÒNG KHÉP KÍN DÒNG ĐIỆN (True FOC Current Loop)
 *          Sử dụng kiến trúc Cascade 3-Loop: Position -> Velocity -> Current FOC.
 *
 * === CÁCH SỬ DỤNG ===
 * Tệp này chứa hàm main mẫu để chạy thử nghiệm điều khiển vị trí 1 trục.
 * Để chạy ví dụ này:
 * 1. Sao chép nội dung tệp này thay thế cho main.c hiện tại, hoặc
 * 2. Đổi tên tệp này thành main.c để biên dịch.
 *
 * === MÔ TẢ HOẠT ĐỘNG ===
 * - Lúc khởi động, đo và trung bình hóa offset cảm biến dòng (1000 mẫu).
 * - Căn chỉnh góc cực từ (alignment) bằng cách cấp điện áp thử nghiệm.
 * - Chạy vòng quét dòng điện tần số cao (20kHz) trong ngắt ADC Injected.
 * - Góc mục tiêu được giữ cố định. Khi bị tác động lệch góc cơ học, vòng vị trí
 *   và vận tốc ngoài sẽ tính toán dòng điện yêu cầu Iq_ref gửi đến vòng dòng FOC.
 * *****************************************************************************
 */

#if 0 /* Đổi thành #if 1 nếu bạn muốn biên dịch trực tiếp file này làm main.c */

#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "foc.h"
#include "as5048a.h"
#include "examples.h"
#include <stdio.h>

/* Định nghĩa phần cứng phụ trợ */
#define GAIN_DRV            10.0f
#define SHUNT_RES           0.005f
#define VOLTAGE_LIMIT       6.0f
#define CURRENT_LIMIT       1.0f
#define PWM_PERIOD          4249.0f
#define MOTOR_POLE_PAIRS    14

/* Khai báo đối tượng FOC và Encoder */
FOC_Handle_t foc_motor;
AS5048A_Handle_t motor_enc;

/* Biến hiệu chỉnh offset dòng điện */
float motor_offset_a = 0.0f;
float motor_offset_b = 0.0f;

/* Góc khóa mục tiêu (Radian): 180 độ = 3.14159 rad */
float target_angle_rad = 3.14159f;

int main(void)
{
  /* 1. Khởi tạo hệ thống HAL và Xung nhịp */
  HAL_Init();
  SystemClock_Config();

  /* 2. Khởi tạo ngoại vi */
  MX_GPIO_Init();
  MX_SPI1_Init();       /* SPI1 cho Encoder */
  MX_ADC1_Init();       /* ADC1 lấy mẫu dòng Ia, Ib */
  MX_TIM1_Init();       /* TIM1 Center-Aligned PWM 20kHz */
  MX_TIM6_Init();       /* TIM6 ngắt định thời 1kHz */

  /* Đảm bảo CS của Encoder ban đầu ở mức HIGH */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CS Pitch
  HAL_Delay(10);

  /* 3. Khởi tạo Encoder AS5048A */
  AS5048A_Init(&motor_enc, &hspi1, GPIOB, GPIO_PIN_12);

  /* 4. Khởi tạo FOC cấu hình Cascade */
  FOC_Init(&foc_motor, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, 
           PWM_PERIOD, MOTOR_POLE_PAIRS, VOLTAGE_LIMIT, 0.001f);

  /* Thiết lập các bộ điều khiển PID trong FOC */
  FOC_SetPID_D(&foc_motor, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_motor, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  
  /* Bật chế độ Vòng dòng khép kín (True FOC) chạy ở tần số ngắt 20kHz */
  FOC_EnableCurrentLoop(&foc_motor, 1.0f / 20000.0f);
  FOC_SetCurrentLimit(&foc_motor, CURRENT_LIMIT);

  /* Cấu hình PID vòng ngoài (Position & Velocity) */
  ExampleConfig_t ex_cfg = {
    .vel_Kp        = 3.20f,
    .vel_Ki        = 0.0f,
    .vel_Kd        = 0.0f,
    .vel_lpf_alpha = 0.9f,
    .pos_Kp        = 0.8f,
    .pos_Ki        = 0.0f,
    .pos_Kd        = 0.0f
  };
  Example_Init(&foc_motor, &ex_cfg);

  /* 5. Khởi động PWM cho TIM1 bao gồm cả CH4 (kích hoạt Trigger ADC) */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* OC4REF kích hoạt TRGO */

  /* 6. Đo và hiệu chỉnh offset dòng điện ban đầu khi chưa cấp điện động cơ */
  HAL_Delay(100);
  float sum_a = 0.0f, sum_b = 0.0f;
  for (int i = 0; i < 1000; i++) {
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);
    sum_a += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) * 3.3f / 4096.0f;
    sum_b += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) * 3.3f / 4096.0f;
  }
  motor_offset_a = sum_a / 1000.0f;
  motor_offset_b = sum_b / 1000.0f;
  printf("[ISNS] Calibrated Offset A: %.3fV | B: %.3fV\r\n", motor_offset_a, motor_offset_b);

  /* 7. Căn chỉnh cực từ động cơ (Align D-axis) */
  printf("[ALIGN] Motor Aligning...\r\n");
  FOC_AlignD(&foc_motor, 3.0f); /* Cấp điện áp thử nghiệm để kéo rotor về vị trí 0 điện */
  HAL_Delay(800);
  
  AS5048A_ReadAngle(&motor_enc);
  FOC_CalibrateAngle(&foc_motor, motor_enc.angle_rad);
  printf("[ALIGN] Align Complete! Offset: %.3f rad\r\n", foc_motor.angle_offset);

  /* 8. Bật vòng điều khiển chính */
  FOC_Start(&foc_motor, motor_enc.angle_rad);
  
  /* Kích hoạt ngắt định thời Timer 6 (1kHz) */
  HAL_TIM_Base_Start_IT(&htim6);

  /* Kích hoạt ngắt ADC Injected (đồng bộ PWM ở 20kHz) */
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  /* Vòng lặp vô hạn */
  while (1)
  {
    // Cập nhật Cascade Position loop (Vòng Ngoài)
    Example_HoldAngle(&foc_motor, &motor_enc, target_angle_rad);
    
    static uint32_t last_print = 0;
    if (HAL_GetTick() - last_print >= 200) {
      last_print = HAL_GetTick();
      printf("Target: %.1f deg | Current: %.1f deg | Iq_meas: %.3fA | Iq_ref: %.3fA\r\n",
             target_angle_rad * 57.29577f,
             motor_enc.angle_deg,
             foc_motor.Iq_meas,
             foc_motor.Iq_ref);
    }
    HAL_Delay(1);
  }
}

/**
  * @brief  Callback ngắt ADC Injected (Tần số cao ~20kHz)
  *         Thực thi vòng lặp dòng điện (Current Loop - Vòng Trong)
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    /* Đọc dòng pha Ia và Ib từ cảm biến (Rank 1 & Rank 2) */
    uint32_t raw_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    
    /* Đổi ra điện áp */
    float volt_a = (float)raw_ia * 3.3f / 4096.0f;
    float volt_b = (float)raw_ib * 3.3f / 4096.0f;
    
    /* Trừ offset hiệu chuẩn lúc khởi động */
    float v_a = volt_a - motor_offset_a;
    float v_b = volt_b - motor_offset_b;
    
    /* Tính dòng điện thực tế dựa trên độ nhạy mạch DRV8302 */
    float current_a = v_a / (GAIN_DRV * SHUNT_RES);
    float current_b = v_b / (GAIN_DRV * SHUNT_RES);
    
    /* Nội suy góc điện động cơ dựa vào tốc độ hiện tại */
    foc_motor.angle_elec += foc_motor.velocity_mech * foc_motor.pole_pairs * (1.0f / 20000.0f);
    if (foc_motor.angle_elec >= 6.28318530718f) {
      foc_motor.angle_elec -= 6.28318530718f;
    } else if (foc_motor.angle_elec < 0.0f) {
      foc_motor.angle_elec += 6.28318530718f;
    }
    
    /* Thực thi giải thuật điều khiển dòng điện True FOC */
    FOC_UpdateCurrentLoop(&foc_motor, current_a, current_b);
  }
}

/**
  * @brief  Callback ngắt Timer 6 (1kHz)
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    // Để trống vì FOC_Update chỉ dùng cho Voltage-Mode.
    // Với True FOC, việc điều khiển và băm xung PWM đã do FOC_UpdateCurrentLoop lo liệu.
  }
}

#endif
