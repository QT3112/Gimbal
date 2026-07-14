/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body — Motor homing to 0° and hold
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "as5048a.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD      4249.0f
#define PI              3.14159265359f
#define DEG_TO_RAD      (PI / 180.0f)
#define RAD_TO_DEG      (180.0f / PI)

/* --- Thông số motor --- */
#define MOTOR_POLE_PAIRS    14       /* Số cặp cực — chỉnh theo motor thực tế */
#define VOLTAGE_LIMIT       0.15f   /* Giới hạn điện áp [V] — tăng nếu motor yếu */
#define TS_S                0.01f   /* Chu kỳ điều khiển: 10ms (phù hợp HAL_Delay(10)) */

/* --- Align encoder --- */
#define ALIGN_VD            0.10f   /* Điện áp Vd khi align [V] */
#define ALIGN_DURATION_MS   600U    /* Thời gian giữ Vd để rotor lock [ms] */

/* --- PID vòng vị trí --- */
#define POS_KP              3.0f    /* Tỉ lệ: tăng nếu đáp ứng chậm */
#define POS_KI              0.05f   /* Tích phân: triệt sai số xác lập */
#define POS_KD              0.0f    /* Vi phân: tăng nếu overshoot nhiều */
#define POS_VEL_MAX         2.0f    /* Tốc độ cơ học tối đa ra PID [rad/s] */

/* --- PID vòng tốc độ (inner loop) --- */
#define VEL_KP              0.10f
#define VEL_KI              0.02f
#define VEL_KD              0.0f
#define VEL_LPF_ALPHA       0.85f   /* Lọc nhiễu tốc độ encoder */

/* --- Ngưỡng "đã về 0°" --- */
#define HOLD_THRESHOLD_RAD  (2.0f * DEG_TO_RAD)   /* ±2° = coi như đến nơi */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- AS5048A Encoder --- */
AS5048A_Handle_t encoder;
uint8_t encoder_ready = 0;

/* --- FOC Handle --- */
FOC_Handle_t foc;

/* --- PID vòng vị trí (position loop) --- */
FOC_PID_t pid_pos;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float clamp_f(float v, float mn, float mx);
static float wrap_angle(float a);          /* Chuẩn hóa về [-π, +π] */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Clamp float trong [mn, mx]
 */
static float clamp_f(float v, float mn, float mx)
{
    if (v < mn) return mn;
    if (v > mx) return mx;
    return v;
}

/**
 * @brief  Đưa góc về phạm vi [-π, +π]
 *         Dùng để tính sai số vị trí ngắn nhất (shortest path)
 */
static float wrap_angle(float a)
{
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_Device_Init();
  MX_I2C3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* --- Khởi động PWM 3 pha --- */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* Enable gate driver */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  // M-OC
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9,  GPIO_PIN_SET);    // OC-ADJ
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    // M-PWM

  /* --- Khởi tạo AS5048A Encoder (SPI1, CS=PC4) --- */
  {
    AS5048A_Status_t enc_ret = AS5048A_Init(&encoder, &hspi1, GPIOC, GPIO_PIN_4);
    printf("[AS5048A] Init=%d (0=OK)\r\n", enc_ret);
    if (enc_ret == AS5048A_OK) {
      encoder_ready = 1;
      AS5048A_ReadDiagnostics(&encoder);
      printf("[AS5048A] AGC=%u CompH=%u CompL=%u COF=%u OCF=%u\r\n",
             encoder.agc_value, encoder.comp_high, encoder.comp_low,
             encoder.cordic_overflow, encoder.offset_comp_finished);
    }
  }

  /* --- Khởi tạo FOC --- */
  FOC_Init(&foc,
           &htim2,
           TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD,
           MOTOR_POLE_PAIRS,
           VOLTAGE_LIMIT,
           TS_S);

  /* Cài PID tốc độ bên trong FOC */
  FOC_SetPID_Vel(&foc, VEL_KP, VEL_KI, VEL_KD, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc, VEL_LPF_ALPHA);

  /* Cài PID vị trí (position loop — ngoài FOC) */
  pid_pos.Kp         = POS_KP;
  pid_pos.Ki         = POS_KI;
  pid_pos.Kd         = POS_KD;
  pid_pos.integral   = 0.0f;
  pid_pos.prev_error = 0.0f;
  pid_pos.output_min = -POS_VEL_MAX;
  pid_pos.output_max =  POS_VEL_MAX;

  /* =========================================================================
   * PHASE 1: ALIGN — Lock rotor về D-axis để hiệu chỉnh offset encoder
   * =========================================================================
   * Áp Vd tại angle_elec=0 trong ALIGN_DURATION_MS ms.
   * Sau khi rotor đứng yên, đọc encoder → lưu làm angle_offset.
   * Không cần encoder sẵn sàng cho bước này (chỉ dùng Vd cố định).
   */
  printf("[ALIGN] Bat dau align encoder...\r\n");
  FOC_Start(&foc);

  uint32_t align_start = HAL_GetTick();
  while ((HAL_GetTick() - align_start) < ALIGN_DURATION_MS) {
    FOC_AlignD(&foc, ALIGN_VD);
    HAL_Delay(5);
  }

  /* Đọc góc encoder sau khi rotor đã lock */
  if (encoder_ready) {
    if (AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
      FOC_CalibrateAngle(&foc, encoder.angle_rad);
      printf("[ALIGN] Hoan thanh. angle_offset=%.4f rad (%.2f deg)\r\n",
             foc.angle_offset, foc.angle_offset * RAD_TO_DEG);
    } else {
      printf("[ALIGN] Loi doc encoder! Dung chuong trinh.\r\n");
      FOC_Stop(&foc);
      Error_Handler();
    }
  } else {
    /* Không có encoder → không thể chạy closed-loop */
    printf("[ALIGN] Encoder chua san sang! Dung chuong trinh.\r\n");
    FOC_Stop(&foc);
    Error_Handler();
  }

  printf("[HOME] Bat dau ve 0 do...\r\n");
  /* USER CODE END 2 */

  /* =========================================================================
   * PHASE 2: HOME → 0° và giữ vị trí (Position Hold Loop)
   * =========================================================================
   *
   * Kiến trúc cascade:
   *
   *  Target = 0 rad
   *       │
   *       ▼
   *   [PID_pos]  ← error = wrap(0 - angle_mech)
   *       │  target_vel [rad/s]
   *       ▼
   *   [FOC_RunVelocity]  ← angle_mech từ encoder
   *       │
   *       ▼
   *     PWM → Motor
   *
   * Khi motor đã trong ngưỡng HOLD_THRESHOLD_RAD thì in "[HOLD]".
   */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* --- Đọc encoder --- */
    AS5048A_Status_t enc_ret = AS5048A_ReadAngle(&encoder);
    if (enc_ret != AS5048A_OK) {
      /* Lỗi đọc: xóa lỗi, bỏ qua chu kỳ này để không gây giật motor */
      uint16_t err_bits = 0;
      AS5048A_ClearErrors(&encoder, &err_bits);
      printf("[ENC] Loi: status=%d err=0x%04X\r\n", enc_ret, err_bits);
      HAL_Delay(10);
      continue;
    }

    float angle_mech = encoder.angle_rad;  /* [0, 2π) */

    /* --- PID vòng vị trí: sai số theo đường ngắn nhất (shortest path) --- */
    float pos_error  = wrap_angle(0.0f - angle_mech);  /* target = 0 rad */
    float target_vel = FOC_PID_Update(&pid_pos, pos_error, TS_S);
    target_vel = clamp_f(target_vel, -POS_VEL_MAX, POS_VEL_MAX);

    /* --- Vòng tốc độ + FOC transforms → PWM --- */
    /* NOTE: Đảo dấu velocity vì chiều quay điện (FOC) ngược chiều encoder.
     *       Nếu motor vẫn dao động sau khi flash, thử bỏ dấu trừ hoặc
     *       hoán đổi 2 dây pha (B↔C) trên phần cứng. */
    FOC_RunVelocity(&foc, angle_mech, -target_vel);

    /* --- Log trạng thái --- */
    float err_deg = pos_error * RAD_TO_DEG;
    if (fabsf(pos_error) < HOLD_THRESHOLD_RAD) {
      printf("[HOLD] angle=%.2f deg | err=%.2f deg | vel=%.3f rad/s\r\n",
             encoder.angle_deg, err_deg, target_vel);
    } else {
      printf("[HOME] angle=%.2f deg | err=%.2f deg | vel=%.3f rad/s\r\n",
             encoder.angle_deg, err_deg, target_vel);
    }

    HAL_Delay(10);  /* Chu kỳ 10ms = TS_S */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
