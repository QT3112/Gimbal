/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5048a.h"
#include "foc.h"
#include "math.h"
#include "mpu6050.h"
#include "stdint.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD 4249.0f
#define PI 3.14159265359f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* --- MPU6050 --- */
MPU6050_Handle_t imu;
uint8_t imu_ready = 0;

/* --- AS5048A Encoder --- */
AS5048A_Handle_t encoder;
uint8_t encoder_ready = 0;

/* --- FOC Controller --- */
FOC_Handle_t foc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  /* USER CODE BEGIN 2 */

  /* --- Khởi động PWM 3 pha --- */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  /* PC6: Enable driver (nếu có gate driver) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  /* --- Khởi tạo FOC ---
   * TIM2: 3 kênh PWM, PWM_PERIOD=4249
   * Pole pairs: 7 (thay đổi theo động cơ thực tế)
   * voltage_limit: 1.0V (giảm nếu motor nóng)
   * Ts: 0.01s (khớp với HAL_Delay(10) trong vòng lặp)
   */
  FOC_Init(&foc, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           4249.0f, /* PWM Period (ARR) */
           7,       /* Số cặp cực - chỉnh theo motor */
           0.05f,   /* voltage_limit [V] */
           0.01f);  /* Ts = 10ms */

  /* --- Khởi tạo AS5048A Encoder --- */
  {
    AS5048A_Status_t enc_ret =
        AS5048A_Init(&encoder, &hspi1, GPIOC, GPIO_PIN_4);
    printf("[AS5048A_Init] ma tra ve = %d "
           "(0=OK, 1=SPI_ERR, 2=PARITY_ERR, 3=EF, 4=CORDIC)\r\n",
           enc_ret);

    if (enc_ret == AS5048A_OK) {
      encoder_ready = 1;
      AS5048A_ReadDiagnostics(&encoder);
      printf("[AS5048A] AGC=%u | CompH=%u CompL=%u | COF=%u | OCF=%u\r\n",
             encoder.agc_value, encoder.comp_high, encoder.comp_low,
             encoder.cordic_overflow, encoder.offset_comp_finished);
    }
  }

  /* Bật FOC - sẵn sàng chạy closed-loop */
  /* --- Cấu hình bộ lọc tốc độ (LPF) ---
   * alpha = 0.9: lọc mạnh, phù hợp cho Ts=10ms
   * Giảm alpha (0.7~0.8) nếu muốn đáp ứng nhanh hơn */
  FOC_SetLPF_Vel(&foc, 0.9f);

  /* --- Cấu hình PID vòng tốc độ ---
   * Điểm khởi đầu an toàn: Kp nhỏ, Ki nhỏ
   * Output clamp trong ±voltage_limit để bảo vệ motor
   * Tăng Kp nếu motor phản ứng quá chậm
   * Tăng Ki nếu tốc độ bị sai số xác lập (không đạt setpoint)
   * Thêm Kd nếu có overshoot */
  FOC_SetPID_Vel(&foc,
                 0.008f,    /* Kp */
                 0.004f,    /* Ki */
                 0.0f,      /* Kd */
                 -foc.voltage_limit,
                  foc.voltage_limit);

  FOC_Start(&foc);

  /* --- CĂN CHỈNH GÓC ENCODER (ALIGNMENT) ---
   * Bắt buộc thực hiện để FOC biết vị trí zero điện tuyệt đối.
   * Nếu bỏ qua bước này, angle_elec tính sai -> áp lực sai -> motor rung/nóng */
  if (encoder_ready) {
    printf("[ALIGN] Dang can chinh rotor...\r\n");
    /* Giữ Vd để kéo rotor về vị trí D-axis tuyệt đối.
     * Sử dụng voltage_limit để đảm bảo đủ lực mà không quá dòng */
    for (int i = 0; i < 50; i++) {
        FOC_AlignD(&foc, foc.voltage_limit);
        HAL_Delay(10);
    }

    /* Đọc encoder sau khi rotor đã đứng yên hoàn toàn */
    if (AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
        FOC_CalibrateAngle(&foc, encoder.angle_rad);
        foc.prev_angle_mech = encoder.angle_rad;
        printf("[ALIGN] Xong! Offset = %.2f rad\r\n", foc.angle_offset);
    }
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ===========================================================
     * Closed-loop Velocity Control
     *
     * FOC_RunVelocity():
     *   1. Đọc angle encoder
     *   2. Tính tốc độ = dθ/dt (vi phân)
     *   3. Lọc LPF để loại nhiễu
     *   4. PID: (target - vel_filtered) → Vq
     *   5. InvPark + InvClarke → PWM
     *
     * Tốc độ đặt: 1.0 vòng/s ≈ 6.28 rad/s cơ học
     * Đổi dấu để đảo chiều quay
     * =========================================================== */
    float target_vel = 1.0f * FOC_TWO_PI; /* [rad/s] cơ học = 1 vòng/giây */

    if (encoder_ready) {
      if (AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
        FOC_RunVelocity(&foc, encoder.angle_rad, target_vel);

        /* Log để quan sát (có thể tắt để cải thiện timing) */
        printf("[VEL] set=%.2f | meas=%.2f | Vq=%.3f | angle=%.1f\r\n",
               target_vel,
               foc.velocity_mech,
               foc.Vq_ref,
               encoder.angle_deg);
      }
    }

    HAL_Delay(10); /* Ts = 10ms — khớp với FOC_Init Ts=0.01f */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSI48;
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
