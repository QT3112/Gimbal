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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "mpu6050.h"
#include "as5048a.h"
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
float voltage_limit = 0.05f;
volatile float theta = 0.0f;
float velocity = 5.0f; // rad/s điện
float Ts = 0.0001f;    // 100us

/* --- MPU6050 --- */
MPU6050_Handle_t imu;  // Handle của cảm biến
uint8_t imu_ready = 0; // Cờ trạng thái: 1 = sẵn sàng

/* --- AS5048A Encoder --- */
AS5048A_Handle_t encoder;   // Handle encoder
uint8_t encoder_ready = 0;  // Cờ trạng thái: 1 = sẵn sàng
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setPhaseVoltage(float theta) {
  float Ua, Ub, Uc;

  Ua = 0.5f + voltage_limit * sinf(theta);
  Ub = 0.5f + voltage_limit * sinf(theta - 2.0f * PI / 3.0f);
  Uc = 0.5f + voltage_limit * sinf(theta - 4.0f * PI / 3.0f);

  uint16_t dutyA = (uint16_t)(Ua * PWM_PERIOD);
  uint16_t dutyB = (uint16_t)(Ub * PWM_PERIOD);
  uint16_t dutyC = (uint16_t)(Uc * PWM_PERIOD);

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dutyA);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, dutyB);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dutyC);
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
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1062); // 25%
  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 2125); // 50%
  // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 3187); // 75%

  /* --- Khởi tạo AS5048A Encoder ---
   * SPI1: đã được CubeMX init qua MX_SPI1_Init() ở trên
   * CS:   PC4 (GPIO_Output, mặc định HIGH)
   */
  {
    AS5048A_Status_t enc_ret = AS5048A_Init(&encoder, &hspi1, GPIOC, GPIO_PIN_4);
    printf("[AS5048A_Init] ma tra ve = %d "
           "(0=OK, 1=SPI_ERR, 2=PARITY_ERR, 3=EF, 4=CORDIC)\r\n", enc_ret);

    if (enc_ret == AS5048A_OK) {
      encoder_ready = 1;

      /* Đọc diagnostics để kiểm tra trạng thái nam châm */
      AS5048A_ReadDiagnostics(&encoder);
      printf("[AS5048A] AGC=%u | CompH=%u CompL=%u | COF=%u | OCF=%u\r\n",
             encoder.agc_value,
             encoder.comp_high,
             encoder.comp_low,
             encoder.cordic_overflow,
             encoder.offset_comp_finished);
    }
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    theta += 0.02f;

    if (theta > 2.0f * PI)
      theta -= 2.0f * PI;

    setPhaseVoltage(theta);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* --- Đọc MPU6050 --- */
    if (imu_ready) {
      MPU6050_ReadAll(&imu);
      printf("Ax=%.2f Ay=%.2f Az=%.2f | Gx=%.2f Gy=%.2f Gz=%.2f\r\n",
             imu.accel_x, imu.accel_y, imu.accel_z, imu.gyro_x, imu.gyro_y,
             imu.gyro_z);
    }

    /* --- Đọc AS5048A Encoder --- */
    if (encoder_ready) {
      AS5048A_Status_t ret = AS5048A_ReadAngle(&encoder);
      if (ret == AS5048A_OK) {
        printf("[ENC] Angle: %.2f deg | %.4f rad | Raw: %u\r\n",
               encoder.angle_deg,
               encoder.angle_rad,
               encoder.raw_angle);
      } else {
        /* Có lỗi: đọc và xóa error register */
        uint16_t err_bits = 0;
        AS5048A_ClearErrors(&encoder, &err_bits);
        printf("[ENC] Loi doc: status=%d | err_bits=0x%04X\r\n", ret, err_bits);
      }
    }

    HAL_Delay(10);
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
