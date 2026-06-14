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
#include "examples.h"
#include "foc.h"
#include "imu_filter.h"
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

/* --- IMU Filter --- */
MahonyFilter_t ahrs;
float pitch_angle = 0.0f;

/* --- Gimbal Angle PID --- */
FOC_PID_t pid_pitch;

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

  // --- KHỞI TẠO FOC ---
  FOC_Init(&foc, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           4249.0f, /* PWM Period (ARR) */
           7,       /* 12N14P = 7 cặp cực */
           0.5f,   /* voltage_limit [V] - tăng dần nếu motor không xoay */
           0.01f); /* Ts = 10ms - PHẢI khớp với HAL_Delay(10) */

  // --- KHỞI TẠO MPU6050 ---
  if (MPU6050_Init(&imu, &hi2c3, MPU6050_ADDR_LOW) == MPU6050_OK) {
    imu_ready = 1;
    MPU6050_CalibrateGyro(&imu, 500);
    printf("[MPU6050] Calibrate xong! Offset X:%.2f, Y:%.2f, Z:%.2f\r\n",
           imu.gyro_offset_x, imu.gyro_offset_y, imu.gyro_offset_z);
    Mahony_Init(&ahrs, 1.5f, 0.005f);
  } else {
    printf("[MPU6050] Loi khoi tao!\r\n");
  }

  // --- KHỞI TẠO AS5048A ---
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

  // --- CẤU HÌNH PID & LPF ---
  // LPF alpha=0.85: lọc vừa đủ, phản ứng nhanh hơn alpha=0.9
  FOC_SetLPF_Vel(&foc, 0.85f);
  FOC_SetPID_Vel(&foc, 0.1f, 0.01f, 0.0f, -foc.voltage_limit, foc.voltage_limit);

  pid_pitch.Kp = 2.0f;
  pid_pitch.Ki = 0.0f;
  pid_pitch.Kd = 0.0f;
  pid_pitch.output_min = -10.0f;
  pid_pitch.output_max = 10.0f;
  FOC_PID_Reset(&pid_pitch);

  // --- ALIGNMENT ---
  FOC_Start(&foc);
  for (int i = 0; i < 50; i++) {
    FOC_AlignD(&foc, foc.voltage_limit);
    HAL_Delay(10);
  }
  if (AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
    FOC_CalibrateAngle(&foc, encoder.angle_rad);
    foc.prev_angle_mech = encoder.angle_rad;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // BƯỚC 2: Đọc IMU
    if (imu_ready) {
      if (MPU6050_ReadAll(&imu) == MPU6050_OK) {
        float gx = imu.gyro_x * (PI / 180.0f);
        float gy = imu.gyro_y * (PI / 180.0f);
        float gz = imu.gyro_z * (PI / 180.0f);
        Mahony_Update(&ahrs, gx, gy, gz, imu.accel_x, imu.accel_y, imu.accel_z,
                      0.01f);
        pitch_angle = ahrs.pitch * (180.0f / PI);
      }
    }

    // BƯỚC 3 & 4: Cascade PID Gimbal
    float target_pitch_angle = 0.0f;
    float pitch_error_rad = (target_pitch_angle - pitch_angle) * (PI / 180.0f);
    float target_vel_rad_s = FOC_PID_Update(&pid_pitch, pitch_error_rad, 0.01f);
    if (encoder_ready) {
      if (AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
        FOC_RunVelocity(&foc, encoder.angle_rad, target_vel_rad_s);
        printf(
            "[GIMBAL] pitch=%.1f | vel_set=%.2f | vel_meas=%.2f | Vq=%.3f\r\n",
            pitch_angle, target_vel_rad_s, foc.velocity_mech, foc.Vq_ref);
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
