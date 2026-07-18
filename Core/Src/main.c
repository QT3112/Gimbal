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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "as5048a.h"
#include "icm42688.h"
#include "pid_lib.h"
#include "math.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ENC_STATE_IDLE = 0,
  ENC_STATE_YAW,
  ENC_STATE_PITCH,
  ENC_STATE_ROLL
} Encoder_State_t;

typedef enum {
  IMU_STATE_IDLE = 0,
  IMU_STATE_FRAME,
  IMU_STATE_PAYLOAD
} IMU_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FOC_PI          3.14159265359f
#define DEG_TO_RAD      (FOC_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / FOC_PI)
#define TWO_PI          6.28318530718f

#define MOTOR_POLE_PAIRS    14
#define VOLTAGE_LIMIT       6.0f
#define SUPPLY_VOLTAGE      12.0f
#define PWM_PERIOD          4249.0f

#define GAIN_DRV   10.0f
#define SHUNT_RES  0.005f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// Sensor Handles
AS5048A_Handle_t yaw_enc;
AS5048A_Handle_t pitch_enc;
AS5048A_Handle_t roll_enc;

ICM42688_Handle_t frame_imu;
ICM42688_Handle_t payload_imu;

// FOC Handles
FOC_Handle_t foc_pitch;
FOC_Handle_t foc_roll;
FOC_Handle_t foc_yaw;

// Outer Loop Position PIDs
PID_Handle_t pid_pos_pitch;
PID_Handle_t pid_pos_roll;
PID_Handle_t pid_pos_yaw;

// SPI DMA Buffers & States
volatile Encoder_State_t enc_state = ENC_STATE_IDLE;
uint8_t enc_tx_buf[4] = { 0xFF, 0xFF, 0xC0, 0x00 };
uint8_t enc_rx_buf[4];

volatile IMU_State_t imu_state = IMU_STATE_IDLE;
uint8_t imu_tx_buf[15] = { 0x1D | 0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
uint8_t imu_rx_buf[15];

// Current Calibration Offsets
float pitch_offset_a = 0.0f;
float pitch_offset_b = 0.0f;
float roll_offset_a = 0.0f;
float roll_offset_b = 0.0f;
float yaw_offset_a = 0.0f;
float yaw_offset_b = 0.0f;

// Control Target Angles (degrees)
float pitch_target_deg = 0.0f;
float roll_target_deg = 0.0f;
float yaw_target_deg = 0.0f;

// Estimated Attitude States (degrees)
float payload_pitch_deg = 0.0f;
float payload_roll_deg = 0.0f;
float frame_pitch_deg = 0.0f;
float frame_roll_deg = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline float _normalize_angle(float angle)
{
    float a = fmodf(angle, 6.28318530718f);
    return (a < 0.0f) ? (a + 6.28318530718f) : a;
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
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // Disconnect/disable all CS lines (HIGH)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // YAW CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // PITCH CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // ROLL CS
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // FRAME CS
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);  // PAYLOAD CS
  HAL_Delay(10);

  // Initialize Encoders
  AS5048A_Init(&yaw_enc, &hspi1, GPIOB, GPIO_PIN_14);
  AS5048A_Init(&pitch_enc, &hspi1, GPIOB, GPIO_PIN_12);
  AS5048A_Init(&roll_enc, &hspi1, GPIOB, GPIO_PIN_13);

  // Initialize IMUs
  ICM42688_Init(&frame_imu, &hspi3, GPIOB, GPIO_PIN_15, NULL);
  ICM42688_Init(&payload_imu, &hspi3, GPIOC, GPIO_PIN_6, NULL);

  // Initialize FOC Structs
  // Pitch (TIM1, ADC1 - Closed Loop Current FOC)
  FOC_Init(&foc_pitch, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, PWM_PERIOD, MOTOR_POLE_PAIRS, VOLTAGE_LIMIT, 0.001f);
  FOC_SetPID_D(&foc_pitch, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_pitch, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Vel(&foc_pitch, 3.20f, 0.0f, 0.0f, -1.0f, 1.0f);
  FOC_EnableCurrentLoop(&foc_pitch, 1.0f / 20000.0f);
  FOC_SetCurrentLimit(&foc_pitch, 1.0f);

  // Roll (TIM8, ADC1 - Closed Loop Current FOC)
  FOC_Init(&foc_roll, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, PWM_PERIOD, MOTOR_POLE_PAIRS, VOLTAGE_LIMIT, 0.001f);
  FOC_SetPID_D(&foc_roll, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_roll, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Vel(&foc_roll, 3.20f, 0.0f, 0.0f, -1.0f, 1.0f);
  FOC_EnableCurrentLoop(&foc_roll, 1.0f / 20000.0f);
  FOC_SetCurrentLimit(&foc_roll, 1.0f);

  // Yaw (TIM3, ADC2 - Closed Loop Current FOC)
  FOC_Init(&foc_yaw, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, PWM_PERIOD, MOTOR_POLE_PAIRS, VOLTAGE_LIMIT, 0.001f);
  FOC_SetPID_D(&foc_yaw, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_yaw, 0.5f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Vel(&foc_yaw, 3.20f, 0.0f, 0.0f, -1.0f, 1.0f);
  FOC_EnableCurrentLoop(&foc_yaw, 1.0f / 20000.0f);
  FOC_SetCurrentLimit(&foc_yaw, 1.0f);

  // Position PIDs
  PID_Init(&pid_pos_pitch, 0.8f, 0.0f, 0.0f, -1.5f, 1.5f);
  PID_Init(&pid_pos_roll, 0.8f, 0.0f, 0.0f, -1.5f, 1.5f);
  PID_Init(&pid_pos_yaw, 0.8f, 0.0f, 0.0f, -1.5f, 1.5f);

  // Start PWM for TIM1/3/8 (and TIM1 CH4 as TRGO)
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // Calibrate Current Sensor Offsets
  HAL_Delay(100);
  float sum_pitch_a = 0.0f, sum_pitch_b = 0.0f;
  float sum_roll_a = 0.0f, sum_roll_b = 0.0f;
  float sum_yaw_a = 0.0f, sum_yaw_b = 0.0f;
  for (int i = 0; i < 1000; i++) {
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);
    HAL_ADCEx_InjectedPollForConversion(&hadc2, 10);
    sum_pitch_a += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) * 3.3f / 4096.0f;
    sum_pitch_b += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) * 3.3f / 4096.0f;
    sum_roll_a  += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3) * 3.3f / 4096.0f;
    sum_roll_b  += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4) * 3.3f / 4096.0f;
    sum_yaw_a   += (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1) * 3.3f / 4096.0f;
    sum_yaw_b   += (float)HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2) * 3.3f / 4096.0f;
  }
  // Tính offset bằng cách lấy trung bình 1000 lần 
  pitch_offset_a = sum_pitch_a / 1000.0f;
  pitch_offset_b = sum_pitch_b / 1000.0f;
  roll_offset_a  = sum_roll_a / 1000.0f;
  roll_offset_b  = sum_roll_b / 1000.0f;
  yaw_offset_a   = sum_yaw_a / 1000.0f;
  yaw_offset_b   = sum_yaw_b / 1000.0f;

  printf("[ISNS] Pitch Offset A: %.3fV | B: %.3fV\r\n", pitch_offset_a, pitch_offset_b);
  printf("[ISNS] Roll Offset A: %.3fV | B: %.3fV\r\n", roll_offset_a, roll_offset_b);
  printf("[ISNS] Yaw Offset A: %.3fV | B: %.3fV\r\n", yaw_offset_a, yaw_offset_b);

  // Align Motors
  printf("[ALIGN] Aligning motors...\r\n");
  
  FOC_AlignD(&foc_pitch, 3.0f);
  HAL_Delay(600);
  AS5048A_ReadAngle(&pitch_enc);
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  
  // FOC_AlignD(&foc_roll, 3.0f);
  // HAL_Delay(600);
  // AS5048A_ReadAngle(&roll_enc);
  // FOC_CalibrateAngle(&foc_roll, roll_enc.angle_rad);

  // FOC_AlignD(&foc_yaw, 3.0f);
  // HAL_Delay(600);
  // AS5048A_ReadAngle(&yaw_enc);
  // FOC_CalibrateAngle(&foc_yaw, yaw_enc.angle_rad);
  printf("[ALIGN] Alignment complete!\r\n");

  // Start FOC
  FOC_Start(&foc_pitch, pitch_enc.angle_rad);
  FOC_Start(&foc_roll, roll_enc.angle_rad);
  FOC_Start(&foc_yaw, yaw_enc.angle_rad);

  // Start TIM6 Control Loop
  HAL_TIM_Base_Start_IT(&htim6);

  // Start ADC Injected Interrupts (triggered by TIM1 TRGO)
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  HAL_ADCEx_InjectedStart_IT(&hadc2);

  printf("[SYSTEM] Initialization complete. Entering main loop.\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print = 0;
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - last_print >= 200) {
      last_print = HAL_GetTick();
      printf("[GIMBAL] P_Tar:%.1f P_Est:%.1f | R_Tar:%.1f R_Est:%.1f | Y_Ang:%.1f\r\n",
             pitch_target_deg, payload_pitch_deg,
             roll_target_deg, payload_roll_deg,
             yaw_enc.angle_deg);
      printf("[ISNS] P_Iq:%.2f | R_Iq:%.2f | Y_Iq:%.2f\r\n",
             foc_pitch.Iq_meas, foc_roll.Iq_meas, foc_yaw.Iq_meas);
    }
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
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    // 1. Pitch Current Loop (Rank 1 & 2)
    uint32_t raw_pitch_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_pitch_ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    
    float volt_pitch_a = (float)raw_pitch_ia * 3.3f / 4096.0f;
    float volt_pitch_b = (float)raw_pitch_ib * 3.3f / 4096.0f;
    
    float v_pitch_a = volt_pitch_a - pitch_offset_a;
    float v_pitch_b = volt_pitch_b - pitch_offset_b;
    
    float current_pitch_a = v_pitch_a / (GAIN_DRV * SHUNT_RES);
    float current_pitch_b = v_pitch_b / (GAIN_DRV * SHUNT_RES);
    
    foc_pitch.angle_elec += foc_pitch.velocity_mech * foc_pitch.pole_pairs * (1.0f / 20000.0f);
    if (foc_pitch.angle_elec >= TWO_PI) foc_pitch.angle_elec -= TWO_PI;
    else if (foc_pitch.angle_elec < 0.0f) foc_pitch.angle_elec += TWO_PI;
    
    FOC_UpdateCurrentLoop(&foc_pitch, current_pitch_a, current_pitch_b);

    // 2. Roll Current Loop (Rank 3 & 4)
    uint32_t raw_roll_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
    uint32_t raw_roll_ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_4);
    
    float volt_roll_a = (float)raw_roll_ia * 3.3f / 4096.0f;
    float volt_roll_b = (float)raw_roll_ib * 3.3f / 4096.0f;
    
    float v_roll_a = volt_roll_a - roll_offset_a;
    float v_roll_b = volt_roll_b - roll_offset_b;
    
    float current_roll_a = v_roll_a / (GAIN_DRV * SHUNT_RES);
    float current_roll_b = v_roll_b / (GAIN_DRV * SHUNT_RES);
    
    foc_roll.angle_elec += foc_roll.velocity_mech * foc_roll.pole_pairs * (1.0f / 20000.0f);
    if (foc_roll.angle_elec >= TWO_PI) foc_roll.angle_elec -= TWO_PI;
    else if (foc_roll.angle_elec < 0.0f) foc_roll.angle_elec += TWO_PI;
    
    FOC_UpdateCurrentLoop(&foc_roll, current_roll_a, current_roll_b);
  }
  else if (hadc->Instance == ADC2) {
    // Yaw Current Loop
    uint32_t raw_ia = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
    uint32_t raw_ib = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_2);
    
    float volt_a = (float)raw_ia * 3.3f / 4096.0f;
    float volt_b = (float)raw_ib * 3.3f / 4096.0f;
    
    float v_a = volt_a - yaw_offset_a;
    float v_b = volt_b - yaw_offset_b;
    
    float current_a = v_a / (GAIN_DRV * SHUNT_RES);
    float current_b = v_b / (GAIN_DRV * SHUNT_RES);
    
    // Extrapolate electrical angle
    foc_yaw.angle_elec += foc_yaw.velocity_mech * foc_yaw.pole_pairs * (1.0f / 20000.0f);
    if (foc_yaw.angle_elec >= TWO_PI) foc_yaw.angle_elec -= TWO_PI;
    else if (foc_yaw.angle_elec < 0.0f) foc_yaw.angle_elec += TWO_PI;
    
    FOC_UpdateCurrentLoop(&foc_yaw, current_a, current_b);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    // 1. Trigger sequential reading of Encoders via SPI1 DMA
    if (enc_state == ENC_STATE_IDLE) {
      enc_state = ENC_STATE_YAW;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // CS YAW Low
      HAL_SPI_TransmitReceive_DMA(&hspi1, enc_tx_buf, enc_rx_buf, 4);
    }

    // 2. Trigger sequential reading of IMUs via SPI3 DMA
    if (imu_state == IMU_STATE_IDLE) {
      imu_state = IMU_STATE_FRAME;
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // CS FRAME Low
      HAL_SPI_TransmitReceive_DMA(&hspi3, imu_tx_buf, imu_rx_buf, 15);
    }

    // 3. Attitude Estimation (Sensor Fusion via Complementary Filter)
    float dt = 0.001f;
    float alpha_filt = 0.98f;

    // Payload Attitude
    float payload_accel_pitch = atan2f(payload_imu.accel_x_g, sqrtf(payload_imu.accel_y_g * payload_imu.accel_y_g + payload_imu.accel_z_g * payload_imu.accel_z_g)) * RAD_TO_DEG;
    payload_pitch_deg = alpha_filt * (payload_pitch_deg + payload_imu.gyro_y_dps * dt) + (1.0f - alpha_filt) * payload_accel_pitch;

    float payload_accel_roll = atan2f(payload_imu.accel_y_g, payload_imu.accel_z_g) * RAD_TO_DEG;
    payload_roll_deg = alpha_filt * (payload_roll_deg + payload_imu.gyro_x_dps * dt) + (1.0f - alpha_filt) * payload_accel_roll;

    // Frame Attitude
    float frame_accel_pitch = atan2f(frame_imu.accel_x_g, sqrtf(frame_imu.accel_y_g * frame_imu.accel_y_g + frame_imu.accel_z_g * frame_imu.accel_z_g)) * RAD_TO_DEG;
    frame_pitch_deg = alpha_filt * (frame_pitch_deg + frame_imu.gyro_y_dps * dt) + (1.0f - alpha_filt) * frame_accel_pitch;

    float frame_accel_roll = atan2f(frame_imu.accel_y_g, frame_imu.accel_z_g) * RAD_TO_DEG;
    frame_roll_deg = alpha_filt * (frame_roll_deg + frame_imu.gyro_x_dps * dt) + (1.0f - alpha_filt) * frame_accel_roll;

    // 4. Cascade PID Controller Updates
    // Pitch (Closed-loop current FOC)
    float pitch_pos_err = pitch_target_deg - payload_pitch_deg;
    float pitch_target_vel = PID_Update(&pid_pos_pitch, pitch_pos_err, dt);
    FOC_RunVelocity(&foc_pitch, pitch_enc.angle_rad, pitch_target_vel);

    // Roll (Closed-loop current FOC)
    float roll_pos_err = roll_target_deg - payload_roll_deg;
    float roll_target_vel = PID_Update(&pid_pos_roll, roll_pos_err, dt);
    FOC_RunVelocity(&foc_roll, roll_enc.angle_rad, roll_target_vel);

    // Yaw (Voltage mode FOC fallback)
    float yaw_pos_err = yaw_target_deg - yaw_enc.angle_deg;
    float yaw_target_vel = PID_Update(&pid_pos_yaw, yaw_pos_err, dt);
    FOC_RunVelocity(&foc_yaw, yaw_enc.angle_rad, yaw_target_vel);
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    // Encoder SPI DMA sequence
    switch (enc_state) {
      case ENC_STATE_YAW:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // CS YAW High
        {
          uint16_t rx_val = ((uint16_t)enc_rx_buf[2] << 8) | enc_rx_buf[3];
          if (AS5048A_CheckParity(rx_val) && !(rx_val & AS5048A_EF_BIT)) {
            yaw_enc.raw_angle = rx_val & AS5048A_DATA_MASK;
            yaw_enc.angle_deg = (float)yaw_enc.raw_angle * (360.0f / 16384.0f);
            yaw_enc.angle_rad = (float)yaw_enc.raw_angle * (6.28318530718f / 16384.0f);
          }
        }
        enc_state = ENC_STATE_PITCH;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // CS PITCH Low
        HAL_SPI_TransmitReceive_DMA(&hspi1, enc_tx_buf, enc_rx_buf, 4);
        break;

      case ENC_STATE_PITCH:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // CS PITCH High
        {
          uint16_t rx_val = ((uint16_t)enc_rx_buf[2] << 8) | enc_rx_buf[3];
          if (AS5048A_CheckParity(rx_val) && !(rx_val & AS5048A_EF_BIT)) {
            pitch_enc.raw_angle = rx_val & AS5048A_DATA_MASK;
            pitch_enc.angle_deg = (float)pitch_enc.raw_angle * (360.0f / 16384.0f);
            pitch_enc.angle_rad = (float)pitch_enc.raw_angle * (6.28318530718f / 16384.0f);
          }
        }
        enc_state = ENC_STATE_ROLL;
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // CS ROLL Low
        HAL_SPI_TransmitReceive_DMA(&hspi1, enc_tx_buf, enc_rx_buf, 4);
        break;

      case ENC_STATE_ROLL:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // CS ROLL High
        {
          uint16_t rx_val = ((uint16_t)enc_rx_buf[2] << 8) | enc_rx_buf[3];
          if (AS5048A_CheckParity(rx_val) && !(rx_val & AS5048A_EF_BIT)) {
            roll_enc.raw_angle = rx_val & AS5048A_DATA_MASK;
            roll_enc.angle_deg = (float)roll_enc.raw_angle * (360.0f / 16384.0f);
            roll_enc.angle_rad = (float)roll_enc.raw_angle * (6.28318530718f / 16384.0f);
          }
        }
        enc_state = ENC_STATE_IDLE;
        break;

      default:
        enc_state = ENC_STATE_IDLE;
        break;
    }
  }
  else if (hspi->Instance == SPI3) {
    // IMU SPI DMA sequence
    switch (imu_state) {
      case IMU_STATE_FRAME:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // CS FRAME High
        {
          frame_imu.raw_temp    = (int16_t)((imu_rx_buf[1]  << 8) | imu_rx_buf[2]);
          frame_imu.raw_accel_x = (int16_t)((imu_rx_buf[3]  << 8) | imu_rx_buf[4]);
          frame_imu.raw_accel_y = (int16_t)((imu_rx_buf[5]  << 8) | imu_rx_buf[6]);
          frame_imu.raw_accel_z = (int16_t)((imu_rx_buf[7]  << 8) | imu_rx_buf[8]);
          frame_imu.raw_gyro_x  = (int16_t)((imu_rx_buf[9]  << 8) | imu_rx_buf[10]);
          frame_imu.raw_gyro_y  = (int16_t)((imu_rx_buf[11] << 8) | imu_rx_buf[12]);
          frame_imu.raw_gyro_z  = (int16_t)((imu_rx_buf[13] << 8) | imu_rx_buf[14]);
          
          float gs = frame_imu.gyro_sensitivity;
          float as = frame_imu.accel_sensitivity;
          frame_imu.gyro_x_dps = (float)frame_imu.raw_gyro_x / gs;
          frame_imu.gyro_y_dps = (float)frame_imu.raw_gyro_y / gs;
          frame_imu.gyro_z_dps = (float)frame_imu.raw_gyro_z / gs;
          frame_imu.accel_x_g  = (float)frame_imu.raw_accel_x / as;
          frame_imu.accel_y_g  = (float)frame_imu.raw_accel_y / as;
          frame_imu.accel_z_g  = (float)frame_imu.raw_accel_z / as;
        }
        imu_state = IMU_STATE_PAYLOAD;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); // CS PAYLOAD Low
        HAL_SPI_TransmitReceive_DMA(&hspi3, imu_tx_buf, imu_rx_buf, 15);
        break;

      case IMU_STATE_PAYLOAD:
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); // CS PAYLOAD High
        {
          payload_imu.raw_temp    = (int16_t)((imu_rx_buf[1]  << 8) | imu_rx_buf[2]);
          payload_imu.raw_accel_x = (int16_t)((imu_rx_buf[3]  << 8) | imu_rx_buf[4]);
          payload_imu.raw_accel_y = (int16_t)((imu_rx_buf[5]  << 8) | imu_rx_buf[6]);
          payload_imu.raw_accel_z = (int16_t)((imu_rx_buf[7]  << 8) | imu_rx_buf[8]);
          payload_imu.raw_gyro_x  = (int16_t)((imu_rx_buf[9]  << 8) | imu_rx_buf[10]);
          payload_imu.raw_gyro_y  = (int16_t)((imu_rx_buf[11] << 8) | imu_rx_buf[12]);
          payload_imu.raw_gyro_z  = (int16_t)((imu_rx_buf[13] << 8) | imu_rx_buf[14]);

          float gs = payload_imu.gyro_sensitivity;
          float as = payload_imu.accel_sensitivity;
          payload_imu.gyro_x_dps = (float)payload_imu.raw_gyro_x / gs;
          payload_imu.gyro_y_dps = (float)payload_imu.raw_gyro_y / gs;
          payload_imu.gyro_z_dps = (float)payload_imu.raw_gyro_z / gs;
          payload_imu.accel_x_g  = (float)payload_imu.raw_accel_x / as;
          payload_imu.accel_y_g  = (float)payload_imu.raw_accel_y / as;
          payload_imu.accel_z_g  = (float)payload_imu.raw_accel_z / as;
        }
        imu_state = IMU_STATE_IDLE;
        break;

      default:
        imu_state = IMU_STATE_IDLE;
        break;
    }
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
