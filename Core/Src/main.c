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
#include "attitude.h"
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
#define DEG_TO_RAD (PI / 180.0f)
#define RAD_TO_DEG (180.0f / PI)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* --- MPU6050 --- */
MPU6050_Handle_t imu_frame;
MPU6050_Handle_t imu_payload;
uint8_t imu_frame_ready = 0;
uint8_t imu_payload_ready = 0;

/* --- Attitude Estimator --- */
Attitude_Handle_t att;

/* --- Gimbal Controllers --- */
FOC_PID_t pid_pitch;
FOC_PID_t pid_vel;

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

  // --- BƯỚC 1: KHỞI TẠO MPU6050 (FRAME & PAYLOAD) ---
  printf("Dang khoi tao IMU Frame (0x68)...\r\n");
  if (MPU6050_Init(&imu_frame, &hi2c3, MPU6050_ADDR_LOW) == MPU6050_OK) {
    imu_frame_ready = 1;
    printf("[IMU Frame] Khoi tao thanh cong! Dang calibrate...\r\n");
    MPU6050_CalibrateGyro(&imu_frame, 500);
    printf("[IMU Frame] Calibrate xong! Offset X:%.2f, Y:%.2f, Z:%.2f\r\n",
           imu_frame.gyro_offset_x, imu_frame.gyro_offset_y,
           imu_frame.gyro_offset_z);
  } else {
    printf("[IMU Frame] Loi khoi tao!\r\n");
  }

  printf("Dang khoi tao IMU Payload (0x69)...\r\n");
  if (MPU6050_Init(&imu_payload, &hi2c3, MPU6050_ADDR_HIGH) == MPU6050_OK) {
    imu_payload_ready = 1;
    printf("[IMU Payload] Khoi tao thanh cong! Dang calibrate...\r\n");
    MPU6050_CalibrateGyro(&imu_payload, 500);
    printf("[IMU Payload] Calibrate xong! Offset X:%.2f, Y:%.2f, Z:%.2f\r\n",
           imu_payload.gyro_offset_x, imu_payload.gyro_offset_y,
           imu_payload.gyro_offset_z);
  } else {
    printf("[IMU Payload] Loi khoi tao!\r\n");
  }

  // Khởi tạo Attitude Estimator (Mahony filter cho 2 IMU)
  Attitude_Init(&att, 2.0f, 0.005f);

  // --- BƯỚC 2: KHỞI TẠO & CĂN CHỈNH FOC ---
  FOC_Init(&foc, &htim2, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           4249.0f, /* PWM Period */
           7,       /* Số cặp cực (ví dụ 12N14P -> 7) */
           0.5f, /* Giới hạn điện áp [V] - Vd dùng để căn chỉnh */
           0.01f); /* Ts = 10ms */

  // LPF cho tốc độ vòng trong
  FOC_SetLPF_Vel(&foc, 0.9f);
  // Cấu hình PID vận tốc (vòng trong)
  FOC_SetPID_Vel(&foc, 0.1f, 0.01f, 0.0f, -foc.voltage_limit,
                 foc.voltage_limit);
  // Cấu hình PID góc (vòng ngoài)
  pid_pitch.Kp = 2.0f;
  pid_pitch.Ki = 0.0f;
  pid_pitch.Kd = 0.0f;
  pid_pitch.output_min = -10.0f; // max target_vel [rad/s]
  pid_pitch.output_max = 10.0f;
  FOC_PID_Reset(&pid_pitch);

  FOC_Start(&foc);
  printf("Dang can chinh Rotor (Alignment). Vui long khong cham vao "
         "Gimbal...\r\n");

  // Khóa Rotor bằng Vd và cập nhật bộ lọc Mahony để tìm góc Offset
  for (int i = 0; i < 100; i++) {
    FOC_AlignD(&foc, foc.voltage_limit); // Áp điện áp Vd để khóa rotor

    // Đồng thời cập nhật IMU để bộ lọc Mahony hội tụ góc
    if (MPU6050_ReadAll(&imu_frame) == MPU6050_OK &&
        MPU6050_ReadAll(&imu_payload) == MPU6050_OK) {

      Attitude_Update(
          &att, imu_frame.gyro_x * DEG_TO_RAD, imu_frame.gyro_y * DEG_TO_RAD,
          imu_frame.gyro_z * DEG_TO_RAD, imu_frame.accel_x, imu_frame.accel_y,
          imu_frame.accel_z, imu_payload.gyro_x * DEG_TO_RAD,
          imu_payload.gyro_y * DEG_TO_RAD, imu_payload.gyro_z * DEG_TO_RAD,
          imu_payload.accel_x, imu_payload.accel_y, imu_payload.accel_z, 0.01f);
    }
    HAL_Delay(10);
  }

  // Lưu lại góc offset (điểm zero) bằng chính góc Pitch của Camera lúc đang
  // khóa Rotor
  float start_pitch = Attitude_GetPayloadPitch(&att);
  FOC_CalibrateAngle(&foc, start_pitch);

  printf(
      "Can chinh xong! Pitch ban dau: %.2f deg, Offset goc dien: %.2f rad\r\n",
      start_pitch * RAD_TO_DEG, foc.angle_offset);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    if (imu_frame_ready && imu_payload_ready) {
      // 1. Đọc dữ liệu từ 2 IMU
      if (MPU6050_ReadAll(&imu_frame) == MPU6050_OK) {
        // Đọc IMU Frame
        float ax1 = imu_frame.accel_x;
        float ay1 = imu_frame.accel_y;
        float az1 = imu_frame.accel_z;
        float gx1 = imu_frame.gyro_x * DEG_TO_RAD;
        float gy1 = imu_frame.gyro_y * DEG_TO_RAD;
        float gz1 = imu_frame.gyro_z * DEG_TO_RAD;

        // Đọc IMU Payload
        MPU6050_ReadAll(&imu_payload);
        float ax2 = imu_payload.accel_x;
        float ay2 = imu_payload.accel_y;
        float az2 = imu_payload.accel_z;
        float gx2 = imu_payload.gyro_x * DEG_TO_RAD;
        float gy2 = imu_payload.gyro_y * DEG_TO_RAD;
        float gz2 = imu_payload.gyro_z * DEG_TO_RAD;

        // printf("ax1: %.2f | ay1: %.2f | az1: %.2f | gx1: %.2f | gy1: %.2f | "
        //        "gz1: %.2f | ax2: %.2f | ay2: %.2f | az2: %.2f | gx2: %.2f | "
        //        "gy2: %.2f | gz2: %.2f\r\n",
        //        ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2);

        // 2. Cập nhật Attitude Estimator
        // Hàm này sẽ kết hợp dữ liệu 2 IMU và tính toán attitude
        Attitude_Update(&att, gx1, gy1, gz1, ax1, ay1, az1, gx2, gy2, gz2, ax2,
                        ay2, az2, 0.01f);

        // 3. Lấy kết quả attitude
        // pitch: roll: yaw theo radian
        float pitch = Attitude_GetPayloadPitch(&att);
        float roll = Attitude_GetPayloadRoll(&att);
        float yaw = Attitude_GetPayloadYaw(&att);

        // Chuyển sang độ để debug
        float pitch_deg = pitch * RAD_TO_DEG;
        float roll_deg = roll * RAD_TO_DEG;
        float yaw_deg = yaw * RAD_TO_DEG;

        // 4. Cascade PID Control (Điều khiển Vị trí + Vận tốc)
        float target_pitch_angle =
            0.0f; // Góc mong muốn của Camera (0 độ = cân bằng)

        // --- Vòng ngoài (Outer Loop): Trình điều khiển Góc ---
        // Sai số góc = Góc mong muốn - Góc thực tế (Lưu ý: phải dùng Radian)
        float pitch_error_rad = target_pitch_angle - pitch;

        // Đầu ra của vòng Góc là Tốc độ quay mong muốn (target_velocity)
        float target_vel_rad_s =
            FOC_PID_Update(&pid_pitch, pitch_error_rad, 0.01f);

        // --- Vòng trong (Inner Loop): Trình điều khiển Vận tốc + Feedforward
        // --- Lấy vận tốc hiện tại của camera (Feedback)
        float cam_rate = Attitude_GetPayloadPitchRate(&att);

        // Lấy vận tốc hiện tại của khung (Feedforward)
        float frame_rate = Attitude_GetFramePitchRate(&att);

        // Sai số vận tốc = Vận tốc mục tiêu - Vận tốc thực tế
        float vel_error = target_vel_rad_s - cam_rate;

        // Thêm Feedforward: Tùy chiều gắn cảm biến mà ta cộng hay trừ
        // frame_rate. Tạm thời trừ trực tiếp frame_rate để bù nhiễu.
        float K_ff = 1.0f; // Hệ số feedforward (từ 0.0 -> 1.0)
        vel_error -= (K_ff * frame_rate);

        // --- Cập nhật giá trị vào FOC ---
        // Tính Điện áp trục Q (Lực kéo) từ vòng PID vận tốc của FOC
        float Vq_ref = FOC_PID_Update(&foc.pid_vel, vel_error, 0.01f);

        // Lấy góc điện từ bộ lọc Attitude (Thay thế hoàn toàn Encoder)
        foc.angle_elec =
            Attitude_GetElecAngle(&att, foc.pole_pairs, foc.angle_offset);

        // Bơm lệnh điện áp (Vd = 0, Vq = Vq_ref)
        FOC_SetVoltage(&foc, 0.0f, Vq_ref);

        // Tính toán SVPWM và xuất ra các kênh Timer
        FOC_Update(&foc);

        // 5. Kiểm tra dữ liệu qua UART (Tần suất thấp để không block vòng lặp)
        static int print_cnt = 0;
        if (++print_cnt >= 10) { // Cứ 100ms in 1 lần
          printf("Pitch: %.2f | TarVel: %.2f | MeasVel: %.2f | Vq: %.2f\r\n",
                 pitch_deg, target_vel_rad_s, cam_rate, Vq_ref);
          print_cnt = 0;
        }
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
