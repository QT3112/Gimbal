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
FOC_PID_t pid_roll; /* PID vòng ngoài trục Roll */

/* --- FOC Controller --- */
FOC_Handle_t foc;      /* FOC trục Pitch (TIM2) */
FOC_Handle_t foc_roll; /* FOC trục Roll  (TIM3) */

/* --- Debug Variables (shared between Interrupt & Main Loop) --- */
volatile float debug_pitch_deg = 0.0f;
volatile float debug_target_vel = 0.0f;
volatile float debug_cam_rate = 0.0f;
volatile float debug_vq_ref = 0.0f;
volatile float debug_roll_deg = 0.0f;
volatile float debug_roll_target_vel = 0.0f;
volatile float debug_roll_cam_rate = 0.0f;
volatile float debug_roll_vq_ref = 0.0f;
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

  /* PC6: Enable driver (nếu có gate driver) */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);

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
           0.002f); /* Ts = 2ms (500Hz) */

  // LPF cho tốc độ vòng trong
  FOC_SetLPF_Vel(&foc, 0.98f);
  // Cấu hình PID vận tốc (vòng trong)
  FOC_SetPID_Vel(&foc, 0.1f, 0.01f, 0.0f, -foc.voltage_limit,
                 foc.voltage_limit);
  // Cấu hình PID góc (vòng ngoài) cho Pitch
  pid_pitch.Kp = 2.0f;
  pid_pitch.Ki = 0.0f;
  pid_pitch.Kd = 0.0f;
  pid_pitch.output_min = -10.0f; // max target_vel [rad/s]
  pid_pitch.output_max = 10.0f;
  FOC_PID_Reset(&pid_pitch);

  // --- KHỞI TẠO FOC ROLL (TIM3) ---
  FOC_Init(&foc_roll, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           4249.0f, /* PWM Period - giống TIM2 */
           7, /* Số cặp cực motor Roll (chỉnh lại nếu motor khác) */
           0.5f,    /* Giới hạn điện áp */
           0.002f); /* Ts = 2ms (500Hz) */

  FOC_SetLPF_Vel(&foc_roll, 0.98f);
  FOC_SetPID_Vel(&foc_roll, 0.1f, 0.01f, 0.0f, -foc_roll.voltage_limit,
                 foc_roll.voltage_limit);

  // Cấu hình PID góc (vòng ngoài) cho Roll
  pid_roll.Kp = 2.0f;
  pid_roll.Ki = 0.0f;
  pid_roll.Kd = 0.0f;
  pid_roll.output_min = -10.0f;
  pid_roll.output_max = 10.0f;
  FOC_PID_Reset(&pid_roll);

  // *** MẸO 2: Cho bộ lọc Mahony "chạy nháp" TRƯỚC khi bật motor ***
  // Motor OFF hoàn toàn trong giai đoạn này (FOC_Start chưa được gọi).
  // 500 vòng x 10ms = 5 giây để Mahony hội tụ từ quaternion (1,0,0,0)
  // về góc thực tế chính xác của thiết bị.
  printf("Dang khoi tao bo loc Mahony (5s). Vui long giu yen Gimbal...\r\n");
  for (int i = 0; i < 500; i++) {
    if (MPU6050_ReadAll(&imu_frame) == MPU6050_OK &&
        MPU6050_ReadAll(&imu_payload) == MPU6050_OK) {

      Attitude_Update(
          &att, imu_frame.gyro_x * DEG_TO_RAD, imu_frame.gyro_y * DEG_TO_RAD,
          imu_frame.gyro_z * DEG_TO_RAD, imu_frame.accel_x, imu_frame.accel_y,
          imu_frame.accel_z, imu_payload.gyro_x * DEG_TO_RAD,
          imu_payload.gyro_y * DEG_TO_RAD, imu_payload.gyro_z * DEG_TO_RAD,
          imu_payload.accel_x, imu_payload.accel_y, imu_payload.accel_z, 0.01f);
    }
    // In tiến trình mỗi giây để người dùng biết hệ thống đang chạy
    if (i % 100 == 0) {
      printf("  Mahony warm-up: %d%%\r\n", i / 5);
    }
    HAL_Delay(10);
  }
  printf("Bo loc Mahony hoi tu! Pitch=%.2f Roll=%.2f\r\n",
         Attitude_GetPayloadPitch(&att) * RAD_TO_DEG,
         Attitude_GetPayloadRoll(&att) * RAD_TO_DEG);

  // =========================================================================
  // BƯỚC 4: HOMING — Định vị Zero điện tử
  //
  // Mục đích: Xác định "góc cơ học zero" của motor khi rotor được nhắt
  //           về trục D của stator bằng cách bơm Vd cố định.
  //
  // Lưu ý quan trọng:
  //   - FOC_CalibrateAngle() nhận GÓC CƠ HỌC (rad) — KHÔNG phải góc điện.
  //     Nó tự tính: angle_offset = current_angle_mech * pole_pairs
  //   - Phải tiếp tục gọi Attitude_Update() trong quá trình Alignment
  //     để q_error luôn được tính mới nhất khi đọc offset.
  // =========================================================================
  FOC_Start(&foc);
  FOC_Start(&foc_roll);
  printf("[HOMING] Bat dau can chinh Rotor (1.5s). Khong cham vao Gimbal!\r\n");

  // --- Giai đoạn 1: Khóa Rotor (150 × 10ms = 1.5 giây) ---
  // Motor bơm Vd cố định để kéo rotor về hướng từ trường stator tại theta=0.
  // Song song đó, tiếp tục đọc IMU để Mahony / q_error luôn cập nhật.
  for (int i = 0; i < 150; i++) {
    FOC_AlignD(&foc, foc.voltage_limit);
    FOC_AlignD(&foc_roll, foc_roll.voltage_limit);

    // Đọc IMU để q_error không bị đóng băng trong lúc Homing
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

  // --- Giai đoạn 2: Đọc điểm zero và lưu offset ---
  // Rotor đã ổn định tại theta_elec = 0 của stator.
  // Góc cơ học tương đối (relative) lúc này chính là điểm zero FOC.
  float start_pitch = Attitude_GetRelativePitch(&att); /* [rad] */
  float start_roll = Attitude_GetRelativeRoll(&att);   /* [rad] */

  // Truyền GÓC CƠ HỌC (không nhân pole_pairs) vào FOC_CalibrateAngle.
  // Hàm này tự tính: angle_offset = current_angle_mech * pole_pairs
  FOC_CalibrateAngle(&foc, start_pitch);
  FOC_CalibrateAngle(&foc_roll, start_roll);

  printf("[HOMING] Xong! RelPitch=%.2f deg | RelRoll=%.2f deg\r\n"
         "         ElecOffset_Pitch=%.3f rad | ElecOffset_Roll=%.3f rad\r\n",
         start_pitch * RAD_TO_DEG, start_roll * RAD_TO_DEG, foc.angle_offset,
         foc_roll.angle_offset);

  // =========================================================================
  // BƯỚC 5: TEST OPEN-LOOP (Xác nhận SVPWM hoạt động đúng)
  //
  // Trước khi bật PID kín, chạy thử 0.5 giây Open-loop để kiểm tra:
  //   - Motor có quay mượt không? (không có tiếng gằn/rung)
  //   - Địu dây pha A-B-C có đúng không?
  //   - Nếu motor không quá nóng sau 0.5s thì hướng quay đang đúng.
  //
  // Lưu ý:
  //   - Vq dương = quay theo chiều tăng góc (thuận)
  //   - Nếu Gimbal bị kéo ra khỏi vị trí ngang trong test này là bình thường.
  //   - Bỏ test này (comment out) sau khi đã xác nhận 1 lần thành công.
  // =========================================================================
  printf("[SVPWM TEST] Chay open-loop 0.5s...\r\n");
  for (int i = 0; i < 50; i++) {
    // Cập nhật góc điện từ q_error mới nhất
    if (MPU6050_ReadAll(&imu_frame) == MPU6050_OK &&
        MPU6050_ReadAll(&imu_payload) == MPU6050_OK) {
      Attitude_Update(
          &att, imu_frame.gyro_x * DEG_TO_RAD, imu_frame.gyro_y * DEG_TO_RAD,
          imu_frame.gyro_z * DEG_TO_RAD, imu_frame.accel_x, imu_frame.accel_y,
          imu_frame.accel_z, imu_payload.gyro_x * DEG_TO_RAD,
          imu_payload.gyro_y * DEG_TO_RAD, imu_payload.gyro_z * DEG_TO_RAD,
          imu_payload.accel_x, imu_payload.accel_y, imu_payload.accel_z, 0.01f);
    }

    // Bơm Vq nhỏ (+0.1V) qua SVPWM với góc điện từ q_error
    foc.angle_elec =
        Attitude_GetElecAnglePitchRel(&att, foc.pole_pairs, foc.angle_offset);
    foc_roll.angle_elec = Attitude_GetElecAngleRollRel(
        &att, foc_roll.pole_pairs, foc_roll.angle_offset);
    FOC_SetVoltage(&foc, 0.0f, 0.1f); /* Vd=0, Vq=+0.1V */
    FOC_SetVoltage(&foc_roll, 0.0f, 0.1f);
    FOC_Update(&foc);
    FOC_Update(&foc_roll);
    HAL_Delay(10);
  }

  // Trả motor về trạng thái nghỉ (không bơm điện áp)
  FOC_SetVoltage(&foc, 0.0f, 0.0f);
  FOC_SetVoltage(&foc_roll, 0.0f, 0.0f);
  FOC_Update(&foc);
  FOC_Update(&foc_roll);
  printf("[SVPWM TEST] Hoan thanh. Bat dau dieu khien PID...\r\n");

  // Kích hoạt ngắt TIM6 để chạy vòng lặp PID 500Hz
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // In dữ liệu qua UART trong vòng lặp chính (không in trong ngắt)
    printf("[P] Pitch:%.2f TarVel:%.2f Vel:%.2f Vq:%.2f | [R] Roll:%.2f "
           "TarVel:%.2f Vel:%.2f Vq:%.2f\r\n",
           debug_pitch_deg, debug_target_vel, debug_cam_rate, debug_vq_ref,
           debug_roll_deg, debug_roll_target_vel, debug_roll_cam_rate,
           debug_roll_vq_ref);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(100); // In chậm rãi 10Hz để dễ xem
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
// Khai báo extern để trình biên dịch không báo lỗi khi chưa config CubeMX.
// (Sẽ được tự động định nghĩa trong tim.c khi bạn gen code từ CubeMX)
extern TIM_HandleTypeDef htim6;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    if (imu_frame_ready && imu_payload_ready) {
      // ================================================================
      // 1. ĐỌC DỮ LIỆU TỪ 2 IMU — Cả 2 phải thành công mới tiếp tục
      // ================================================================
      if (MPU6050_ReadAll(&imu_frame) == MPU6050_OK &&
          MPU6050_ReadAll(&imu_payload) == MPU6050_OK) {

        const float ax1 = imu_frame.accel_x, ay1 = imu_frame.accel_y,
                    az1 = imu_frame.accel_z;
        const float gx1 = imu_frame.gyro_x * DEG_TO_RAD;
        const float gy1 = imu_frame.gyro_y * DEG_TO_RAD;
        const float gz1 = imu_frame.gyro_z * DEG_TO_RAD;

        const float ax2 = imu_payload.accel_x, ay2 = imu_payload.accel_y,
                    az2 = imu_payload.accel_z;
        const float gx2 = imu_payload.gyro_x * DEG_TO_RAD;
        const float gy2 = imu_payload.gyro_y * DEG_TO_RAD;
        const float gz2 = imu_payload.gyro_z * DEG_TO_RAD;

        const float dt = 0.002f; /* Ts = 2ms → 500Hz */

        // ================================================================
        // 2. CẬP NHẬT ATTITUDE ESTIMATOR (Bước 2 & 3)
        //    → Chạy 2 bộ Mahony song song
        //    → Yaw Lock: kéo Yaw payload về Yaw frame (chống trôi)
        //    → Tính q_error = conj(q_frame) ⊗ q_payload
        //    → Tách relative_pitch, relative_roll (góc cơ học motor)
        // ================================================================
        Attitude_Update(&att, gx1, gy1, gz1, ax1, ay1, az1, gx2, gy2, gz2, ax2,
                        ay2, az2, dt);

        /* Góc TUYỆT ĐỐI của Camera — dùng cho outer Angle PID.
         * Mục tiêu: giữ camera nằm ngang (pitch_abs = 0, roll_abs = 0). */
        const float pitch_abs = Attitude_GetPayloadPitch(&att);
        const float roll_abs = Attitude_GetPayloadRoll(&att);

        /* Hệ số Feedforward.
         * K_ff = 1.0: Motor cộng TOÀN BỘ tốc độ của tay cầm vào target_vel.
         * Giảm về 0.5~0.8 nếu gimbal rung do feedforward quá mạnh. */
        const float K_ff = 1.0f;

        // ================================================================
        // BƯỚC 6A. TRỤC PITCH — Cascade PID + Feedforward + FOC SVPWM
        //
        //  Outer (Angle):  pitch_abs → [P_angle] → target_vel_p
        //  Feedforward:    frame_pitch_rate × K_ff  (+, bù trước)
        //  Inner (Rate):   (target_vel_p + ff) − cam_rate → [PI_vel] → Vq
        //  FOC SVPWM:      pitch_rel × pole_pairs − offset → angle_elec → PWM
        // ================================================================

        /* Vòng ngoài — Angle PID */
        const float pitch_err = 0.0f - pitch_abs;
        const float target_vel_p = FOC_PID_Update(&pid_pitch, pitch_err, dt);

        /* Feedforward: CỘNG tốc độ khung vào setpoint (không trừ).
         * Khi tay cầm nghiêng → frame_pitch_rate > 0 → motor bơm lực
         * cùng chiều TRƯỚC KHI camera kịp bị kéo đi. */
        const float frame_pitch_rate = Attitude_GetFramePitchRate(&att);
        const float ff_pitch = K_ff * frame_pitch_rate;

        /* Vòng trong — Rate PID */
        const float cam_pitch_rate = Attitude_GetPayloadPitchRate(&att);
        const float vel_err_pitch = (target_vel_p + ff_pitch) - cam_pitch_rate;
        const float Vq_pitch = FOC_PID_Update(&foc.pid_vel, vel_err_pitch, dt);

        /* FOC SVPWM — dùng góc TƯƠNG ĐỐI (q_error) thay encoder */
        foc.angle_elec = Attitude_GetElecAnglePitchRel(&att, foc.pole_pairs,
                                                       foc.angle_offset);
        FOC_SetVoltage(&foc, 0.0f, Vq_pitch);
        FOC_Update(&foc);

        // ================================================================
        // BƯỚC 6B. TRỤC ROLL — Cascade PID + Feedforward + FOC SVPWM
        // ================================================================

        /* Vòng ngoài — Angle PID */
        const float roll_err = 0.0f - roll_abs;
        const float target_vel_r = FOC_PID_Update(&pid_roll, roll_err, dt);

        /* Feedforward */
        const float frame_roll_rate = Attitude_GetFrameRollRate(&att);
        const float ff_roll = K_ff * frame_roll_rate;

        /* Vòng trong — Rate PID */
        const float cam_roll_rate = Attitude_GetPayloadRollRate(&att);
        const float vel_err_roll = (target_vel_r + ff_roll) - cam_roll_rate;
        const float Vq_roll =
            FOC_PID_Update(&foc_roll.pid_vel, vel_err_roll, dt);

        /* FOC SVPWM — Roll */
        foc_roll.angle_elec = Attitude_GetElecAngleRollRel(
            &att, foc_roll.pole_pairs, foc_roll.angle_offset);
        FOC_SetVoltage(&foc_roll, 0.0f, Vq_roll);
        FOC_Update(&foc_roll);

        // ================================================================
        // 3. LƯU BIẾN DEBUG
        //    while(1) đọc và printf ra UART mỗi 100ms — KHÔNG printf ở đây!
        // ================================================================
        debug_pitch_deg = pitch_abs * RAD_TO_DEG;
        debug_target_vel = target_vel_p;
        debug_cam_rate = cam_pitch_rate;
        debug_vq_ref = Vq_pitch;

        debug_roll_deg = roll_abs * RAD_TO_DEG;
        debug_roll_target_vel = target_vel_r;
        debug_roll_cam_rate = cam_roll_rate;
        debug_roll_vq_ref = Vq_roll;
      }
    }
  }
}
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
