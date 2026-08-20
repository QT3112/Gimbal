/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5048a.h"
#include "foc_v1.h"
#include "icm42688.h"
#include "imu_filter.h"
#include "sbus_protocol.h"
#include "mapping_sbus_channel.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* === Giới hạn góc mềm trục motor === */
#define PITCH_MIN_DEG             15.0f
#define PITCH_MAX_DEG             280.0f
#define ROLL_MIN_DEG              90.0f
#define ROLL_MAX_DEG              325.0f   /* Mở rộng để cho phép vị trí home 310° */
#define YAW_MIN_DEG               65.0f
#define YAW_MAX_DEG               255.0f

/*=== Vị trí Home (khởi động tự động quay về đây) ===*/
#define HOME_ROLL_DEG             310.0f
#define HOME_PITCH_DEG            215.0f
#define HOME_YAW_DEG              165.0f
#define HOME_TOL_DEG              2.0f    /* Sai số cho phép để coi là đã đến home (deg) */
#define HOME_TIMEOUT_MS           12000   /* Tối đa 12 giây để homing */

/*=== Góc mục tiêu cố định motor (dự phòng) ===*/
#define PITCH_SETPOINT_DEG        220.0f
#define ROLL_SETPOINT_DEG         185.0f
#define YAW_SETPOINT_DEG          180.0f
/*=== Góc mục tiêu cố định IMU ===*/
#define DEMO3AX_TARGET_ROLL_DEG   0.4f
#define DEMO3AX_TARGET_PITCH_DEG -1.6f
#define DEMO3AX_TARGET_YAW_DEG    19.0f
/*=== Hệ số đảo chiều ===*/
#define DEMO3AX_SIGN_ROLL         (-1.0f)
#define DEMO3AX_SIGN_PITCH        (1.0f)
#define DEMO3AX_SIGN_YAW          (1.0f)

/*=== Cấu hình phần cứng mạch dòng ===*/
#define GAIN_DRV                  10.0f
#define SHUNT_RES                 0.005f
#define VOLTAGE_LIMIT             1.5f 
#define PWM_PERIOD                4249.0f
#define MOTOR_POLE_PAIRS          7


#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f
#define DEG2RAD(d) ((d) * DEG_TO_RAD)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

AS5048A_Handle_t pitch_enc;
AS5048A_Handle_t roll_enc;
AS5048A_Handle_t yaw_enc;
FOC_Handle_t foc_pitch;
FOC_Handle_t foc_roll;
FOC_Handle_t foc_yaw;

ICM42688_Handle_t imu_payload;
MahonyFilter_t mahony_imu;
uint8_t icm_init_ok = 0;
Quaternion_t q_target_3d = {1.0f, 0.0f, 0.0f, 0.0f};
float e_rot[3] = {0.0f, 0.0f, 0.0f};

Quaternion_t q_demo3_target; /* Hướng mục tiêu: chốt khi vào mode, giữ cố định sau đó */

/* PID vòng ngoài: error góc (rad) → velo setpoint (rad/s) cho từng trục */
PID_Handle_t pid_3ax_roll_pos;  /* e_rot[1] (Y) → vel_roll  */
PID_Handle_t pid_3ax_pitch_pos; /* e_rot[0] (X) → vel_pitch */
PID_Handle_t pid_3ax_yaw_pos;   /* e_rot[2] (Z) → vel_yaw   */

SBUS_Handle_t sbus_rx;
SBUS_Mapping_Handle_t sbus_map;   /* Mapping SBUS raw → target angle gimbal */

volatile float target_pitch_angle = 0.0f;
volatile float target_roll_angle  = 0.0f;
volatile float target_yaw_angle   = 0.0f;

/* === Trạng thái máy trạng thái của gimbal === */
typedef enum {
  GIMBAL_STATE_HOMING = 0,  /* Đang tự động quay về vị trí home */
  GIMBAL_STATE_SBUS,        /* Đang nhận lệnh điều khiển từ SBUS */
} GimbalState_t;

GimbalState_t g_gimbal_state = GIMBAL_STATE_HOMING;

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
  MX_DMA_Init();
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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*=== Khởi tạo encoder ===*/
  AS5048A_Init(&pitch_enc, &hspi1, ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin);
  AS5048A_Init(&roll_enc, &hspi1, ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin);
  AS5048A_Init(&yaw_enc, &hspi1, ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin);
  HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim6); // TIMER 6 dùng để đọc các ngoại vi như encoder và IMU
  HAL_Delay(100); // Chờ lấy mẫu vài frame góc ban đầu từ AS5048A

  /*=== Khởi tạo trục Pitch ===*/
  FOC_Init(&foc_pitch, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, /* voltage_supply: Bus DC 12V */
           VOLTAGE_LIMIT, 1.0f, 0.0005f, 0.00005f);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_pitch, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_pitch, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_pitch, 0.96f);

  /*=== Khởi tạo trục Roll ===*/
  FOC_Init(&foc_roll, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_roll, 8.0f, 0.4f, 0.0f, -4.0f, 4.0f);
  FOC_SetPID_VEL(&foc_roll, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_roll, 0.96f);

  /*=== Khởi tạo trục Yaw ===*/
  FOC_Init(&foc_yaw, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_yaw, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_yaw, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_yaw, 0.96f);

  foc_pitch.enabled = 1;
  foc_roll.enabled = 1;
  foc_yaw.enabled = 1;

  FOC_AlignD(&foc_pitch, 1.0f);
  FOC_AlignD(&foc_roll, 1.0f);
  FOC_AlignD(&foc_yaw, 1.0f);
  HAL_Delay(1000);

  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  FOC_CalibrateAngle(&foc_roll, roll_enc.angle_rad);
  FOC_CalibrateAngle(&foc_yaw, yaw_enc.angle_rad);

  FOC_Start(&foc_pitch, pitch_enc.angle_rad);
  FOC_Start(&foc_roll, roll_enc.angle_rad);
  FOC_Start(&foc_yaw, yaw_enc.angle_rad);

  /*=== Khởi tạo PID vòng ngoài cho cả 3 trục ===*/
  PID_Init(&pid_3ax_roll_pos, 4.0f, 0.3f, 0.05f, -3.0f, 3.0f);
  PID_Init(&pid_3ax_pitch_pos, 4.0f, 0.3f, 0.05f, -3.0f, 3.0f);
  PID_Init(&pid_3ax_yaw_pos, 3.0f, 0.2f, 0.0f, -2.0f, 2.0f);

  
  ICM42688_Status_t icm_3ax_status = ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_3ax_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[DEMO_3AXIS] ICM42688 OK! Dang hieu chuan Gyro Bias (giu im 1s)...\r\n");
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[DEMO_3AXIS] Bias: X=%.2f Y=%.2f Z=%.2f dps\r\n",
           imu_payload.gyro_bias_x, imu_payload.gyro_bias_y,
           imu_payload.gyro_bias_z);
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
    printf("[DEMO_3AXIS] Cho Mahony AHRS on định (2s)...\r\n");
    HAL_Delay(2000);

    Quaternion_FromEuler(DEG2RAD(DEMO3AX_TARGET_ROLL_DEG),
                         DEG2RAD(DEMO3AX_TARGET_PITCH_DEG),
                         DEG2RAD(DEMO3AX_TARGET_YAW_DEG), &q_demo3_target);

    printf("[DEMO_3AXIS] Huong muc tieu co dinh: Roll=%.1f, Pitch=%.1f, Yaw=%.1f (deg)\r\n",
           DEMO3AX_TARGET_ROLL_DEG, DEMO3AX_TARGET_PITCH_DEG, DEMO3AX_TARGET_YAW_DEG);
    printf("[DEMO_3AXIS] Quat muc tieu: w=%.3f x=%.3f y=%.3f z=%.3f\r\n",
           q_demo3_target.q0, q_demo3_target.q1, q_demo3_target.q2,
           q_demo3_target.q3);
  } else {
    icm_init_ok = 0;
    printf("[DEMO_3AXIS] LOI khoi tao ICM42688! Code: %d\r\n", icm_3ax_status);
  }

  HAL_TIM_Base_Start_IT(&htim7);  // TIMER7 chạy FOC
  SBUS_Status_t sbus_init_ret = SBUS_Init(&sbus_rx, &huart1);
  if (sbus_init_ret == SBUS_OK) {
    printf("[SBUS] Khoi tao thanh cong!\r\n");
  } else {
    printf("[SBUS] LOI khoi tao! Check USART1 DMA config.\r\n");
  }

  /* Đặt target về vị trí HOME — TIM7 ISR sẽ tự động drive motor đến đây */
  target_roll_angle  = DEG2RAD(HOME_ROLL_DEG);
  target_pitch_angle = DEG2RAD(HOME_PITCH_DEG);
  target_yaw_angle   = DEG2RAD(HOME_YAW_DEG);
  printf("[HOME] Bat dau Homing: Roll=%.1f Pitch=%.1f Yaw=%.1f (deg)\r\n",
         HOME_ROLL_DEG, HOME_PITCH_DEG, HOME_YAW_DEG);

  uint32_t last_print_time = HAL_GetTick();
  uint32_t home_start_tick = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
    uint32_t now = HAL_GetTick();

    /* Luon gọi SBUS_Process để DMA ring buffer luôn được đọc */
    SBUS_Status_t sbus_st = SBUS_Process(&sbus_rx);

    /* === STATE MACHINE === */
    if (g_gimbal_state == GIMBAL_STATE_HOMING) {
      /* Kiểm tra từng trục đã về home chưa (dùng angle_deg từ encoder) */
      float err_roll  = roll_enc.angle_deg  - HOME_ROLL_DEG;
      float err_pitch = pitch_enc.angle_deg - HOME_PITCH_DEG;
      float err_yaw   = yaw_enc.angle_deg   - HOME_YAW_DEG;
      if (err_roll  < 0.0f) err_roll  = -err_roll;
      if (err_pitch < 0.0f) err_pitch = -err_pitch;
      if (err_yaw   < 0.0f) err_yaw   = -err_yaw;

      uint8_t homed = (err_roll  <= HOME_TOL_DEG)
                   && (err_pitch <= HOME_TOL_DEG)
                   && (err_yaw   <= HOME_TOL_DEG);
      uint8_t timeout = ((now - home_start_tick) >= HOME_TIMEOUT_MS);

      if (homed || timeout) {
        /* Chuyển sang chế độ SBUS, khởi đầu từ vị trí encoder hiện tại */
        SBUS_Mapping_Init(&sbus_map,
                          &target_roll_angle,
                          &target_pitch_angle,
                          &target_yaw_angle,
                          roll_enc.angle_rad,
                          pitch_enc.angle_rad,
                          yaw_enc.angle_rad);
        g_gimbal_state = GIMBAL_STATE_SBUS;
        if (homed) {
          printf("[HOME] Homing HOAN THANH! Chuyen sang che do SBUS.\r\n");
        } else {
          printf("[HOME] TIMEOUT! Chuyen sang che do SBUS (eR=%.1f eP=%.1f eY=%.1f).\r\n",
                 err_roll, err_pitch, err_yaw);
        }
      }
    } else {
      /* GIMBAL_STATE_SBUS: nhận lệnh điều khiển từ tay điều */
      SBUS_Mapping_Update(&sbus_map, &sbus_rx, sbus_st);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (now - last_print_time >= 100) {
      last_print_time = now;

      if (g_gimbal_state == GIMBAL_STATE_HOMING) {
        /* In tiến trình homing */
        printf("[HOMING] Roll:%6.1f→%.1f | Pitch:%6.1f→%.1f | Yaw:%6.1f→%.1f (deg)\r\n",
               roll_enc.angle_deg,  HOME_ROLL_DEG,
               pitch_enc.angle_deg, HOME_PITCH_DEG,
               yaw_enc.angle_deg,   HOME_YAW_DEG);
      } else {
        /* In telemetry chế độ SBUS */
        if (sbus_st == SBUS_OK) {
          uint16_t ch1, ch2, ch4;
          SBUS_GetChannel(&sbus_rx, 1, &ch1);
          SBUS_GetChannel(&sbus_rx, 2, &ch2);
          SBUS_GetChannel(&sbus_rx, 4, &ch4);
          const char *cmd_str[] = {"NEG", "HOLD", "POS"};
          printf("[TARGET] Roll:%6.1f | Pitch:%6.1f | Yaw:%6.1f (deg)\r\n",
                 target_roll_angle  * 57.2957795f,
                 target_pitch_angle * 57.2957795f,
                 target_yaw_angle   * 57.2957795f);
          printf("[ENC]    Roll:%6.1f | Pitch:%6.1f | Yaw:%6.1f (deg)\r\n",
                 roll_enc.angle_deg,
                 pitch_enc.angle_deg,
                 yaw_enc.angle_deg);
          printf("[SBUS]   CH1=%4u(%s) CH2=%4u(%s) CH4=%4u(%s)\r\n",
                 ch1, cmd_str[sbus_map.cmd_roll  + 1],
                 ch2, cmd_str[sbus_map.cmd_pitch + 1],
                 ch4, cmd_str[sbus_map.cmd_yaw   + 1]);
        } else if (sbus_st == SBUS_FAILSAFE) {
          printf("[SBUS] CANH BAO: FAILSAFE dang active!\r\n");
        } else if (sbus_st == SBUS_FRAME_LOST) {
          printf("[SBUS] CANH BAO: Frame Lost!\r\n");
        } else if (sbus_st == SBUS_TIMEOUT) {
          printf("[SBUS] LOI: TIMEOUT! Mat tin hieu Receiver (> %dms).\r\n", SBUS_TIMEOUT_MS);
        } else {
          printf("[SBUS] Dang cho frame dau tien...\r\n");
        }
      }

    }
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

/* ============================================================
 * SPI DMA Buffers cho Encoder AS5048A (2-frame pipeline)
 *
 * AS5048A protocol: 2 SPI frames cần thiết để đọc angle
 *   Phase 0: MOSI = READ ANGLE (0xFFFF) → MISO = rác (ignored)
 *   Phase 1: MOSI = NOP (0xC000)         → MISO = angle data
 * ============================================================ */
static uint16_t enc_tx_read_cmd = 0xFFFF; /* READ ANGLE command */
static uint16_t enc_tx_nop_cmd = 0xC000; /* NOP: nhận data từ frame trước */
static uint16_t enc_rx_dummy;            /* Frame 0: bỏ qua */
static uint16_t enc_rx_angle;            /* Frame 1: chứa angle data */
volatile uint8_t enc_busy = 0;
static uint8_t enc_phase = 0; /* 0 = gửi lệnh, 1 = lấy data */


/* ============================================================
 * SPI DMA Buffers cho ICM42688 (15-byte Burst Read Pipeline)
 * ============================================================ */
static uint8_t icm_tx_buf[15] = {0x1D | 0x80, 0}; /* 0x1D | 0x80: Read command bắt đầu từ TEMP_DATA1 */
static uint8_t icm_rx_buf[15];
volatile uint8_t icm_dma_busy = 0;


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    /* Đã chuyển sang dùng Polling trong TIM6 cho hspi1 */
  } else if (hspi->Instance == SPI3) {
    /* SPI3 DMA Burst Read ICM42688 xong (15 bytes) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    icm_dma_busy = 0;

    /* Parse 14-byte data từ icm_rx_buf (byte 0 là dummy/cmd) */
    imu_payload.raw_temp = (int16_t)((icm_rx_buf[1] << 8) | icm_rx_buf[2]);
    imu_payload.raw_accel_x = (int16_t)((icm_rx_buf[3] << 8) | icm_rx_buf[4]);
    imu_payload.raw_accel_y = (int16_t)((icm_rx_buf[5] << 8) | icm_rx_buf[6]);
    imu_payload.raw_accel_z = (int16_t)((icm_rx_buf[7] << 8) | icm_rx_buf[8]);
    imu_payload.raw_gyro_x = (int16_t)((icm_rx_buf[9] << 8) | icm_rx_buf[10]);
    imu_payload.raw_gyro_y = (int16_t)((icm_rx_buf[11] << 8) | icm_rx_buf[12]);
    imu_payload.raw_gyro_z = (int16_t)((icm_rx_buf[13] << 8) | icm_rx_buf[14]);

    float gs = imu_payload.gyro_sensitivity;
    float as = imu_payload.accel_sensitivity;

    float gx = (float)imu_payload.raw_gyro_x / gs;
    float gy = (float)imu_payload.raw_gyro_y / gs;
    float gz = (float)imu_payload.raw_gyro_z / gs;

    if (imu_payload.gyro_calibrated) {
      gx -= imu_payload.gyro_bias_x;
      gy -= imu_payload.gyro_bias_y;
      gz -= imu_payload.gyro_bias_z;
    }

    imu_payload.gyro_x_dps = gx;
    imu_payload.gyro_y_dps = gy;
    imu_payload.gyro_z_dps = gz;

    imu_payload.accel_x_g = (float)imu_payload.raw_accel_x / as;
    imu_payload.accel_y_g = (float)imu_payload.raw_accel_y / as;
    imu_payload.accel_z_g = (float)imu_payload.raw_accel_z / as;
    imu_payload.temp_c =
        (float)imu_payload.raw_temp / ICM42688_TEMP_SENS + ICM42688_TEMP_OFFSET;

    /* Cập nhật Mahony 3D AHRS ngay tại ngắt DMA 1kHz (dt = 0.001s) */
    Mahony_Update(&mahony_imu, imu_payload.gyro_x_dps * DEG_TO_RAD,
                  imu_payload.gyro_y_dps * DEG_TO_RAD,
                  imu_payload.gyro_z_dps * DEG_TO_RAD, imu_payload.accel_x_g,
                  imu_payload.accel_y_g, imu_payload.accel_z_g, 0.001f);

    Quaternion_t q_meas_3d = {mahony_imu.q0, mahony_imu.q1, mahony_imu.q2,
                              mahony_imu.q3};
    Quaternion_ComputeError(&q_target_3d, &q_meas_3d, e_rot);
  }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    if (!enc_busy) {
      enc_busy = 1;
      uint16_t rx_dummy;
      uint16_t rx_angle_pitch, rx_angle_roll, rx_angle_yaw;

      /* === Phase 0 (Pipeline): Gửi lệnh READ ANGLE cho cả 3 trục liên tiếp ===
       */
      /* PITCH */
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd, (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);

      /* ROLL */
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd, (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);

      /* YAW */
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd, (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

      /* === Phase 1 (Pipeline): Gửi lệnh NOP để clock ra Data góc === */
      /* PITCH */
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd, (uint8_t *)&rx_angle_pitch, 1, 2);
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);

      /* ROLL */
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd, (uint8_t *)&rx_angle_roll, 1, 2);
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);

      /* YAW */
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd, (uint8_t *)&rx_angle_yaw, 1, 2);
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

      /* === Xử lý Parity và tính Góc === */
      if (AS5048A_CheckParity(rx_angle_pitch)) {
        pitch_enc.raw_angle = rx_angle_pitch & AS5048A_DATA_MASK;
        pitch_enc.angle_rad = (float)pitch_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        pitch_enc.angle_deg = (float)pitch_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
      if (AS5048A_CheckParity(rx_angle_roll)) {
        roll_enc.raw_angle = rx_angle_roll & AS5048A_DATA_MASK;
        roll_enc.angle_rad = (float)roll_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        roll_enc.angle_deg = (float)roll_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
      if (AS5048A_CheckParity(rx_angle_yaw)) {
        yaw_enc.raw_angle = rx_angle_yaw & AS5048A_DATA_MASK;
        yaw_enc.angle_rad = (float)yaw_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        yaw_enc.angle_deg = (float)yaw_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }

      enc_busy = 0;
    }
    /* TIM6: Kích hoạt SPI3 DMA đọc Burst 15-byte cho ICM42688 (SPI3) */
    if (!icm_dma_busy && icm_init_ok) {
      icm_dma_busy = 1;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
      if (HAL_SPI_TransmitReceive_DMA(&hspi3, icm_tx_buf, icm_rx_buf, 15) !=HAL_OK) {
        icm_dma_busy = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      }
    }
  } else if (htim->Instance == TIM7) {
    /* Clamp mục tiêu trong giới hạn mềm (Soft Limit) trước khi đưa vào FOC */
    const float pitch_min_rad = DEG2RAD(PITCH_MIN_DEG);
    const float pitch_max_rad = DEG2RAD(PITCH_MAX_DEG);
    const float roll_min_rad = DEG2RAD(ROLL_MIN_DEG);
    const float roll_max_rad = DEG2RAD(ROLL_MAX_DEG);
    const float yaw_min_rad = DEG2RAD(YAW_MIN_DEG);
    const float yaw_max_rad = DEG2RAD(YAW_MAX_DEG);

    if (target_pitch_angle < pitch_min_rad) target_pitch_angle = pitch_min_rad;
    if (target_pitch_angle > pitch_max_rad) target_pitch_angle = pitch_max_rad;
    if (target_roll_angle < roll_min_rad) target_roll_angle = roll_min_rad;
    if (target_roll_angle > roll_max_rad) target_roll_angle = roll_max_rad;
    if (target_yaw_angle < yaw_min_rad) target_yaw_angle = yaw_min_rad;
    if (target_yaw_angle > yaw_max_rad) target_yaw_angle = yaw_max_rad;

    /* Gọi hàm Position Loop cho cả 3 trục */
    FOC_PositionLoop(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
    FOC_PositionLoop(&foc_roll, roll_enc.angle_rad, target_roll_angle);
    FOC_PositionLoop(&foc_yaw, yaw_enc.angle_rad, target_yaw_angle);
  }
}


/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
