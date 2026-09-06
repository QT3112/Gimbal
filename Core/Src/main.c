/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5048a.h"
#include "foc_v1.h"
#include "icm42688.h"
#include "imu_filter.h"
#include "mapping_sbus_channel.h"
#include "sbus_protocol.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USE_EULER_ANGLE_CONTROL 1 /* 1: Dùng Euler, 0: Dùng Quaternion cho vòng ngoài */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* === Giới hạn góc mềm trục motor === */
#define MOTOR_PITCH_MIN_DEG 15.0f
#define MOTOR_PITCH_MAX_DEG 280.0f
#define MOTOR_ROLL_MIN_DEG 90.0f
#define MOTOR_ROLL_MAX_DEG 325.0f 
#define MOTOR_YAW_MIN_DEG 65.0f
#define MOTOR_YAW_MAX_DEG 255.0f
#define MOTOR_YAW_FORBIDDEN_MIN_DEG 65.0f
#define MOTOR_YAW_FORBIDDEN_MAX_DEG 225.0f


/*=== Vị trí Home motor (khởi động tự động quay về đây) ===*/
#define HOME_MOTOR_ROLL_DEG 312.0f
#define HOME_MOTOR_PITCH_DEG 115.0f
#define HOME_MOTOR_YAW_DEG 342.0f
#define HOME_MOTOR_TOL_DEG 2.0f /* Sai số cho phép để coi là đã đến home_MOTOR (deg) */
#define HOME_MOTOR_TIMEOUT_MS 12000 /* Tối đa 12 giây để homing */

/*=== Góc mục tiêu cố định motor (dự phòng) ===*/
#define MOTOR_PITCH_SETPOINT_DEG 220.0f
#define MOTOR_ROLL_SETPOINT_DEG 185.0f
#define MOTOR_YAW_SETPOINT_DEG 180.0f

/*=== Góc mục tiêu của IMU ===*/
#define IMU_PITCH_TARGET_DEG 0.0f
#define IMU_ROLL_TARGET_DEG 0.0f
#define IMU_YAW_TARGET_DEG 150.0f

/*=== Cấu hình phần cứng mạch dòng ===*/
#define GAIN_DRV 10.0f
#define SHUNT_RES 0.005f
#define VOLTAGE_LIMIT 2.0f
#define PWM_PERIOD 4249.0f
#define MOTOR_POLE_PAIRS 7

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f
#define DEG2RAD(d) ((d) * DEG_TO_RAD)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PROGRAM_MODE_MAIN 0
#define PROGRAM_MODE_IMU_TEST 1
#define PROGRAM_MODE_3AXIS_FOLLOW_IMU 2
#define PROGRAM_MODE PROGRAM_MODE_3AXIS_FOLLOW_IMU
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

AS5048A_Handle_t motor_pitch_enc;
AS5048A_Handle_t motor_roll_enc;
AS5048A_Handle_t motor_yaw_enc;
FOC_Handle_t foc_motor_pitch;
FOC_Handle_t foc_motor_roll;
FOC_Handle_t foc_motor_yaw;

ICM42688_Handle_t imu_payload;
MahonyFilter_t mahony_imu;
uint8_t icm_init_ok = 0;

Quaternion_t q_demo3_target; /* Hướng mục tiêu: chốt khi vào mode, giữ cố định sau đó */

/* PID vòng ngoài: error góc (rad) → velo setpoint (rad/s) cho từng trục */
PID_Handle_t pid_motor_roll_pos;
PID_Handle_t pid_motor_pitch_pos;
PID_Handle_t pid_motor_yaw_pos;

PID_Handle_t pid_imu_roll_pos;
PID_Handle_t pid_imu_pitch_pos;
PID_Handle_t pid_imu_yaw_pos;

SBUS_Handle_t sbus_rx;
SBUS_Mapping_Handle_t sbus_map; /* Mapping SBUS raw → target angle gimbal */

volatile float target_motor_pitch_angle = 0.0f;
volatile float target_motor_roll_angle = 0.0f;
volatile float target_motor_yaw_angle = 0.0f;

volatile float imu_stab_vel_roll = 0.0f;
volatile float imu_stab_vel_pitch = 0.0f;
volatile float imu_stab_vel_yaw = 0.0f;

/* Góc IMU chốt lúc startup — giữ cố định làm target ổn định */
volatile float imu_pitch_target_rad = 0.0f;
volatile float imu_yaw_target_rad   = 0.0f;

/* === Trạng thái máy trạng thái của gimbal === */
typedef enum {
  GIMBAL_STATE_HOMING = 0,
  GIMBAL_STATE_SBUS,
  GIMBAL_STATE_IMU_STAB,
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
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  uint32_t last_print_time = HAL_GetTick();
  uint32_t home_start_tick = HAL_GetTick();

#if (PROGRAM_MODE == PROGRAM_MODE_MAIN)

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*=== Khởi tạo encoder ===*/
  AS5048A_Init(&motor_pitch_enc, &hspi1, ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin);
  AS5048A_Init(&motor_roll_enc, &hspi1, ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin);
  AS5048A_Init(&motor_yaw_enc, &hspi1, ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin);
  HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim6); // TIMER 6 dùng để đọc các ngoại vi như encoder và IMU
  HAL_Delay(100); // Chờ lấy mẫu vài frame góc ban đầu từ AS5048A

  /*=== Khởi tạo trục Pitch ===*/
  FOC_Init(&foc_motor_pitch, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, /* voltage_supply: Bus DC 12V */
           VOLTAGE_LIMIT, 1.0f, 0.0005f, 0.00005f);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_motor_pitch, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_motor_pitch, 0.4f, 0.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_motor_pitch, 0.96f);

  // /*=== Khởi tạo trục Roll ===*/
  FOC_Init(&foc_motor_roll, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_motor_roll, 8.0f, 0.4f, 0.0f, -4.0f, 4.0f);
  FOC_SetPID_VEL(&foc_motor_roll, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_motor_roll, 0.96f);

  /*=== Khởi tạo trục Yaw ===*/
  FOC_Init(&foc_motor_yaw, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_motor_yaw, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_motor_yaw, 0.4f, 0.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_motor_yaw, 0.96f);

  foc_motor_pitch.enabled = 1;
  foc_motor_roll.enabled = 1;
  foc_motor_yaw.enabled = 1;

  FOC_AlignD(&foc_motor_pitch, 1.0f);
  FOC_AlignD(&foc_motor_roll, 1.0f);
  FOC_AlignD(&foc_motor_yaw, 1.0f);
  HAL_Delay(1000);

  FOC_CalibrateAngle(&foc_motor_pitch, motor_pitch_enc.angle_rad);
  FOC_CalibrateAngle(&foc_motor_roll, motor_roll_enc.angle_rad);
  FOC_CalibrateAngle(&foc_motor_yaw, motor_yaw_enc.angle_rad);

  FOC_Start(&foc_motor_pitch, motor_pitch_enc.angle_rad);
  FOC_Start(&foc_motor_roll, motor_roll_enc.angle_rad);
  FOC_Start(&foc_motor_yaw, motor_yaw_enc.angle_rad);

  /*=== Khởi tạo IMU ===*/
  ICM42688_Status_t imu_status = ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (imu_status == ICM42688_OK) {
    icm_init_ok = 1;
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
    HAL_Delay(2000);
    printf("[IMU] AHRS hien tai: R=%.2f P=%.2f Y=%.2f (deg)\r\n",
           mahony_imu.roll * RAD_TO_DEG, 
           mahony_imu.pitch * RAD_TO_DEG,
           mahony_imu.yaw * RAD_TO_DEG);
  } else {
    icm_init_ok = 0;
    printf("[IMU] LOI khoi tao ICM42688! Code: %d\r\n", imu_status);
  }

  
  HAL_TIM_Base_Start_IT(&htim16); // Outer PID
  HAL_TIM_Base_Start_IT(&htim7);  // Inner PID

  SBUS_Status_t sbus_init_ret = SBUS_Init(&sbus_rx, &huart1);
  if (sbus_init_ret == SBUS_OK) {
    printf("[SBUS] Khoi tao thanh cong!\r\n");
  } else {
    printf("[SBUS] LOI khoi tao! Check USART1 DMA config.\r\n");
  }

  /* Đặt target về vị trí HOME — TIM7 ISR sẽ tự động drive motor đến đây */
  target_motor_roll_angle = DEG2RAD(HOME_MOTOR_ROLL_DEG);
  target_motor_pitch_angle = DEG2RAD(HOME_MOTOR_PITCH_DEG);
  target_motor_yaw_angle = DEG2RAD(HOME_MOTOR_YAW_DEG);
  printf("[HOME] Bat dau Homing: Roll=%.1f Pitch=%.1f Yaw=%.1f (deg)\r\n",
         HOME_MOTOR_ROLL_DEG, HOME_MOTOR_PITCH_DEG, HOME_MOTOR_YAW_DEG);
#elif (PROGRAM_MODE == PROGRAM_MODE_IMU_TEST)
  HAL_Delay(1000);
#elif (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*=== Khởi tạo encoder Pitch và Yaw ===*/
  AS5048A_Init(&motor_pitch_enc, &hspi1, ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin);
  AS5048A_Init(&motor_yaw_enc,   &hspi1, ENC_YAW_CS_GPIO_Port,   ENC_YAW_CS_Pin);
  HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port,   ENC_YAW_CS_Pin,   GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim6); /* TIM6: đọc encoder + trigger IMU DMA */
  HAL_Delay(100);                /* Chờ vài frame encoder ổn định */

  /*=== Khởi tạo FOC trục Pitch (TIM3) ===*/
  FOC_Init(&foc_motor_pitch, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f, 0.00005f);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  FOC_SetPID_VEL(&foc_motor_pitch, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_motor_pitch, 0.96f);

  /*=== Khởi tạo FOC trục Yaw (TIM1) ===*/
  FOC_Init(&foc_motor_yaw, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f, 0.00005f);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  FOC_SetPID_VEL(&foc_motor_yaw, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_motor_yaw, 0.96f);

  foc_motor_pitch.enabled = 1;
  foc_motor_yaw.enabled   = 1;

  /*=== AlignD: kéo rotor về D-axis để calibrate angle offset ===*/
  FOC_AlignD(&foc_motor_pitch, 1.0f);
  FOC_AlignD(&foc_motor_yaw,   1.0f);
  HAL_Delay(1000);

  FOC_CalibrateAngle(&foc_motor_pitch, motor_pitch_enc.angle_rad);
  FOC_CalibrateAngle(&foc_motor_yaw,   motor_yaw_enc.angle_rad);

  FOC_Start(&foc_motor_pitch, motor_pitch_enc.angle_rad);
  FOC_Start(&foc_motor_yaw,   motor_yaw_enc.angle_rad);

  /*=== Khởi tạo IMU ICM42688 ===*/
  ICM42688_Status_t imu_status3 = ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (imu_status3 == ICM42688_OK) {
    icm_init_ok = 1;
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
    printf("[IMU] Khoi tao OK. Cho Mahony hoi tu (2s)...\r\n");
    HAL_Delay(2000); /* Chờ Mahony filter hội tụ */

    /* Chốt góc IMU hiện tại làm target ổn định (Dừ gimbal ở vị trí mong muốn trong 2s này!) */
    imu_pitch_target_rad = mahony_imu.pitch;
    imu_yaw_target_rad   = mahony_imu.yaw;
    printf("[IMU] Target chot: Pitch=%.2f  Yaw=%.2f (deg)\r\n",
           imu_pitch_target_rad * RAD_TO_DEG,
           imu_yaw_target_rad   * RAD_TO_DEG);
  } else {
    icm_init_ok = 0;
    printf("[IMU] LOI khoi tao! Code: %d\r\n", imu_status3);
  }

  /*=== Khởi tạo PID outer loop: IMU angle error → velocity setpoint [rad/s] ===*/
  PID_Init(&pid_imu_pitch_pos, 2.0f, 0.05f, 0.0f, -8.0f, 8.0f);
  PID_Init(&pid_imu_yaw_pos,   2.0f, 0.05f, 0.0f, -8.0f, 8.0f);

  /*=== Khởi động timers điều khiển ===*/
  HAL_TIM_Base_Start_IT(&htim16); /* Outer PID @ 500Hz */
  HAL_TIM_Base_Start_IT(&htim7);  /* Inner FOC @ 2kHz  */
  printf("[3AXIS_IMU] San sang! Bat dau on dinh.\r\n");
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  

  while (1) {
#if (PROGRAM_MODE == PROGRAM_MODE_MAIN)
    uint32_t now = HAL_GetTick();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (now - last_print_time >= 100) {
      last_print_time = now;
    }

    printf(
          "[AHRS]  R:%5.2f P:%5.2f Y:%5.2f (deg) | [GYRO]  Gx:%5.2f Gy:%5.2f "
          "Gz:%5.2f (dps) | [ACCEL] Ax:%5.3f Ay:%5.3f Az:%5.3f (g) \r\n",
          mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG,
          mahony_imu.yaw * RAD_TO_DEG, imu_payload.gyro_x_dps,
          imu_payload.gyro_y_dps, imu_payload.gyro_z_dps, imu_payload.accel_x_g,
          imu_payload.accel_y_g, imu_payload.accel_z_g);

    printf("[ENC] R:%6.1f P:%6.1f Y:%6.1f (deg) | [FORCE] R:%5.2f P:%5.2f Y:%5.2f (V)\r\n",
                 motor_roll_enc.angle_deg, motor_pitch_enc.angle_deg, motor_yaw_enc.angle_deg,
                 foc_motor_roll.Vq_ref, foc_motor_pitch.Vq_ref, foc_motor_yaw.Vq_ref);

    HAL_Delay(100);

#elif (PROGRAM_MODE == PROGRAM_MODE_IMU_TEST)
    HAL_Delay(1000);
    printf("Test \r\n");
#elif (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
    uint32_t now = HAL_GetTick();
    if (now - last_print_time >= 100) {
      last_print_time = now;
      printf("[AHRS] P:%6.2f Y:%6.2f (deg) | [TGT] P:%6.2f Y:%6.2f (deg) | [ENC] P:%6.1f Y:%6.1f (deg) | [FORCE] P:%5.2f Y:%5.2f (V)\r\n",
             mahony_imu.pitch * RAD_TO_DEG, mahony_imu.yaw * RAD_TO_DEG,
             imu_pitch_target_rad * RAD_TO_DEG, imu_yaw_target_rad * RAD_TO_DEG,
             motor_pitch_enc.angle_deg, motor_yaw_enc.angle_deg,
             foc_motor_pitch.Vq_ref, foc_motor_yaw.Vq_ref);
    }
#endif
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
    imu_payload.temp_c = (float)imu_payload.raw_temp / ICM42688_TEMP_SENS + ICM42688_TEMP_OFFSET;

    Mahony_Update(&mahony_imu, imu_payload.gyro_x_dps * DEG_TO_RAD,
                  imu_payload.gyro_y_dps * DEG_TO_RAD,
                  imu_payload.gyro_z_dps * DEG_TO_RAD, imu_payload.accel_x_g,
                  imu_payload.accel_y_g, imu_payload.accel_z_g, 0.0005f);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    if (!enc_busy) {
      enc_busy = 1;
      uint16_t rx_dummy;
      uint16_t rx_angle_pitch, rx_angle_roll, rx_angle_yaw;

      /* === Phase 0 (Pipeline): Gửi lệnh READ ANGLE cho cả 3 trục liên tiếp === */
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
        motor_pitch_enc.raw_angle = rx_angle_pitch & AS5048A_DATA_MASK;
        motor_pitch_enc.angle_rad = (float)motor_pitch_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        motor_pitch_enc.angle_deg = (float)motor_pitch_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
      if (AS5048A_CheckParity(rx_angle_roll)) {
        motor_roll_enc.raw_angle = rx_angle_roll & AS5048A_DATA_MASK;
        motor_roll_enc.angle_rad = (float)motor_roll_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        motor_roll_enc.angle_deg = (float)motor_roll_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
      if (AS5048A_CheckParity(rx_angle_yaw)) {
        motor_yaw_enc.raw_angle = rx_angle_yaw & AS5048A_DATA_MASK;
        motor_yaw_enc.angle_rad = (float)motor_yaw_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        motor_yaw_enc.angle_deg = (float)motor_yaw_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }

      enc_busy = 0;
    }
    /* TIM6: Kích hoạt SPI3 DMA đọc Burst 15-byte cho ICM42688 (SPI3) */
    if (!icm_dma_busy && icm_init_ok) {
      icm_dma_busy = 1;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
      if (HAL_SPI_TransmitReceive_DMA(&hspi3, icm_tx_buf, icm_rx_buf, 15) != HAL_OK) {
        icm_dma_busy = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      }
    }
  } else if (htim->Instance == TIM16) {
    /* Outer loop @ 500Hz: IMU angle error → velocity setpoint [rad/s] cho FOC_VelocityLoop */
    if (icm_init_ok) {
      /* Sai số: target chốt lúc startup - góc IMU thực tế
       * Khi frame nghiêng pitch tăng → error âm → velocity âm → motor bù ngược lại */
      float imu_error_pitch = -imu_pitch_target_rad + mahony_imu.pitch;
      float imu_error_yaw   = -imu_yaw_target_rad   + mahony_imu.yaw;

      /* Wrap về [-π, +π]: chọn hướng bù ngắn nhất (quan trọng cho Yaw) */
      if (imu_error_pitch >  PI) imu_error_pitch -= TWO_PI;
      if (imu_error_pitch < -PI) imu_error_pitch += TWO_PI;
      if (imu_error_yaw   >  PI) imu_error_yaw   -= TWO_PI;
      if (imu_error_yaw   < -PI) imu_error_yaw   += TWO_PI;

      /* PID → velocity setpoint [rad/s] */
      imu_stab_vel_pitch = PID_Update(&pid_imu_pitch_pos, imu_error_pitch, 0.002f);
      imu_stab_vel_yaw   = PID_Update(&pid_imu_yaw_pos,   imu_error_yaw,   0.002f);
    }
  } else if (htim->Instance == TIM7) {
#if (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
    /* ================================================================
     * 3AXIS_FOLLOW_IMU: Vòng trong (2kHz) dùng FOC_VelocityLoop
     * Velocity setpoint được tính từ outer loop (TIM16 @ 500Hz)
     * ================================================================ */
    FOC_VelocityLoop(&foc_motor_pitch, motor_pitch_enc.angle_rad, imu_stab_vel_pitch);
    //FOC_VelocityLoop(&foc_motor_roll, motor_roll_enc.angle_rad, imu_stab_vel_roll);
    FOC_VelocityLoop(&foc_motor_yaw, motor_yaw_enc.angle_rad, imu_stab_vel_yaw);
#else
    /* ================================================================
     * PROGRAM_MODE_MAIN: Tach hai nhánh SBUS/HOMING va IMU_STAB
     * ================================================================ */
    
    const float pitch_min_rad = DEG2RAD(MOTOR_PITCH_MIN_DEG);
    const float pitch_max_rad = DEG2RAD(MOTOR_PITCH_MAX_DEG);
    const float roll_min_rad = DEG2RAD(MOTOR_ROLL_MIN_DEG);
    const float roll_max_rad = DEG2RAD(MOTOR_ROLL_MAX_DEG);
    // const float yaw_min_rad = DEG2RAD(MOTOR_YAW_MIN_DEG);
    // const float yaw_max_rad = DEG2RAD(MOTOR_YAW_MAX_DEG);
    const float yaw_forbid_min_rad = DEG2RAD(MOTOR_YAW_FORBIDDEN_MIN_DEG);
    const float yaw_forbid_max_rad = DEG2RAD(MOTOR_YAW_FORBIDDEN_MAX_DEG);


    if (target_motor_pitch_angle < pitch_min_rad)
      target_motor_pitch_angle = pitch_min_rad;
    if (target_motor_pitch_angle > pitch_max_rad)
      target_motor_pitch_angle = pitch_max_rad;
    if (target_motor_roll_angle < roll_min_rad)
      target_motor_roll_angle = roll_min_rad;
    if (target_motor_roll_angle > roll_max_rad)
      target_motor_roll_angle = roll_max_rad;
    // if (target_motor_yaw_angle < yaw_min_rad)
    //   target_motor_yaw_angle = yaw_min_rad;
    // if (target_motor_yaw_angle > yaw_max_rad)
    //   target_motor_yaw_angle = yaw_max_rad;


    // Kiểm tra xem target có bị rơi vào vùng cấm không (từ 65 độ -> 225 độ)
    if (target_motor_yaw_angle > yaw_forbid_min_rad && target_motor_yaw_angle < yaw_forbid_max_rad) {
      float dist_to_min = target_motor_yaw_angle - yaw_forbid_min_rad;
      float dist_to_max = yaw_forbid_max_rad - target_motor_yaw_angle;
      if (dist_to_min < dist_to_max) {
        target_motor_yaw_angle = yaw_forbid_min_rad; // Đẩy về 65 độ
      } else {
        target_motor_yaw_angle = yaw_forbid_max_rad; // Đẩy về 225 độ
      }
    }


    FOC_PositionLoop(&foc_motor_pitch, motor_pitch_enc.angle_rad, target_motor_pitch_angle);
    //FOC_PositionLoop(&foc_motor_roll,  motor_roll_enc.angle_rad,  target_motor_roll_angle);
    FOC_PositionLoop(&foc_motor_yaw,   motor_yaw_enc.angle_rad,   target_motor_yaw_angle);
  
#endif
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
