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
#include "mapping_sbus_channel.h"
#include "sbus_protocol.h"
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
#define PITCH_MIN_DEG 15.0f
#define PITCH_MAX_DEG 280.0f
#define ROLL_MIN_DEG 90.0f
#define ROLL_MAX_DEG 325.0f /* Mở rộng để cho phép vị trí home 310° */
#define YAW_MIN_DEG 65.0f
#define YAW_MAX_DEG 255.0f

/*=== Vị trí Home motor (khởi động tự động quay về đây) ===*/
#define HOME_ROLL_DEG 310.0f
#define HOME_PITCH_DEG 215.0f
#define HOME_YAW_DEG 165.0f
#define HOME_TOL_DEG 2.0f /* Sai số cho phép để coi là đã đến home (deg) */
#define HOME_TIMEOUT_MS 12000 /* Tối đa 12 giây để homing */

/*=== Góc mục tiêu cố định motor (dự phòng) ===*/
#define PITCH_SETPOINT_DEG 220.0f
#define ROLL_SETPOINT_DEG 185.0f
#define YAW_SETPOINT_DEG 180.0f
/*=== Góc mục tiêu cố định IMU ===*/
#define DEMO3AX_TARGET_ROLL_DEG -1.7f
#define DEMO3AX_TARGET_PITCH_DEG 3.0f
#define DEMO3AX_TARGET_YAW_DEG 165.0f
/*=== Hệ số đảo chiều ===*/
#define DEMO3AX_SIGN_ROLL (-1.0f)
#define DEMO3AX_SIGN_PITCH (1.0f)
#define DEMO3AX_SIGN_YAW (1.0f)

/*=== PID vòng ngoài IMU Stabilization (Outer loop: e_rot [rad] → vel [rad/s])
 * ===*/
/* Bắt đầu với Kp thấp, Ki=0, Kd=0 để tránh dao động; tăng dần khi test */
#define IMU_STAB_KP_ROLL 4.0f
#define IMU_STAB_KI_ROLL 0.3f
#define IMU_STAB_KD_ROLL 0.05f
#define IMU_STAB_VEL_LIMIT_ROLL 3.0f /* [rad/s] */

#define IMU_STAB_KP_PITCH 4.0f
#define IMU_STAB_KI_PITCH 0.3f
#define IMU_STAB_KD_PITCH 0.05f
#define IMU_STAB_VEL_LIMIT_PITCH 3.0f /* [rad/s] */

#define IMU_STAB_KP_YAW 3.0f
#define IMU_STAB_KI_YAW 0.2f
#define IMU_STAB_KD_YAW 0.0f
#define IMU_STAB_VEL_LIMIT_YAW 2.0f /* [rad/s] */

/* Hệ số Gyro Feedforward: cộng trực tiếp vận tốc góc đo được vào lệnh motor */
/* để giảm lag phản hồi. Bắt đầu từ 0.0f rồi tăng dần khi cần */
#define IMU_STAB_GYRO_FF_GAIN 0.0f

/* Ngưỡng raw SBUS để nhận biết CH5 đang ở vị trí HIGH (toggle) */
#define SBUS_CH5_HIGH_THRESH 1200U

/*=== Cấu hình phần cứng mạch dòng ===*/
#define GAIN_DRV 10.0f
#define SHUNT_RES 0.005f
#define VOLTAGE_LIMIT 1.5f
#define PWM_PERIOD 4249.0f
#define MOTOR_POLE_PAIRS 7

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f
#define DEG2RAD(d) ((d) * DEG_TO_RAD)

/*=== [3AXIS_FOLLOW_IMU] Vị trí IMU cố định cần giữ (do người dùng đo được)
 * ===*/
/* Các giá trị này là góc Euler đo được từ Mahony khi gimbal cân bằng đồng thời
 */
/* Đưa vào dưới dạng Euler (deg) để Quaternion_FromEuler() xử lý */
#define F3AX_TARGET_ROLL_DEG   0.92f    /* AHRS R đo được khi gimbal cân bằng */
#define F3AX_TARGET_PITCH_DEG  8.69f    /* AHRS P đo được khi gimbal cân bằng */
#define F3AX_TARGET_YAW_DEG   22.89f   /* AHRS Y đo được khi gimbal cân bằng */

/* Cách tiếp cận đơn giản hơn: Khởi động rồi chốt q_target = q_meas ngay lúc bắt
 * đầu */
/* (có thể dùng thay cho Euler target cứng). Xem giải thích bên dưới. */

/*=== [3AXIS_FOLLOW_IMU] PID vòng ngoài (e_rot [rad] → vel [rad/s]) ===*/
/* Giữ nguyên các thông số tương tự PROGRAM_MODE_MAIN IMU_STAB */
#define F3AX_KP_ROLL 4.0f
#define F3AX_KI_ROLL 0.3f
#define F3AX_KD_ROLL 0.05f
#define F3AX_VEL_LIMIT_ROLL 3.0f /* [rad/s] */

#define F3AX_KP_PITCH 4.0f
#define F3AX_KI_PITCH 0.3f
#define F3AX_KD_PITCH 0.05f
#define F3AX_VEL_LIMIT_PITCH 3.0f /* [rad/s] */

#define F3AX_KP_YAW 3.0f
#define F3AX_KI_YAW 0.2f
#define F3AX_KD_YAW 0.0f
#define F3AX_VEL_LIMIT_YAW 2.0f /* [rad/s] */

#define F3AX_GYRO_FF_GAIN 0.0f /* Feedforward Gyro, bắt đầu từ 0 */

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
SBUS_Mapping_Handle_t sbus_map; /* Mapping SBUS raw → target angle gimbal */

volatile float target_pitch_angle = 0.0f;
volatile float target_roll_angle = 0.0f;
volatile float target_yaw_angle = 0.0f;

/* === Trạng thái máy trạng thái của gimbal === */
typedef enum {
  GIMBAL_STATE_HOMING = 0, /* Đang tự động quay về vị trí home */
  GIMBAL_STATE_SBUS, /* Nhận lệnh SBUS, điều khiển encoder position */
  GIMBAL_STATE_IMU_STAB, /* Giữ hướng theo IMU (Cascade: IMU outer + Encoder
                            inner) */
} GimbalState_t;

GimbalState_t g_gimbal_state = GIMBAL_STATE_HOMING;

/* === Biến cho chế độ IMU Stabilization === */
/* Velocity setpoint do PID vòng ngoài (IMU) xuất ra, được đọc trong TIM7 ISR */
volatile float imu_stab_vel_roll = 0.0f;
volatile float imu_stab_vel_pitch = 0.0f;
volatile float imu_stab_vel_yaw = 0.0f;

/* Trạng thái trước của CH5 để phát hiện cạnh lên (toggle) */
static uint8_t sbus_ch5_prev = 0;

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
  uint32_t last_print_time = HAL_GetTick();
  uint32_t home_start_tick = HAL_GetTick();

#if (PROGRAM_MODE == PROGRAM_MODE_MAIN)

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

  /*=== Khởi tạo PID vòng ngoài IMU Stabilization (Outer loop) ===*/
  /* Dùng param từ #define để dễ tuning mà không cần tìm sâu vào code */
  PID_Init(&pid_3ax_roll_pos, IMU_STAB_KP_ROLL, IMU_STAB_KI_ROLL,
           IMU_STAB_KD_ROLL, -IMU_STAB_VEL_LIMIT_ROLL, IMU_STAB_VEL_LIMIT_ROLL);
  PID_Init(&pid_3ax_pitch_pos, IMU_STAB_KP_PITCH, IMU_STAB_KI_PITCH,
           IMU_STAB_KD_PITCH, -IMU_STAB_VEL_LIMIT_PITCH,
           IMU_STAB_VEL_LIMIT_PITCH);
  PID_Init(&pid_3ax_yaw_pos, IMU_STAB_KP_YAW, IMU_STAB_KI_YAW, IMU_STAB_KD_YAW,
           -IMU_STAB_VEL_LIMIT_YAW, IMU_STAB_VEL_LIMIT_YAW);

  ICM42688_Status_t icm_3ax_status =
      ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_3ax_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[DEMO_3AXIS] ICM42688 OK! Dang hieu chuan Gyro Bias (giu im "
           "1s)...\r\n");
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

    printf("[DEMO_3AXIS] Huong muc tieu co dinh: Roll=%.1f, Pitch=%.1f, "
           "Yaw=%.1f (deg)\r\n",
           DEMO3AX_TARGET_ROLL_DEG, DEMO3AX_TARGET_PITCH_DEG,
           DEMO3AX_TARGET_YAW_DEG);
    printf("[DEMO_3AXIS] Quat muc tieu: w=%.3f x=%.3f y=%.3f z=%.3f\r\n",
           q_demo3_target.q0, q_demo3_target.q1, q_demo3_target.q2,
           q_demo3_target.q3);
  } else {
    icm_init_ok = 0;
    printf("[DEMO_3AXIS] LOI khoi tao ICM42688! Code: %d\r\n", icm_3ax_status);
  }

  HAL_TIM_Base_Start_IT(&htim7); // TIMER7 chạy FOC
  SBUS_Status_t sbus_init_ret = SBUS_Init(&sbus_rx, &huart1);
  if (sbus_init_ret == SBUS_OK) {
    printf("[SBUS] Khoi tao thanh cong!\r\n");
  } else {
    printf("[SBUS] LOI khoi tao! Check USART1 DMA config.\r\n");
  }

  /* Đặt target về vị trí HOME — TIM7 ISR sẽ tự động drive motor đến đây */
  target_roll_angle = DEG2RAD(HOME_ROLL_DEG);
  target_pitch_angle = DEG2RAD(HOME_PITCH_DEG);
  target_yaw_angle = DEG2RAD(HOME_YAW_DEG);
  printf("[HOME] Bat dau Homing: Roll=%.1f Pitch=%.1f Yaw=%.1f (deg)\r\n",
         HOME_ROLL_DEG, HOME_PITCH_DEG, HOME_YAW_DEG);
#elif (PROGRAM_MODE == PROGRAM_MODE_IMU_TEST)
  /* Phai bat TIM6 de trigger doc IMU qua DMA */
  HAL_TIM_Base_Start_IT(&htim6);
  ICM42688_Status_t icm_3ax_status = ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_3ax_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[DEMO_3AXIS] ICM42688 OK! Dang hieu chuan Gyro Bias (giu im 1s)...\r\n");
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[DEMO_3AXIS] Bias: X=%.2f Y=%.2f Z=%.2f dps\r\n",
           imu_payload.gyro_bias_x, imu_payload.gyro_bias_y, imu_payload.gyro_bias_z);
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
    printf("[DEMO_3AXIS] Cho Mahony AHRS on định (2s)...\r\n");
    HAL_Delay(2000);

    Quaternion_FromEuler(DEG2RAD(DEMO3AX_TARGET_ROLL_DEG),
                         DEG2RAD(DEMO3AX_TARGET_PITCH_DEG),
                         DEG2RAD(DEMO3AX_TARGET_YAW_DEG), &q_demo3_target);

    printf("[DEMO_3AXIS] Huong muc tieu co dinh: Roll=%.1f, Pitch=%.1f, Yaw=%.1f (deg)\r\n",
           DEMO3AX_TARGET_ROLL_DEG, DEMO3AX_TARGET_PITCH_DEG, DEMO3AX_TARGET_YAW_DEG);
    printf("[DEMO_3AXIS] Quat muc tieu: w=%.3f x=%.3f y=%.3f z=%.3f\r\n",
           q_demo3_target.q0, q_demo3_target.q1, q_demo3_target.q2, q_demo3_target.q3);
  } else {
    icm_init_ok = 0;
    printf("[DEMO_3AXIS] LOI khoi tao ICM42688! Code: %d\r\n", icm_3ax_status);
  }

#elif (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
  /* ================================================================
   * PROGRAM_MODE_3AXIS_FOLLOW_IMU: Giữ Gimbal cân bằng tại 1 hướng
   * cố định trong không gian 3D (Cascade: IMU Outer + Encoder Inner)
   * ================================================================ */

  /* --- Phần cứng --- */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /* Encoder */
  AS5048A_Init(&pitch_enc, &hspi1, ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin);
  AS5048A_Init(&roll_enc, &hspi1, ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin);
  AS5048A_Init(&yaw_enc, &hspi1, ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin);
  HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim6); /* TIM6: doc encoder + trigger IMU DMA */
  HAL_Delay(100);

  /* Motor FOC Pitch */
  FOC_Init(&foc_pitch, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_pitch, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_pitch, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_pitch, 0.96f);

  /* Motor FOC Roll */
  FOC_Init(&foc_roll, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  FOC_SetPID_POS(&foc_roll, 8.0f, 0.4f, 0.0f, -4.0f, 4.0f);
  FOC_SetPID_VEL(&foc_roll, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_roll, 0.96f);

  /* Motor FOC Yaw */
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

  /* Align + Calibrate */
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

  /* --- PID vòng ngoài IMU --- */
  PID_Init(&pid_3ax_roll_pos, F3AX_KP_ROLL, F3AX_KI_ROLL, F3AX_KD_ROLL,
           -F3AX_VEL_LIMIT_ROLL, F3AX_VEL_LIMIT_ROLL);
  PID_Init(&pid_3ax_pitch_pos, F3AX_KP_PITCH, F3AX_KI_PITCH, F3AX_KD_PITCH,
           -F3AX_VEL_LIMIT_PITCH, F3AX_VEL_LIMIT_PITCH);
  PID_Init(&pid_3ax_yaw_pos, F3AX_KP_YAW, F3AX_KI_YAW, F3AX_KD_YAW,
           -F3AX_VEL_LIMIT_YAW, F3AX_VEL_LIMIT_YAW);

  /* --- Khởi tạo IMU ICM42688 --- */
  ICM42688_Status_t icm_f3ax_status =
      ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_f3ax_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[F3AX] ICM42688 OK! Calibrating Gyro Bias (giu im ~1s)...\r\n");
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[F3AX] Bias: X=%.3f Y=%.3f Z=%.3f dps\r\n", imu_payload.gyro_bias_x,
           imu_payload.gyro_bias_y, imu_payload.gyro_bias_z);

    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
    printf("[F3AX] Cho Mahony on dinh (2s)...\r\n");
    HAL_Delay(2000);

    /* === Đặt hướng mục tiêu bằng góc Euler đo được (AHRS R/P/Y) ===
     * Dùng Quaternion_FromEuler để chuyển Euler -> Quaternion.
     * Nếu muốn chốt theo hướng khởi động: thay bằng q_target_3d = q_meas. */
    Quaternion_FromEuler(DEG2RAD(F3AX_TARGET_ROLL_DEG),
                         DEG2RAD(F3AX_TARGET_PITCH_DEG),
                         DEG2RAD(F3AX_TARGET_YAW_DEG),
                         &q_target_3d);

    printf("[F3AX] Target Euler: R=%.2f P=%.2f Y=%.2f (deg)\r\n",
           F3AX_TARGET_ROLL_DEG, F3AX_TARGET_PITCH_DEG, F3AX_TARGET_YAW_DEG);
    printf("[F3AX] Quat target: w=%.3f x=%.3f y=%.3f z=%.3f\r\n",
           q_target_3d.q0, q_target_3d.q1, q_target_3d.q2, q_target_3d.q3);
    printf("[F3AX] AHRS hien tai: R=%.2f P=%.2f Y=%.2f (deg)\r\n",
           mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG,
           mahony_imu.yaw * RAD_TO_DEG);
  } else {
    icm_init_ok = 0;
    printf("[F3AX] LOI khoi tao ICM42688! Code: %d\r\n", icm_f3ax_status);
  }

  HAL_TIM_Base_Start_IT(&htim7); /* TIM7: chay FOC velocity loop */
  printf("[F3AX] He thong san sang! Gimbal dang giu huong co dinh.\r\n");

#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {
#if (PROGRAM_MODE == PROGRAM_MODE_MAIN)
    uint32_t now = HAL_GetTick();

    /* Luon gọi SBUS_Process để DMA ring buffer luôn được đọc */
    SBUS_Status_t sbus_st = SBUS_Process(&sbus_rx);

    /* === STATE MACHINE === */
    if (g_gimbal_state == GIMBAL_STATE_HOMING) {
      /* Kiểm tra từng trục đã về home chưa (dùng angle_deg từ encoder) */
      float err_roll = roll_enc.angle_deg - HOME_ROLL_DEG;
      float err_pitch = pitch_enc.angle_deg - HOME_PITCH_DEG;
      float err_yaw = yaw_enc.angle_deg - HOME_YAW_DEG;
      if (err_roll < 0.0f)
        err_roll = -err_roll;
      if (err_pitch < 0.0f)
        err_pitch = -err_pitch;
      if (err_yaw < 0.0f)
        err_yaw = -err_yaw;

      uint8_t homed = (err_roll <= HOME_TOL_DEG) &&
                      (err_pitch <= HOME_TOL_DEG) && (err_yaw <= HOME_TOL_DEG);
      uint8_t timeout = ((now - home_start_tick) >= HOME_TIMEOUT_MS);

      if (homed || timeout) {
        /* Chuyển sang chế độ SBUS, khởi đầu từ vị trí encoder hiện tại */
        SBUS_Mapping_Init(&sbus_map, &target_roll_angle, &target_pitch_angle,
                          &target_yaw_angle, roll_enc.angle_rad,
                          pitch_enc.angle_rad, yaw_enc.angle_rad);
        g_gimbal_state = GIMBAL_STATE_SBUS;
        if (homed) {
          printf("[HOME] Homing HOAN THANH! Chuyen sang che do SBUS.\r\n");
        } else {
          printf("[HOME] TIMEOUT! Chuyen sang che do SBUS (eR=%.1f eP=%.1f "
                 "eY=%.1f).\r\n",
                 err_roll, err_pitch, err_yaw);
        }
      }
    } else {
      /* --- Xử lý toggle CH5: bật/tắt chế độ IMU Stabilization --- */
      /* Chỉ phát hiện khi SBUS đang hoạt động bình thường */
      if (sbus_st == SBUS_OK && icm_init_ok) {
        uint16_t ch5_raw = 0;
        SBUS_GetChannel(&sbus_rx, 5, &ch5_raw);
        uint8_t ch5_now = (ch5_raw > SBUS_CH5_HIGH_THRESH) ? 1U : 0U;

        /* Phát hiện cạnh lên (LOW→HIGH): toggle chế độ */
        if (ch5_now && !sbus_ch5_prev) {
          if (g_gimbal_state == GIMBAL_STATE_SBUS) {
            /* Chuyển sang IMU_STAB: chốt q_target_3d tại hướng hiện tại của IMU
             */
            /* → Gimbal không nhảy đột ngột khi vừa bật */
            Quaternion_t q_now = {mahony_imu.q0, mahony_imu.q1, mahony_imu.q2,
                                  mahony_imu.q3};
            q_target_3d = q_now;
            /* Reset PID vòng ngoài để tránh windup tích lũy từ trước */
            PID_Reset(&pid_3ax_roll_pos);
            PID_Reset(&pid_3ax_pitch_pos);
            PID_Reset(&pid_3ax_yaw_pos);
            g_gimbal_state = GIMBAL_STATE_IMU_STAB;
            printf("[STAB] Bat che do IMU Stabilization!\r\n");
          } else if (g_gimbal_state == GIMBAL_STATE_IMU_STAB) {
            /* Chuyển về SBUS: lấy góc encoder hiện tại làm target để không giật
             */
            target_roll_angle = roll_enc.angle_rad;
            target_pitch_angle = pitch_enc.angle_rad;
            target_yaw_angle = yaw_enc.angle_rad;
            /* Reset velocity output về 0 */
            imu_stab_vel_roll = 0.0f;
            imu_stab_vel_pitch = 0.0f;
            imu_stab_vel_yaw = 0.0f;
            g_gimbal_state = GIMBAL_STATE_SBUS;
            printf("[STAB] Tat IMU Stabilization, chuyen ve SBUS.\r\n");
          }
        }
        sbus_ch5_prev = ch5_now;
      }

      /* --- Cập nhật lệnh điều khiển tùy chế độ --- */
      if (g_gimbal_state == GIMBAL_STATE_SBUS) {
        /* Chế độ cũ: SBUS điều khiển encoder position */
        SBUS_Mapping_Update(&sbus_map, &sbus_rx, sbus_st);
      } else if (g_gimbal_state == GIMBAL_STATE_IMU_STAB) {
        /* Chế độ IMU: Tính velocity setpoint từ PID vòng ngoài (e_rot) */
        /* Đã dời PID_Update vào ngắt HAL_SPI_TxRxCpltCallback để chạy đúng 1kHz (dt=0.001s). */
        /* Ở đây không gọi hàm PID_Update nữa để tránh sai số tần số. */
      }
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (now - last_print_time >= 100) {
      last_print_time = now;

      if (g_gimbal_state == GIMBAL_STATE_HOMING) {
        /* In tiến trình homing */
        printf("[HOMING] Roll:%6.1f→%.1f | Pitch:%6.1f→%.1f | Yaw:%6.1f→%.1f "
               "(deg)\r\n",
               roll_enc.angle_deg, HOME_ROLL_DEG, pitch_enc.angle_deg,
               HOME_PITCH_DEG, yaw_enc.angle_deg, HOME_YAW_DEG);
      } else if (g_gimbal_state == GIMBAL_STATE_IMU_STAB) {
        /* In telemetry chế độ IMU Stabilization */
        printf("[STAB]   AHRS R:%6.2f P:%6.2f Y:%6.2f (deg)\r\n",
               mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG,
               mahony_imu.yaw * RAD_TO_DEG);
        printf("[STAB]   eRot X:%6.3f Y:%6.3f Z:%6.3f (rad) | Vel R:%5.2f "
               "P:%5.2f Y:%5.2f (rad/s)\r\n",
               e_rot[0], e_rot[1], e_rot[2], imu_stab_vel_roll,
               imu_stab_vel_pitch, imu_stab_vel_yaw);
      } else {
        /* In telemetry chế độ SBUS */
        if (sbus_st == SBUS_OK) {
          uint16_t ch1, ch2, ch4;
          SBUS_GetChannel(&sbus_rx, 1, &ch1);
          SBUS_GetChannel(&sbus_rx, 2, &ch2);
          SBUS_GetChannel(&sbus_rx, 4, &ch4);
          const char *cmd_str[] = {"NEG", "HOLD", "POS"};
          printf("[TARGET] Roll:%6.1f | Pitch:%6.1f | Yaw:%6.1f (deg)\r\n",
                 target_roll_angle * 57.2957795f,
                 target_pitch_angle * 57.2957795f,
                 target_yaw_angle * 57.2957795f);
          printf("[ENC]    Roll:%6.1f | Pitch:%6.1f | Yaw:%6.1f (deg)\r\n",
                 roll_enc.angle_deg, pitch_enc.angle_deg, yaw_enc.angle_deg);
          printf("[SBUS]   CH1=%4u(%s) CH2=%4u(%s) CH4=%4u(%s)\r\n", ch1,
                 cmd_str[sbus_map.cmd_roll + 1], ch2,
                 cmd_str[sbus_map.cmd_pitch + 1], ch4,
                 cmd_str[sbus_map.cmd_yaw + 1]);
        } else if (sbus_st == SBUS_FAILSAFE) {
          printf("[SBUS] CANH BAO: FAILSAFE dang active!\r\n");
        } else if (sbus_st == SBUS_FRAME_LOST) {
          printf("[SBUS] CANH BAO: Frame Lost!\r\n");
        } else if (sbus_st == SBUS_TIMEOUT) {
          printf("[SBUS] LOI: TIMEOUT! Mat tin hieu Receiver (> %dms).\r\n",
                 SBUS_TIMEOUT_MS);
        } else {
          printf("[SBUS] Dang cho frame dau tien...\r\n");
        }
      }
    }

#elif (PROGRAM_MODE == PROGRAM_MODE_IMU_TEST)
    uint32_t now = HAL_GetTick();
    if (now - last_print_time >= 100) {
      last_print_time = now;
      printf("[AHRS]  R:%7.2f P:%7.2f Y:%7.2f (deg) | [GYRO]  Gx:%7.2f Gy:%7.2f Gz:%7.2f (dps) | [ACCEL] Ax:%7.3f Ay:%7.3f Az:%7.3f (g) \r\n",
             mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG, mahony_imu.yaw * RAD_TO_DEG, 
             imu_payload.gyro_x_dps, imu_payload.gyro_y_dps, imu_payload.gyro_z_dps, 
             imu_payload.accel_x_g, imu_payload.accel_y_g, imu_payload.accel_z_g);
      // printf("[TEMP]  %.1f degC\r\n", imu_payload.temp_c);
      // printf("---\r\n");
    }

#elif (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
    /* ================================================================
     * Cascade Outer Loop:
     * PID vòng ngoài đã được chuyển vào DMA ISR của SPI3
     * để đảm bảo chạy ở tần số ổn định 1kHz (dt = 0.001s).
     * ================================================================ */
    uint32_t now = HAL_GetTick(); 

    /* In telemetry 100ms một lần */
    if (now - last_print_time >= 100) {
      last_print_time = now;
      // printf("[F3AX]  AHRS R:%6.2f P:%6.2f Y:%6.2f (deg)\r\n",
      //        mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG, mahony_imu.yaw * RAD_TO_DEG);
      // printf("[F3AX]  eRot X:%6.3f Y:%6.3f Z:%6.3f (rad) | Vel R:%5.2f P:%5.2f Y:%5.2f\r\n",
      //        e_rot[0], e_rot[1], e_rot[2], imu_stab_vel_roll, imu_stab_vel_pitch, imu_stab_vel_yaw);
      printf("AHRS R:%6.2f P:%6.2f Y:%6.2f (deg)| [ENC] Roll:%6.1f Pitch:%6.1f Yaw:%6.1f (deg) \r\n",
      mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG, mahony_imu.yaw * RAD_TO_DEG,
      roll_enc.angle_deg, pitch_enc.angle_deg, yaw_enc.angle_deg);
    }
#endif
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
static uint8_t icm_tx_buf[15] = {
    0x1D | 0x80, 0}; /* 0x1D | 0x80: Read command bắt đầu từ TEMP_DATA1 */
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

    // Remap 
    float gimbal_gx = imu_payload.gyro_y_dps;
    float gimbal_gy = imu_payload.gyro_x_dps;
    float gimbal_gz = imu_payload.gyro_z_dps;

    float gimbal_ax = imu_payload.accel_y_g;
    float gimbal_ay = imu_payload.accel_x_g;
    float gimbal_az = imu_payload.accel_z_g;

    /* Cập nhật Mahony 3D AHRS ngay tại ngắt DMA 2kHz (dt = 0.0005s) */
    Mahony_Update(&mahony_imu, gimbal_gx * DEG_TO_RAD,
                  gimbal_gy * DEG_TO_RAD,
                  gimbal_gz * DEG_TO_RAD, gimbal_ax,
                  gimbal_ay, gimbal_az, 0.0005f);

    Quaternion_t q_meas_3d = {mahony_imu.q0, mahony_imu.q1, mahony_imu.q2,
                              mahony_imu.q3};
    Quaternion_ComputeError(&q_target_3d, &q_meas_3d, e_rot);

    /* --- Cập nhật PID vòng ngoài (IMU Stabilization) --- */
    /* Chạy ngay sau khi có e_rot mới để đảm bảo dt = 0.0005s chính xác */
#if (PROGRAM_MODE == PROGRAM_MODE_MAIN)
    if (g_gimbal_state == GIMBAL_STATE_IMU_STAB) {
      // /* Lúc này e_rot[0] (X của AHRS) đã chuẩn là Roll của Gimbal */
      // float vel_r = PID_Update(&pid_3ax_roll_pos, e_rot[0], 0.0005f);
      // float vel_p = PID_Update(&pid_3ax_pitch_pos, e_rot[1], 0.0005f);
      // float vel_y = PID_Update(&pid_3ax_yaw_pos, e_rot[2], 0.0005f);

      // vel_r += gimbal_gx * DEG_TO_RAD * IMU_STAB_GYRO_FF_GAIN;
      // vel_p += gimbal_gy * DEG_TO_RAD * IMU_STAB_GYRO_FF_GAIN;
      // vel_y += gimbal_gz * DEG_TO_RAD * IMU_STAB_GYRO_FF_GAIN;

      // imu_stab_vel_roll = vel_r;
      // imu_stab_vel_pitch = vel_p;
      // imu_stab_vel_yaw = vel_y;
    }
#elif (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
    if (icm_init_ok) {
      // /* Lúc này e_rot[0] (X của AHRS) đã chuẩn là Roll của Gimbal */
      // float vel_r = PID_Update(&pid_3ax_roll_pos, e_rot[0], 0.0005f);
      // float vel_p = PID_Update(&pid_3ax_pitch_pos, e_rot[1], 0.0005f);
      // float vel_y = PID_Update(&pid_3ax_yaw_pos, e_rot[2], 0.0005f);

      // vel_r += gimbal_gx * DEG_TO_RAD * F3AX_GYRO_FF_GAIN;
      // vel_p += gimbal_gy * DEG_TO_RAD * F3AX_GYRO_FF_GAIN;
      // vel_y += gimbal_gz * DEG_TO_RAD * F3AX_GYRO_FF_GAIN;

      // imu_stab_vel_roll = vel_r;
      // imu_stab_vel_pitch = vel_p;
      // imu_stab_vel_yaw = vel_y;
    }
#endif
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
      if (HAL_SPI_TransmitReceive_DMA(&hspi3, icm_tx_buf, icm_rx_buf, 15) != HAL_OK) {
        icm_dma_busy = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      }
    }
  } else if (htim->Instance == TIM7) {
#if (PROGRAM_MODE == PROGRAM_MODE_3AXIS_FOLLOW_IMU)
    // /* ================================================================
    //  * 3AXIS_FOLLOW_IMU: Vòng trong (1kHz) dùng FOC_VelocityLoop
    //  * Velocity setpoint được ghi từ outer loop trong while(1)
    //  * ================================================================ */
    // /* Motor Roll vật lý điều khiển trục pitch IMU và ngược lại:
    //  * đổi chéo foc_roll <-> foc_pitch vì kiến trúc cơ khí quay vuông góc */
    // FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, imu_stab_vel_roll);
    // FOC_VelocityLoop(&foc_roll,  roll_enc.angle_rad,  imu_stab_vel_pitch);
    // FOC_VelocityLoop(&foc_yaw,   yaw_enc.angle_rad,   imu_stab_vel_yaw);

    /* 1. Lấy Quaternion đo được từ Mahony (cập nhật liên tục từ DMA ISR) */
    Quaternion_t q_meas = {mahony_imu.q0, mahony_imu.q1, mahony_imu.q2, mahony_imu.q3};

    /* 2. Tính vector sai số góc 3D (không Gimbal Lock) */
    Quaternion_ComputeError(&q_demo3_target, &q_meas, e_rot);

    /* 3. Vòng ngoài: sai số góc → lệnh vận tốc (rad/s) cho từng trục */
    float vel_roll  = DEMO3AX_SIGN_ROLL  * PID_Update(&pid_3ax_roll_pos,  e_rot[1], foc_roll.Ts);
    float vel_pitch = DEMO3AX_SIGN_PITCH * PID_Update(&pid_3ax_pitch_pos, e_rot[0], foc_pitch.Ts);
    float vel_yaw   = DEMO3AX_SIGN_YAW   * PID_Update(&pid_3ax_yaw_pos,   e_rot[2], foc_yaw.Ts);

    /* 4. Vòng trong: FOC_VelocityLoop đóng vòng tốc độ + điện áp (Encoder làm feedback) */
    FOC_VelocityLoop(&foc_roll,  roll_enc.angle_rad,  vel_roll);
    FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, vel_pitch);
    FOC_VelocityLoop(&foc_yaw,   yaw_enc.angle_rad,   vel_yaw);
#else
    /* ================================================================
     * PROGRAM_MODE_MAIN: Tach hai nhánh SBUS/HOMING va IMU_STAB
     * ================================================================ */
    if (g_gimbal_state == GIMBAL_STATE_IMU_STAB) {
      /* Đổi chéo foc_roll <-> foc_pitch vì kiến trúc cơ khí quay vuông góc */
      FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, imu_stab_vel_roll);
      FOC_VelocityLoop(&foc_roll,  roll_enc.angle_rad,  imu_stab_vel_pitch);
      FOC_VelocityLoop(&foc_yaw,   yaw_enc.angle_rad,   imu_stab_vel_yaw);
    } else {
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

      FOC_PositionLoop(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
      FOC_PositionLoop(&foc_roll, roll_enc.angle_rad, target_roll_angle);
      FOC_PositionLoop(&foc_yaw, yaw_enc.angle_rad, target_yaw_angle);
    }
#endif
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
