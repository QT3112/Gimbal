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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEST_MODE_VELOCITY 1
#define TEST_MODE_POSITION 2
#define TEST_MODE_CURRENT 3
#define TEST_MODE_ICM42688 4
#define TEST_MODE_DEMO_1AXIS 5
#define TEST_MODE_READ_ENCODER 6
#define TEST_MODE_DEMO_3AXIS 7

#define TEST_MODE TEST_MODE_DEMO_3AXIS

/* === Giới hạn góc === */
#define PITCH_MIN_DEG 15.0f
#define PITCH_MAX_DEG 280.0f
#define ROLL_MIN_DEG 90.0f
#define ROLL_MAX_DEG 265.0f
#define YAW_MIN_DEG 65.0f
#define YAW_MAX_DEG 255.0f

#define PI 3.14159265359f
#define TWO_PI 6.28318530718f
#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f
#define DEG2RAD(d) ((d) * DEG_TO_RAD)

#define PITCH_SETPOINT_DEG 220.0f
#define ROLL_SETPOINT_DEG 185.0f
#define YAW_SETPOINT_DEG 180.0f

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

/* Biến lưu góc và vận tốc mục tiêu (rad & rad/s) */
volatile float target_pitch_angle = 0.0f;
volatile float target_roll_angle = 0.0f;
volatile float target_yaw_angle = 0.0f;
volatile float target_velocity_rad_s = 0.0f;
float test_angle_elec = 0.0f;

/* ===========================================================
 * Biến dành riêng cho TEST_MODE_DEMO_1AXIS (Gimbal 1 trục — Roll)
 * =========================================================== */
volatile float imu_roll_rad = 0.0f; /* Góc Roll từ Mahony AHRS [rad] */
volatile float imu_roll_vel_rad_s =
    0.0f; /* Vận tốc góc Roll từ Gyro X [rad/s] */
float gimbal_target_roll_rad =
    0.0f; /* Setpoint LOCK: góc được chốt khi vào mode */

/* ===========================================================
 * PID riêng biệt cho LOCK_MODE (TEST_MODE_DEMO_1AXIS)
 * Hoàn toàn độc lập với PID của các Encoder-based modes.
 * Được swap vào foc_pitch khi vào mode, khôi phục khi thoát.
 * =========================================================== */
PID_Handle_t pid_lock_pos;   /* Outer loop: IMU Roll angle → target_vel */
PID_Handle_t pid_lock_vel;   /* Inner loop: Gyro X [rad/s] → Vq */
PID_Handle_t pid_backup_pos; /* Lưu PID Position của Encoder mode */
PID_Handle_t pid_backup_vel; /* Lưu PID Velocity của Encoder mode */

/* ===========================================================
 * Biến dành riêng cho TEST_MODE_DEMO_3AXIS (Gimbal 3 trục — IMU Stabilization)
 * =========================================================== */
Quaternion_t
    q_demo3_target; /* Hướng mục tiêu: chốt khi vào mode, giữ cố định sau đó */

/* PID vòng ngoài: error góc (rad) → velo setpoint (rad/s) cho từng trục */
PID_Handle_t pid_3ax_roll_pos;
PID_Handle_t pid_3ax_pitch_pos;
PID_Handle_t pid_3ax_yaw_pos;

/* Hệ số chiều quay motor */
#define DEMO3AX_SIGN_ROLL (-1.0f)
#define DEMO3AX_SIGN_PITCH (1.0f)
#define DEMO3AX_SIGN_YAW (1.0f)

float pitch_offset_a = 0.0f;
float pitch_offset_b = 0.0f;

/* Biến lưu trữ raw ADC để debug */
volatile uint32_t log_raw_ia = 0;
volatile uint32_t log_raw_ib = 0;
volatile uint16_t log_enc_raw = 0; /* Debug: raw SPI data từ AS5048A */

/* Biến runtime điều khiển từ GUI (volatile vì được ghi từ ISR CDC) */
volatile uint8_t g_mode = TEST_MODE; /* Chế độ hiện tại: 1=VEL, 2=POS */

/* ===========================================================
 * Biến hiệu chỉnh offset zero-current ADC (chạy trong ISR)
 * Lấy mẫu khi motor đang dừng (Duty=50%, không có dòng).
 * =========================================================== */
volatile uint32_t cal_sum_a = 0;
volatile uint32_t cal_sum_b = 0;
volatile uint32_t cal_count = 0;
#define ADC_CAL_SAMPLES 2000 /* 2000 mẫu × 50μs/mẫu = 100ms hiệu chỉnh */

#define GAIN_DRV 10.0f
#define SHUNT_RES 0.005f
#define VOLTAGE_LIMIT 1.5f
#define PWM_PERIOD 4249.0f
#define MOTOR_POLE_PAIRS 7

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
  HAL_GPIO_WritePin(
      GPIOB, GPIO_PIN_1,
      GPIO_PIN_SET); // set enable chung cho các chứng năng cần thiết

  // Khởi tạo encoder
  AS5048A_Init(&pitch_enc, &hspi1, ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin);
  AS5048A_Init(&roll_enc, &hspi1, ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin);
  AS5048A_Init(&yaw_enc, &hspi1, ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin);

  HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(100); // Chờ lấy mẫu vài frame góc ban đầu từ AS5048A

  /* Khởi tạo FOC cho trục Pitch (TIMER 1)*/
  FOC_Init(&foc_pitch, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  FOC_SetPID_POS(&foc_pitch, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_pitch, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_pitch, 0.96f);

  /* Khởi tạo FOC cho trục Roll (TIMER 8) */
  FOC_Init(&foc_roll, &htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

  FOC_SetPID_POS(&foc_roll, 8.0f, 0.4f, 0.0f, -4.0f, 4.0f);
  FOC_SetPID_VEL(&foc_roll, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_roll, 0.96f);

  /* Khởi tạo FOC cho trục Yaw (TIMER 3) */
  FOC_Init(&foc_yaw, &htim3, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, VOLTAGE_LIMIT, 1.0f, 0.0005f,
           0.00005f);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  FOC_SetPID_POS(&foc_yaw, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_yaw, 0.4f, 5.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_yaw, 0.96f);

#if (TEST_MODE == TEST_MODE_POSITION)
  foc_pitch.enabled = 1;
  foc_roll.enabled = 1;
  foc_yaw.enabled = 1;

  /* Căn chỉnh trục D cho cả 3 motor (Dùng điện áp đủ lớn để ép chặt motor về 0)
   */
  FOC_AlignD(&foc_pitch, 1.0f);
  FOC_AlignD(&foc_roll, 1.0f);
  FOC_AlignD(&foc_yaw, 1.0f);
  HAL_Delay(1000);

  /* Lưu điểm Zero Offset cho cả 3 trục */
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  FOC_CalibrateAngle(&foc_roll, roll_enc.angle_rad);
  FOC_CalibrateAngle(&foc_yaw, yaw_enc.angle_rad);

  /* Khởi động an toàn FOC cho cả 3 trục */
  FOC_Start(&foc_pitch, pitch_enc.angle_rad);
  FOC_Start(&foc_roll, roll_enc.angle_rad);
  FOC_Start(&foc_yaw, yaw_enc.angle_rad);

  /* Chốt mục tiêu cả 3 trục */
  target_pitch_angle = DEG2RAD(PITCH_SETPOINT_DEG);
  target_roll_angle = DEG2RAD(ROLL_SETPOINT_DEG);
  target_yaw_angle = DEG2RAD(YAW_SETPOINT_DEG);

  HAL_TIM_Base_Start_IT(&htim7);

  /* Khởi tạo ICM42688 song song với FOC (chỉ để theo dõi, không điều khiển) */
  ICM42688_Status_t icm_pos_status =
      ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_pos_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf(
        "[POS_MODE] ICM42688 OK! Dang hieu chuan Gyro Bias (giu im 1s)...\r\n");
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[POS_MODE] Gyro Bias: X=%.2f Y=%.2f Z=%.2f dps\r\n",
           imu_payload.gyro_bias_x, imu_payload.gyro_bias_y,
           imu_payload.gyro_bias_z);
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
  } else {
    icm_init_ok = 0;
    printf("[POS_MODE] CANH BAO: ICM42688 loi khoi tao (code %d), chi dung "
           "Encoder!\r\n",
           icm_pos_status);
  }
#elif (TEST_MODE == TEST_MODE_VELOCITY)
  foc_pitch.enabled = 1;
  FOC_AlignD(&foc_pitch, 0.5f);
  HAL_Delay(1000);
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  foc_pitch.position_loop_enabled = 1;
  foc_pitch.velocity_loop_enabled = 1;
  foc_pitch.current_loop_enabled = 0;
  HAL_TIM_Base_Start_IT(&htim7);
#elif (TEST_MODE == TEST_MODE_CURRENT)
  foc_pitch.enabled = 1;
  FOC_AlignD(&foc_pitch, 0.5f);
  HAL_Delay(1000);
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  foc_pitch.position_loop_enabled = 0;
  foc_pitch.velocity_loop_enabled = 0;
  foc_pitch.current_loop_enabled = 1;
  HAL_ADCEx_InjectedStart_IT(&hadc1);
  // HAL_TIM_Base_Start_IT(&htim7);
#elif (TEST_MODE == TEST_MODE_ICM42688)
  /* Dừng FOC motor control khi test cảm biến ICM42688 */
  foc_pitch.enabled = 0;
  FOC_Stop(&foc_pitch);

  /* Khởi tạo cảm biến ICM-42688-P qua SPI3 DMA (CS: PB15 - IMU_PAYLOAD_CS) */
  ICM42688_Status_t icm_status =
      ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[ICM42688] Khoi tao thanh cong & bat UI LPF 50Hz! (WHO_AM_I = "
           "0x47)\r\n");
    printf("[ICM42688] Dang hieu chuan Gyro Bias (giu im cam bien 1s)...\r\n");
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[ICM42688] Bias hoan tat: X=%.2f, Y=%.2f, Z=%.2f dps\r\n",
           imu_payload.gyro_bias_x, imu_payload.gyro_bias_y,
           imu_payload.gyro_bias_z);

    /* Khởi tạo bộ lọc Mahony AHRS */
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
  } else {
    icm_init_ok = 0;
    printf("[ICM42688] Loi khoi tao! Status code: %d\r\n", icm_status);
  }

#elif (TEST_MODE == TEST_MODE_DEMO_3AXIS)
  /* ================================================================
   * DEMO_3AXIS: Gimbal 3 trục — IMU Stabilization (giữ hướng payload)
   * Luồng: ICM42688 (SPI3 DMA) → Mahony AHRS → Quaternion Error
   *          → PID outer (3 trục) → FOC_VelocityLoop x3 → SVPWM
   * ================================================================ */

  /* 1. Enable và căn chỉnh FOC cả 3 trục (giống POSITION mode) */
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

  /* 2. Khởi tạo PID vòng ngoài cho cả 3 trục */
  PID_Init(&pid_3ax_roll_pos, 4.0f, 0.3f, 0.05f, -3.0f, 3.0f);
  PID_Init(&pid_3ax_pitch_pos, 4.0f, 0.3f, 0.05f, -3.0f, 3.0f);
  PID_Init(&pid_3ax_yaw_pos, 3.0f, 0.2f, 0.0f, -2.0f, 2.0f);

  /* 3. Khởi tạo ICM42688 + Mahony AHRS */
  ICM42688_Status_t icm_3ax_status =
      ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_3ax_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[DEMO_3AXIS] ICM42688 OK! Dang hieu chuan Gyro Bias\r\n");
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[DEMO_3AXIS] Bias: X=%.2f Y=%.2f Z=%.2f dps\r\n",
           imu_payload.gyro_bias_x, imu_payload.gyro_bias_y,
           imu_payload.gyro_bias_z);
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);

    /* 4. Chờ Mahony converge (~2s) rồi chốt hướng mục tiêu */
    printf("[DEMO_3AXIS] Cho Mahony AHRS on định (2s)...\r\n");
    HAL_Delay(2000);
    q_demo3_target.q0 = mahony_imu.q0;
    q_demo3_target.q1 = mahony_imu.q1;
    q_demo3_target.q2 = mahony_imu.q2;
    q_demo3_target.q3 = mahony_imu.q3;
    printf("[DEMO_3AXIS] Hướng mục tiêu chot: w=%.3f x=%.3f y=%.3f z=%.3f\r\n",
           q_demo3_target.q0, q_demo3_target.q1, q_demo3_target.q2,
           q_demo3_target.q3);
  } else {
    icm_init_ok = 0;
    printf("[DEMO_3AXIS] LOI khoi tao ICM42688! Code: %d\r\n", icm_3ax_status);
  }

  /* 5. Bật control loop 2kHz (TIM7) */
  HAL_TIM_Base_Start_IT(&htim7);
  printf("[DEMO_3AXIS] Kich hoat! 3-Axis Stabilization dang chay...\r\n");
#elif (TEST_MODE == TEST_MODE_READ_ENCODER)
  printf("--- BẮT ĐẦU TEST_MODE_READ_ENCODER (3 Trục SPI Pipeline) ---\r\n");
  /* Tắt hoàn toàn motor, chỉ đọc SPI trong ngắt TIM6 */
  foc_pitch.enabled = 0;
#endif

#if (TEST_MODE == TEST_MODE_POSITION || TEST_MODE == TEST_MODE_VELOCITY)
  uint32_t last_step_time = HAL_GetTick();
  uint8_t step_state = 0;
#endif

  /* Khởi tạo giá trị mặc định: GUI sẽ điều khiển từ đây */
  uint32_t last_print_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint32_t now = HAL_GetTick();

#if (TEST_MODE == TEST_MODE_POSITION)
    /* --- Bỏ qua kịch bản tự động xoay góc (Step Response) để giữ nguyên 1 góc
     * cố định --- */
    /*
    if (now - last_step_time >= 4000) {
      last_step_time = now;
      step_state = (step_state + 1) % 4;
      if (step_state == 0) {
        target_pitch_angle = 0.0f;
        target_roll_angle = 0.0f;
      } else if (step_state == 1) {
        target_pitch_angle = 1.5707963f;
        target_roll_angle = 1.5707963f;
      } ...
    }
    */

    /* Quy đổi góc mục tiêu ra độ để hiển thị */
    float target_pitch_deg = target_pitch_angle * RAD_TO_DEG;
    float target_roll_deg = target_roll_angle * RAD_TO_DEG;
    float target_yaw_deg = target_yaw_angle * RAD_TO_DEG;

    float err_pitch_deg = target_pitch_deg - pitch_enc.angle_deg;
    float err_roll_deg = target_roll_deg - roll_enc.angle_deg;
    float err_yaw_deg = target_yaw_deg - yaw_enc.angle_deg;

    while (err_pitch_deg > 180.0f)
      err_pitch_deg -= 360.0f;
    while (err_pitch_deg < -180.0f)
      err_pitch_deg += 360.0f;
    while (err_roll_deg > 180.0f)
      err_roll_deg -= 360.0f;
    while (err_roll_deg < -180.0f)
      err_roll_deg += 360.0f;
    while (err_yaw_deg > 180.0f)
      err_yaw_deg -= 360.0f;
    while (err_yaw_deg < -180.0f)
      err_yaw_deg += 360.0f;

    /* Print Telemetry (10Hz) - Encoder 3 trục */
    // printf("[P] Tgt:%5.1f Act:%5.1f Err:%5.1f Vq:%5.2f | "
    //        "[R] Tgt:%5.1f Act:%5.1f Err:%5.1f Vq:%5.2f | "
    //        "[Y] Tgt:%5.1f Act:%5.1f Err:%5.1f Vq:%5.2f\r\n",
    //        target_pitch_deg, pitch_enc.angle_deg, err_pitch_deg,
    //        foc_pitch.Vq_ref, target_roll_deg, roll_enc.angle_deg,
    //        err_roll_deg, foc_roll.Vq_ref, target_yaw_deg, yaw_enc.angle_deg,
    //        err_yaw_deg, foc_yaw.Vq_ref);

    /* Print Telemetry IMU (10Hz) - Mahony AHRS */
    if (icm_init_ok) {
      printf("[IMU] R:%5.1f P:%5.1f Y:%5.1f (deg) | Gx:%5.1f Gy:%5.1f Gz:%5.1f "
             "(dps) | Ax:%5.2f Ay:%5.2f Az:%5.2f (g)\r\n",
             mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG,
             mahony_imu.yaw * RAD_TO_DEG, imu_payload.gyro_x_dps,
             imu_payload.gyro_y_dps, imu_payload.gyro_z_dps,
             imu_payload.accel_x_g, imu_payload.accel_y_g,
             imu_payload.accel_z_g);
    }

#elif (TEST_MODE == TEST_MODE_VELOCITY)
    /* Kịch bản test step response vận tốc tự động mỗi 3 giây */
    if (now - last_step_time >= 3000) {
      last_step_time = now;
      step_state = (step_state + 1) % 3;
      if (step_state == 0) {
        target_velocity_rad_s = 3.0f; /* Quay thuận 3.0 rad/s (~28.6 RPM)
                                       */
      } else if (step_state == 1) {
        target_velocity_rad_s = -3.0f; /* Quay ngược -3.0 rad/s */
      } else {
        target_velocity_rad_s = 0.0f; /* Dừng 0 rad/s */
      }
    }

    /* Print Telemetry (10Hz) để giám sát đáp ứng tốc độ */
    printf("TargetVel: %.2f | VelFilt: %.2f | VelRaw: %.2f | Vq: %.2fV | Enc: "
           "%.2f deg\r\n",
           target_velocity_rad_s, foc_pitch.velocity_mech,
           foc_pitch.velocity_mech_raw, foc_pitch.Vq_ref, pitch_enc.angle_deg);
#elif (TEST_MODE == TEST_MODE_CURRENT)
    /* === VÒNG LẶP HỞ (OPEN LOOP) TEST CHIỀU DÒNG ĐIỆN === */
    /* Áp điện áp cố định Vq = +0.30V (bỏ qua PID để tránh rít/rung bão hòa) */
    foc_pitch.pid_d.integral = 0.0f;
    foc_pitch.pid_q.integral = 0.30f;

    printf("OPEN_LOOP | Vq_applied: +0.30V | Iq_meas: %.3f A | Id_meas: %.3f A "
           "| OffA: %lu | OffB: %lu | rawA: %lu | rawB: %lu \r\n",
           foc_pitch.I_dq.q, foc_pitch.I_dq.d, foc_pitch.adc_offset_a,
           foc_pitch.adc_offset_b, log_raw_ia, log_raw_ib);

#elif (TEST_MODE == TEST_MODE_ICM42688)
    if (now - last_print_time >= 100) {
      last_print_time = now;
      if (icm_init_ok) {
        float roll_deg = mahony_imu.roll * 57.2957795f;
        float pitch_deg = mahony_imu.pitch * 57.2957795f;
        float yaw_deg = mahony_imu.yaw * 57.2957795f;

        printf("[SPI3 DMA 3D Quat] Err_Rad: X=%6.3f Y=%6.3f Z=%6.3f | "
               "Quat[w=%.2f,x=%.2f,y=%.2f,z=%.2f] | AHRS[deg]: R=%5.1f P=%5.1f "
               "Y=%5.1f\r\n",
               e_rot[0], e_rot[1], e_rot[2], mahony_imu.q0, mahony_imu.q1,
               mahony_imu.q2, mahony_imu.q3, roll_deg, pitch_deg, yaw_deg);
      } else {
        printf("[ICM42688] Cam bien chua duoc khoi tao thanh cong!\r\n");
      }
    }
#elif (TEST_MODE == TEST_MODE_DEMO_1AXIS)
    /* Telemetry gimbal 1 trục Roll (10Hz) */
    if (now - last_print_time >= 100) {
      last_print_time = now;
      if (icm_init_ok) {
        printf("[DEMO_1AXIS] Roll: %6.2f deg | GyroX: %6.2f dps | Enc: "
               "%6.2f deg | Vq: %.3fV | Vel: %.2f rad/s | Raw: 0x%04X\r\n",
               imu_roll_rad * 57.2957795131f, imu_payload.gyro_x_dps,
               pitch_enc.angle_deg, foc_pitch.Vq_ref, foc_pitch.velocity_mech,
               log_enc_raw);
      } else {
        printf("[DEMO_1AXIS] IMU chua san sang!\r\n");
      }
    }
#elif (TEST_MODE == TEST_MODE_READ_ENCODER)
    /* In dữ liệu góc của cả 3 encoder ra UART (20Hz) */
    if (now - last_print_time >= 50) {
      last_print_time = now;
      printf("[ENC3D] Pitch: %6.2f | Roll: %6.2f | Yaw: %6.2f (deg)\r\n",
             pitch_enc.angle_deg, roll_enc.angle_deg, yaw_enc.angle_deg);
    }
#elif (TEST_MODE == TEST_MODE_DEMO_3AXIS)
    /* Telemetry gimbal 3 trục (10Hz) */
    if (now - last_print_time >= 100) {
      last_print_time = now;
      if (icm_init_ok) {
        /* Dòng 1: Vector sai số góc + lệnh vận tốc (debug PID) */
        printf("[3AX_ERR] eR:%6.3f eP:%6.3f eY:%6.3f | VR:%5.2f VP:%5.2f "
               "VY:%5.2f\r\n",
               e_rot[0], e_rot[1], e_rot[2], foc_roll.velocity_mech,
               foc_pitch.velocity_mech, foc_yaw.velocity_mech);
        /* Dòng 2: Trạng thái Vq (hiệu quả lực giữ của từng trục) */
        printf("[3AX_FOC] Vq_R:%5.2f Vq_P:%5.2f Vq_Y:%5.2f | AHRS R:%5.1f "
               "P:%5.1f Y:%5.1f (deg)\r\n",
               foc_roll.Vq_ref, foc_pitch.Vq_ref, foc_yaw.Vq_ref,
               mahony_imu.roll * RAD_TO_DEG, mahony_imu.pitch * RAD_TO_DEG,
               mahony_imu.yaw * RAD_TO_DEG);
      } else {
        printf("[DEMO_3AXIS] IMU chua san sang!\r\n");
      }
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

    /* Cập nhật biến Roll cho DEMO_1AXIS stabilization loop
     * imu_roll_rad     : Góc Roll đầu ra từ Mahony AHRS [rad]
     * imu_roll_vel_rad_s: Vận tốc góc Gyro_X [rad/s] — dùng cho inner velocity
     * loop */
    imu_roll_rad = -mahony_imu.roll;
    imu_roll_vel_rad_s = -imu_payload.gyro_x_dps * DEG_TO_RAD;
  }
}

/* Hàm bắt lỗi SPI: nếu có nhiễu gây Overrun, HAL sẽ gọi hàm này.
 * Phải reset cờ busy để TIM6 có thể gọi lại vòng mới */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    HAL_SPI_Abort(&hspi1); /* Ép SPI1 thoát khỏi trạng thái lỗi/bận */
    enc_busy = 0;
    enc_phase = 0;
    HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
  } else if (hspi->Instance == SPI3) {
    HAL_SPI_Abort(&hspi3);
    icm_dma_busy = 0;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  }
}

/* Hàm ngắt Timer định kỳ: TIM6 (1kHz I/O Trigger) & TIM7 (2kHz Central Control
 * Loop) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    /* CHUYỂN SANG POLLING: Đọc AS5048A trực tiếp trong ngắt TIM6 (tốn ~10us)
     * Để loại trừ hoàn toàn các lỗi do DMA bị kẹt, OVR error không được reset,
     * hoặc bộ nhớ không cập nhật. */
    if (!enc_busy) {
      enc_busy = 1;
      uint16_t rx_dummy;
      uint16_t rx_angle_pitch, rx_angle_roll, rx_angle_yaw;

      /* === Phase 0 (Pipeline): Gửi lệnh READ ANGLE cho cả 3 trục liên tiếp
       * ===*/
      /* PITCH */
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd,
                              (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);

      /* ROLL */
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd,
                              (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);

      /* YAW */
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd,
                              (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

      /* === Phase 1 (Pipeline): Gửi lệnh NOP để clock ra Data góc === */
      /* PITCH */
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd,
                              (uint8_t *)&rx_angle_pitch, 1, 2);
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);

      /* ROLL */
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd,
                              (uint8_t *)&rx_angle_roll, 1, 2);
      HAL_GPIO_WritePin(ENC_ROLL_CS_GPIO_Port, ENC_ROLL_CS_Pin, GPIO_PIN_SET);

      /* YAW */
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd,
                              (uint8_t *)&rx_angle_yaw, 1, 2);
      HAL_GPIO_WritePin(ENC_YAW_CS_GPIO_Port, ENC_YAW_CS_Pin, GPIO_PIN_SET);

      log_enc_raw = rx_angle_pitch;

      /* === Xử lý Parity và tính Góc === */
      if (AS5048A_CheckParity(rx_angle_pitch)) {
        pitch_enc.raw_angle = rx_angle_pitch & AS5048A_DATA_MASK;
        pitch_enc.angle_rad =
            (float)pitch_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        pitch_enc.angle_deg =
            (float)pitch_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
      if (AS5048A_CheckParity(rx_angle_roll)) {
        roll_enc.raw_angle = rx_angle_roll & AS5048A_DATA_MASK;
        roll_enc.angle_rad =
            (float)roll_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        roll_enc.angle_deg =
            (float)roll_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
      if (AS5048A_CheckParity(rx_angle_yaw)) {
        yaw_enc.raw_angle = rx_angle_yaw & AS5048A_DATA_MASK;
        yaw_enc.angle_rad =
            (float)yaw_enc.raw_angle * (TWO_PI / AS5048A_MAX_VALUE);
        yaw_enc.angle_deg =
            (float)yaw_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }

      enc_busy = 0;
    }
    /* TIM6: Kích hoạt SPI3 DMA đọc Burst 15-byte cho ICM42688 (SPI3) */
    if (!icm_dma_busy && icm_init_ok) {
      icm_dma_busy = 1;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
      if (HAL_SPI_TransmitReceive_DMA(&hspi3, icm_tx_buf, icm_rx_buf, 15) !=
          HAL_OK) {
        icm_dma_busy = 0;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
      }
    }
  } else if (htim->Instance == TIM7) {
    /* Chạy Vòng lặp kín theo chế độ hiện tại (không bị nghẽn I/O) */
    if (g_mode == TEST_MODE_VELOCITY) {
      FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, target_velocity_rad_s);
    } else if (g_mode == TEST_MODE_DEMO_1AXIS) {
      /* 1. Sai số góc IMU tuyệt đối */
      float imu_error = gimbal_target_roll_rad - imu_roll_rad;
      if (imu_error > PI)
        imu_error -= TWO_PI;
      if (imu_error < -PI)
        imu_error += TWO_PI;

      /* 2. Position PID -> Lệnh vận tốc cơ học (rad/s) */
      float target_vel =
          PID_Update(&foc_pitch.pid_pos, imu_error, foc_pitch.Ts);

      /* 3. Gọi FOC_VelocityLoop
       * - Tự tính velocity_mech = d(pitch_enc.angle_rad) / dt
       * - Tự cập nhật góc điện elec_angle = pitch_enc.angle_rad * pole_pairs
       * - Tính Vq = pid_vel(target_vel - velocity_mech)
       * - Gọi FOC_SVPWM sinh từ trường bám theo rotor (closeloop hoàn toàn) */
      FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, target_vel);
    } else if (g_mode == TEST_MODE_POSITION) {
      /* Clamp mục tiêu trong giới hạn mềm (Soft Limit) trước khi đưa vào FOC */
      const float pitch_min_rad = DEG2RAD(PITCH_MIN_DEG);
      const float pitch_max_rad = DEG2RAD(PITCH_MAX_DEG);
      const float roll_min_rad = DEG2RAD(ROLL_MIN_DEG);
      const float roll_max_rad = DEG2RAD(ROLL_MAX_DEG);
      const float yaw_min_rad = DEG2RAD(YAW_MIN_DEG);
      const float yaw_max_rad = DEG2RAD(YAW_MAX_DEG);

      if (target_pitch_angle < pitch_min_rad)
        target_pitch_angle = pitch_min_rad;
      if (target_pitch_angle > pitch_max_rad)
        target_pitch_angle = pitch_max_rad;
      if (target_roll_angle < roll_min_rad)
        target_roll_angle = roll_min_rad;
      if (target_roll_angle > roll_max_rad)
        target_roll_angle = roll_max_rad;
      if (target_yaw_angle < yaw_min_rad)
        target_yaw_angle = yaw_min_rad;
      if (target_yaw_angle > yaw_max_rad)
        target_yaw_angle = yaw_max_rad;

      /* Gọi hàm Position Loop cho cả 3 trục */
      FOC_PositionLoop(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
      FOC_PositionLoop(&foc_roll, roll_enc.angle_rad, target_roll_angle);
      FOC_PositionLoop(&foc_yaw, yaw_enc.angle_rad, target_yaw_angle);
    } else if (g_mode == TEST_MODE_DEMO_3AXIS) {
      /* 1. Lấy Quaternion đo được từ Mahony (cập nhật liên tục từ DMA ISR) */
      Quaternion_t q_meas = {mahony_imu.q0, mahony_imu.q1, mahony_imu.q2,
                             mahony_imu.q3};

      /* 2. Tính vector sai số góc 3D (không Gimbal Lock) */
      Quaternion_ComputeError(&q_demo3_target, &q_meas, e_rot);

      /* 3. Vòng ngoài: sai số góc → lệnh vận tốc (rad/s) cho từng trục */
      float vel_roll = DEMO3AX_SIGN_ROLL *
                       PID_Update(&pid_3ax_roll_pos, e_rot[1], foc_roll.Ts);
      float vel_pitch = DEMO3AX_SIGN_PITCH *
                        PID_Update(&pid_3ax_pitch_pos, e_rot[0], foc_pitch.Ts);
      float vel_yaw =
          DEMO3AX_SIGN_YAW * PID_Update(&pid_3ax_yaw_pos, e_rot[2], foc_yaw.Ts);

      /* 4. Vòng trong: FOC_VelocityLoop đóng vòng tốc độ + điện áp (Encoder làm
       * feedback) */
      FOC_VelocityLoop(&foc_roll, roll_enc.angle_rad, vel_roll);
      FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, vel_pitch);
      FOC_VelocityLoop(&foc_yaw, yaw_enc.angle_rad, vel_yaw);
    }
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    // uint32_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    // uint32_t raw_b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    // log_raw_ia = raw_a;
    // log_raw_ib = raw_b;

    // if (!foc_pitch.current_calibrated) {
    //   /* === Giai đoạn hiệu chỉnh offset zero-current ===
    //    * Motor đang dừng (PWM=50% Duty), không có dòng.
    //    * Tích lũy ADC raw để tính offset trung bình. */
    //   cal_sum_a += raw_a;
    //   cal_sum_b += raw_b;
    //   cal_count++;
    //   if (cal_count >= ADC_CAL_SAMPLES) {
    //     FOC_CalibrateCurrentOffset(&foc_pitch, cal_sum_a, cal_sum_b,
    //                                ADC_CAL_SAMPLES);
    //     /* current_calibrated được set = 1 tự động bởi
    //      * FOC_CalibrateCurrentOffset */
    //   }
    // } else {
    //   /* === Giai đoạn điều khiển bình thường === */
    //   FOC_UpdateCurrentLoopADC(&foc_pitch, raw_a, raw_b);
    // }
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
