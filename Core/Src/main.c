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

#define COMM_MODE_TERMINAL_LOG 1
#define COMM_MODE_APP_GUI 2

#define TEST_MODE_VELOCITY 1
#define TEST_MODE_POSITION 2
#define TEST_MODE_CURRENT 3
#define TEST_MODE_ICM42688 4
#define TEST_MODE_DEMO_1AXIS 5

#define TEST_MODE TEST_MODE_DEMO_1AXIS
#define COMM_MODE COMM_MODE_TERMINAL_LOG
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

AS5048A_Handle_t pitch_enc;
FOC_Handle_t foc_pitch;

ICM42688_Handle_t imu_payload;
MahonyFilter_t mahony_imu;
uint8_t icm_init_ok = 0;

Quaternion_t q_target_3d = {1.0f, 0.0f, 0.0f, 0.0f};
float e_rot[3] = {0.0f, 0.0f, 0.0f};

/* Biến lưu góc và vận tốc mục tiêu (rad & rad/s) */
volatile float target_pitch_angle = 0.0f;
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

float pitch_offset_a = 0.0f;
float pitch_offset_b = 0.0f;

/* Biến lưu trữ raw ADC để debug */
volatile uint32_t log_raw_ia = 0;
volatile uint32_t log_raw_ib = 0;
volatile uint16_t log_enc_raw = 0; /* Debug: raw SPI data từ AS5048A */

/* Biến runtime điều khiển từ GUI (volatile vì được ghi từ ISR CDC) */
volatile uint8_t g_mode = TEST_MODE; /* Chế độ hiện tại: 1=VEL, 2=POS */
volatile uint8_t g_foc_stop_req = 0;  /* Flag dừng khẩn cấp từ GUI */
volatile uint8_t g_foc_start_req = 0; /* Flag bật lại FOC từ GUI */
volatile uint8_t g_align_req = 0;     /* Flag re-alignment từ GUI */

/* ===========================================================
 * Biến hiệu chỉnh offset zero-current ADC (chạy trong ISR)
 * Lấy mẫu khi motor đang dừng (Duty=50%, không có dòng).
 * =========================================================== */
volatile uint32_t cal_sum_a = 0;
volatile uint32_t cal_sum_b = 0;
volatile uint32_t cal_count = 0;
#define ADC_CAL_SAMPLES 2000 /* 2000 mẫu × 50μs/mẫu = 100ms hiệu chỉnh */

/* Cấu hình phần cứng mạch dòng */
#define GAIN_DRV 10.0f
#define SHUNT_RES 0.005f
#define VOLTAGE_LIMIT                                                                               \
  1.5f /* Nâng giới hạn điện áp 1.5V để Vq không bị bão hòa va đập (clamp) gây \
          rung motor */
#define PWM_PERIOD 4249.0f
#define MOTOR_POLE_PAIRS 14

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ============================================================
 * DEMO1AXIS_Enter() — Vào LOCK_MODE (IMU-based stabilization)
 *
 * Thực hiện:
 *   1. Backup PID Position/Velocity của Encoder mode
 *   2. Khởi tạo PID LOCK riêng với tham số phù hợp plant IMU
 *      (Kp_pos nhỏ hơn do trễ pha Mahony, Kp_vel lớn hơn vì Gyro mịn)
 *   3. Swap PID mới vào foc_pitch (đã reset integral = 0)
 *   4. Chốt gimbal_target_roll_rad = imu_roll_rad tại thời điểm gọi
 *      (giữ đúng góc hiện tại, không bị giật về 0°)
 *
 * Gọi sau khi: IMU đã khởi tạo và có dữ liệu ổn định
 * ============================================================ */
#if (TEST_MODE == TEST_MODE_DEMO_1AXIS)
static void DEMO1AXIS_Enter(void) {
  /* Bước 1: Backup PID Encoder mode */
  pid_backup_pos = foc_pitch.pid_pos;
  pid_backup_vel = foc_pitch.pid_vel;

  /* Bước 2: Khởi tạo PID LOCK riêng cho IMU plant */
  PID_Init(&pid_lock_pos, 4.5f, 0.0f, 0.0f, -1.5f, 1.5f);
  PID_Init(&pid_lock_vel, 0.45f, 0.0f, 0.0f, -1.5f, 1.5f);

  /* Bước 3: Swap PID vào FOC handle */
  foc_pitch.pid_pos = pid_lock_pos;
  foc_pitch.pid_vel = pid_lock_vel;

  /* Bước 4: Chốt góc mục tiêu = góc IMU thực tế tại thời điểm vào mode
   * (không cứng về 0° để tránh giật khi gimbal đang nghiêng lúc bật nguồn) */
  gimbal_target_roll_rad = 0;
}

/* ============================================================
 * DEMO1AXIS_Exit() — Thoát LOCK_MODE
 *
 * Khôi phục PID Encoder mode với double-reset để đảm bảo:
 *   - Integral của LOCK không ảnh hưởng mode kế tiếp
 *   - Integral stale trong backup không gây spike khi restore
 * ============================================================ */
static void DEMO1AXIS_Exit(void) {
  /* Reset PID LOCK hiện tại trước khi ghi đè */
  PID_Reset(&foc_pitch.pid_pos);
  PID_Reset(&foc_pitch.pid_vel);

  /* Khôi phục PID Encoder mode */
  foc_pitch.pid_pos = pid_backup_pos;
  foc_pitch.pid_vel = pid_backup_vel;

  /* Reset lại PID vừa restore để xóa integral cũ trước khi vào LOCK */
  PID_Reset(&foc_pitch.pid_pos);
  PID_Reset(&foc_pitch.pid_vel);
}
#endif /* TEST_MODE == TEST_MODE_DEMO_1AXIS */

/**
 * @brief  ParseCommand — Xử lý lệnh '#CMD,val1,val2,...' nhận từ GUI qua USB
 * CDC
 *
 * Được gọi từ CDC_Receive_FS() trong USB ISR context.
 * Cần nhanh, không block. Ghi giá trị trực tiếp hoặc set volatile flag.
 *
 * Lệnh hỗ trợ:
 *   #MODE,VEL|POS        — Đổi chế độ điều khiển
 *   #TPOS,<deg>          — Đặt góc mục tiêu (chế độ Position)
 *   #TVEL,<rad_s>        — Đặt vận tốc mục tiêu (chế độ Velocity)
 *   #PID_POS,Kp,Ki,Kd   — Cập nhật PID vòng vị trí
 *   #PID_VEL,Kp,Ki,Kd   — Cập nhật PID vòng vận tốc
 *   #LPF,alpha           — Cập nhật LPF vận tốc
 *   #VLIM,volts          — Cập nhật giới hạn điện áp Vq
 *   #STOP                — Dừng động cơ (disable FOC)
 *   #START               — Bật lại FOC
 *   #ALIGN               — Yêu cầu re-alignment trục D
 */
void ParseCommand(const char *line) {
#if (COMM_MODE == COMM_MODE_APP_GUI)
  /* Sao chép sang buffer local để dùng strtok an toàn */
  static char buf[128];
  strncpy(buf, line, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  /* Tách CMD (trước dấu ',') */
  char *cmd = strtok(buf, ",");
  if (!cmd)
    return;

  /* -------------------------------------------------------- */
  if (strcmp(cmd, "#MODE") == 0) {
    char *val = strtok(NULL, ",");
    if (!val)
      return;
    if (strncmp(val, "VEL", 3) == 0) {
      g_mode = TEST_MODE_VELOCITY;
      foc_pitch.velocity_loop_enabled = 1;
      foc_pitch.position_loop_enabled = 0;
    } else if (strncmp(val, "POS", 3) == 0) {
      g_mode = TEST_MODE_POSITION;
      foc_pitch.velocity_loop_enabled = 1;
      foc_pitch.position_loop_enabled = 1;
      target_pitch_angle = pitch_enc.angle_rad; /* giữ vị trí hiện tại */
    } else if (strncmp(val, "ICM", 3) == 0) {
      g_mode = TEST_MODE_ICM42688;
    }

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#TPOS") == 0) {
    char *val = strtok(NULL, ",");
    if (!val)
      return;
    float deg = strtof(val, NULL);
    target_pitch_angle = deg * (3.14159265f / 180.0f);

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#TVEL") == 0) {
    char *val = strtok(NULL, ",");
    if (!val)
      return;
    target_velocity_rad_s = strtof(val, NULL);

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#PID_POS") == 0) {
    char *s_kp = strtok(NULL, ",");
    char *s_ki = strtok(NULL, ",");
    char *s_kd = strtok(NULL, ",");
    if (!s_kp || !s_ki || !s_kd)
      return;
    float kp = strtof(s_kp, NULL);
    float ki = strtof(s_ki, NULL);
    float kd = strtof(s_kd, NULL);
    FOC_SetPID_POS(&foc_pitch, kp, ki, kd, foc_pitch.pid_pos.out_min,
                   foc_pitch.pid_pos.out_max);

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#PID_VEL") == 0) {
    char *s_kp = strtok(NULL, ",");
    char *s_ki = strtok(NULL, ",");
    char *s_kd = strtok(NULL, ",");
    if (!s_kp || !s_ki || !s_kd)
      return;
    float kp = strtof(s_kp, NULL);
    float ki = strtof(s_ki, NULL);
    float kd = strtof(s_kd, NULL);
    FOC_SetPID_VEL(&foc_pitch, kp, ki, kd, foc_pitch.pid_vel.out_min,
                   foc_pitch.pid_vel.out_max);

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#LPF") == 0) {
    char *val = strtok(NULL, ",");
    if (!val)
      return;
    float alpha = strtof(val, NULL);
    if (alpha > 0.0f && alpha < 1.0f) {
      FOC_SetLPF_Vel(&foc_pitch, alpha);
    }

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#VLIM") == 0) {
    char *val = strtok(NULL, ",");
    if (!val)
      return;
    float vlim = strtof(val, NULL);
    if (vlim > 0.0f && vlim <= 12.0f) {
      foc_pitch.voltage_limit = vlim;
      /* Cập nhật lại output clamp của Velocity PID */
      FOC_SetPID_VEL(&foc_pitch, foc_pitch.pid_vel.Kp, foc_pitch.pid_vel.Ki,
                     foc_pitch.pid_vel.Kd, -vlim, vlim);
    }

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#STOP") == 0) {
    g_foc_stop_req = 1;

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#START") == 0) {
    g_foc_start_req = 1;

    /* -------------------------------------------------------- */
  } else if (strcmp(cmd, "#ALIGN") == 0) {
    g_align_req = 1;
  }
#endif
}

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
  /* 1. Khởi tạo cảm biến góc AS5048A Encoder */

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  AS5048A_Init(&pitch_enc, &hspi1, ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin);
  HAL_TIM_Base_Start_IT(&htim6);

  HAL_Delay(100); // Chờ lấy mẫu vài frame góc ban đầu từ AS5048A

  FOC_Init(&foc_pitch, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS, 12.0f, /* voltage_supply: Bus DC 12V */
           VOLTAGE_LIMIT, 1.0f, 0.0005f, 0.00005f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); /* PA8 -> Phase A */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); /* PA9 -> Phase B */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); /* PA10 -> Phase C */

  FOC_SetPID_POS(&foc_pitch, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  FOC_SetPID_VEL(&foc_pitch, 0.12f, 0.4f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_pitch, 0.96f);

  /* Cấu hình Vòng lặp Hở (Open Loop Voltage = 0.30V) để test chiều dòng */
  FOC_SetPID_D(&foc_pitch, 0.0f, 0.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_pitch, 0.0f, 0.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  foc_pitch.pid_d.integral = 0.0f;
  foc_pitch.pid_q.integral = 0.30f; /* Áp điện áp cố định Vq = +0.30V */
  FOC_ConfigureCurrentSense(&foc_pitch, SHUNT_RES, GAIN_DRV, 3.3f);

#if (TEST_MODE == TEST_MODE_POSITION)
  foc_pitch.enabled = 1;
  FOC_AlignD(&foc_pitch, 0.5f);
  HAL_Delay(1000);
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  foc_pitch.position_loop_enabled = 1;
  foc_pitch.velocity_loop_enabled = 1;
  foc_pitch.current_loop_enabled = 0;
  target_pitch_angle = pitch_enc.angle_rad;
  HAL_TIM_Base_Start_IT(&htim7);
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

#elif (TEST_MODE == TEST_MODE_DEMO_1AXIS)
  /* ================================================================
   * DEMO_1AXIS: Gimbal 1 trục — LOCK_MODE (giữ góc Roll IMU cố định)
   * Luồng: ICM42688 (SPI3 DMA) → Mahony AHRS → imu_roll_rad
   *        Gyro X (hardware LPF 50Hz) → imu_roll_vel_rad_s
   *        TIM7 (2kHz) → FOC_PositionLoop_IMU → SVPWM
   * PID: Dùng PID riêng (pid_lock_pos/vel), độc lập với Encoder modes
   * ================================================================ */

  /* 1. Khởi tạo IMU ICM42688 qua SPI3 DMA */
  ICM42688_Status_t icm_status =
      ICM42688_Init(&imu_payload, &hspi3, GPIOC, GPIO_PIN_6, NULL);
  if (icm_status == ICM42688_OK) {
    icm_init_ok = 1;
    printf("[DEMO_1AXIS] ICM42688 OK! Dang hieu chuan Gyro Bias (giu im "
           "1s)...\r\n");
    /* CalibrateGyroBias mất ~500ms → TIM6 DMA sẽ cập nhật imu_roll_rad
     * trong thời gian này, đảm bảo imu_roll_rad có giá trị thực khi
     * DEMO1AXIS_Enter() */
    ICM42688_CalibrateGyroBias(&imu_payload, 500);
    printf("[DEMO_1AXIS] Bias hoan tat: X=%.2f Y=%.2f Z=%.2f dps\r\n",
           imu_payload.gyro_bias_x, imu_payload.gyro_bias_y,
           imu_payload.gyro_bias_z);
    /* 2. Khởi tạo Mahony AHRS để ước lượng Roll chính xác */
    Mahony_Init(&mahony_imu, 1.0f, 0.005f);
  } else {
    icm_init_ok = 0;
    printf("[DEMO_1AXIS] LOI khoi tao ICM42688! Code: %d\r\n", icm_status);
  }

  /* 3. Khởi tạo FOC motor — Position Loop (Encoder làm SVPWM, IMU làm feedback)
   */
  foc_pitch.enabled = 1;
  FOC_AlignD(&foc_pitch, 0.5f); /* Căn chỉnh trục D */
  HAL_Delay(1000);
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  foc_pitch.position_loop_enabled = 1;
  foc_pitch.velocity_loop_enabled = 1;
  foc_pitch.current_loop_enabled = 0;

  /* 4. Swap PID sang LOCK_MODE và chốt góc mục tiêu = góc IMU hiện tại
   *    (gọi sau khi IMU đã chạy ~500ms → imu_roll_rad có giá trị thực)
   *    Đây là điểm cốt lõi: PID Encoder mode KHÔNG bị ảnh hưởng */
  DEMO1AXIS_Enter();
  printf("[DEMO_1AXIS] LOCK_MODE kich hoat. Target Roll: %.2f deg\r\n",
         gimbal_target_roll_rad * 57.2957795f);

  /* 5. Bật control loop 2kHz (TIM7) */
  HAL_TIM_Base_Start_IT(&htim7);
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

#if (COMM_MODE == COMM_MODE_APP_GUI)
    /* --- Xử lý các yêu cầu từ GUI (volatile flags từ ParseCommand) --- */
    if (g_foc_stop_req) {
      g_foc_stop_req = 0;
      foc_pitch.enabled = 0;
      FOC_Stop(&foc_pitch);
    }
    if (g_foc_start_req) {
      g_foc_start_req = 0;
      foc_pitch.enabled = 1;
    }
    if (g_align_req) {
      g_align_req = 0;
      foc_pitch.enabled = 0;
      FOC_AlignD(&foc_pitch, 0.5f);
      HAL_Delay(1000);
      FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
      target_pitch_angle = pitch_enc.angle_rad;
      foc_pitch.enabled = 1;
    }

    /* --- Gửi Telemetry lên GUI qua USB CDC (10Hz) --- */
    if (now - last_print_time >= 100) {
      last_print_time = now;

      if (g_mode == TEST_MODE_VELOCITY) {
        /* Format: $VEL,target_vel,vel_filt,vel_raw,vq,enc_deg */
        printf("$VEL,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", target_velocity_rad_s,
               foc_pitch.velocity_mech, foc_pitch.velocity_mech_raw,
               foc_pitch.Vq_ref, pitch_enc.angle_deg);
      } else if (g_mode == TEST_MODE_ICM42688) {
        if (icm_init_ok) {
          ICM42688_ReadSensor(&imu_payload);
          printf("$IMU,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f\r\n",
                 imu_payload.accel_x_g, imu_payload.accel_y_g,
                 imu_payload.accel_z_g, imu_payload.gyro_x_dps,
                 imu_payload.gyro_y_dps, imu_payload.gyro_z_dps,
                 imu_payload.temp_c);
        }
      } else {
        /* Format: $POS,target_pos,enc_deg,err_deg,vq,vel_filt */
        float target_pos_deg = target_pitch_angle * (180.0f / 3.14159265f);
        float err_pos_deg = target_pos_deg - pitch_enc.angle_deg;
        while (err_pos_deg > 180.0f)
          err_pos_deg -= 360.0f;
        while (err_pos_deg < -180.0f)
          err_pos_deg += 360.0f;
        printf("$POS,%.1f,%.1f,%.1f,%.2f,%.2f\r\n", target_pos_deg,
               pitch_enc.angle_deg, err_pos_deg, foc_pitch.Vq_ref,
               foc_pitch.velocity_mech);
      }
    }
#elif (COMM_MODE == COMM_MODE_TERMINAL_LOG)
#if (TEST_MODE == TEST_MODE_POSITION)
    /* Kịch bản test step response vị trí tự động mỗi 4 giây: 0 rad ->
    +90 deg
     * -> +180 deg -> -90 deg */
    if (now - last_step_time >= 4000) {
      last_step_time = now;
      step_state = (step_state + 1) % 4;
      if (step_state == 0) {
        target_pitch_angle = 0.0f; /* 0 deg */
      } else if (step_state == 1) {
        target_pitch_angle = 1.5707963f; /* +90 deg (+pi/2 rad) */
      } else if (step_state == 2) {
        target_pitch_angle = 3.14159265f; /* +180 deg (+pi rad) */
      } else if (step_state == 3) {
        target_pitch_angle = -1.5707963f; /* -90 deg (-pi/2 rad) */
      }
    }

    float target_pos_deg = target_pitch_angle * (180.0f / 3.14159265f);
    float err_pos_deg = target_pos_deg - pitch_enc.angle_deg;
    while (err_pos_deg > 180.0f)
      err_pos_deg -= 360.0f;
    while (err_pos_deg < -180.0f)
      err_pos_deg += 360.0f;

    /* Print Telemetry (10Hz) để giám sát đáp ứng vị trí */
    printf("TargetPos: %.1f deg | Enc: %.1f deg | Err: %.1f deg | Vq: %.2fV | "
           "VelFilt: %.2f\r\n",
           target_pos_deg, pitch_enc.angle_deg, err_pos_deg, foc_pitch.Vq_ref,
           foc_pitch.velocity_mech);

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
#endif
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
    Mahony_Update(&mahony_imu, imu_payload.gyro_x_dps * 0.0174532925f,
                  imu_payload.gyro_y_dps * 0.0174532925f,
                  imu_payload.gyro_z_dps * 0.0174532925f, imu_payload.accel_x_g,
                  imu_payload.accel_y_g, imu_payload.accel_z_g, 0.001f);

    Quaternion_t q_meas_3d = {mahony_imu.q0, mahony_imu.q1, mahony_imu.q2,
                              mahony_imu.q3};
    Quaternion_ComputeError(&q_target_3d, &q_meas_3d, e_rot);

    /* Cập nhật biến Roll cho DEMO_1AXIS stabilization loop
     * imu_roll_rad     : Góc Roll đầu ra từ Mahony AHRS [rad]
     * imu_roll_vel_rad_s: Vận tốc góc Gyro_X [rad/s] — dùng cho inner velocity
     * loop */
    imu_roll_rad = -mahony_imu.roll;
    imu_roll_vel_rad_s = -imu_payload.gyro_x_dps * 0.0174532925f;
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
      uint16_t rx_dummy, rx_angle;

      /* Phase 0: Gửi lệnh READ ANGLE (0xFFFF) */
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_read_cmd,
                              (uint8_t *)&rx_dummy, 1, 2);
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
      /* Delay > 350ns (tCSn) */
      for (volatile int i = 0; i < 200; i++) {
      }

      /* Phase 1: Gửi lệnh NOP (0xC000) để clock ra dữ liệu của Phase 0 */
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&enc_tx_nop_cmd,
                              (uint8_t *)&rx_angle, 1, 2);
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);

      log_enc_raw = rx_angle;
      if (AS5048A_CheckParity(rx_angle)) {
        pitch_enc.raw_angle = rx_angle & AS5048A_DATA_MASK;
        pitch_enc.angle_rad =
            (float)pitch_enc.raw_angle * (6.28318530718f / AS5048A_MAX_VALUE);
        pitch_enc.angle_deg =
            (float)pitch_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
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
      if (imu_error > 3.14159265359f)
        imu_error -= 6.28318530718f;
      if (imu_error < -3.14159265359f)
        imu_error += 6.28318530718f;

      /* 2. Position PID -> Lệnh vận tốc cơ học (rad/s) */
      float target_vel =
          PID_Update(&foc_pitch.pid_pos, imu_error, foc_pitch.Ts);

      /* 3. Gọi FOC_VelocityLoop
       * - Tự tính velocity_mech = d(pitch_enc.angle_rad) / dt
       * - Tự cập nhật góc điện elec_angle = pitch_enc.angle_rad * pole_pairs
       * - Tính Vq = pid_vel(target_vel - velocity_mech)
       * - Gọi FOC_SVPWM sinh từ trường bám theo rotor (closeloop hoàn toàn) */
      FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, target_vel);
    } else {
      FOC_PositionLoop(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
    }
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    uint32_t raw_a = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_b = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    log_raw_ia = raw_a;
    log_raw_ib = raw_b;

    if (!foc_pitch.current_calibrated) {
      /* === Giai đoạn hiệu chỉnh offset zero-current ===
       * Motor đang dừng (PWM=50% Duty), không có dòng.
       * Tích lũy ADC raw để tính offset trung bình. */
      cal_sum_a += raw_a;
      cal_sum_b += raw_b;
      cal_count++;
      if (cal_count >= ADC_CAL_SAMPLES) {
        FOC_CalibrateCurrentOffset(&foc_pitch, cal_sum_a, cal_sum_b,
                                   ADC_CAL_SAMPLES);
        /* current_calibrated được set = 1 tự động bởi
         * FOC_CalibrateCurrentOffset */
      }
    } else {
      /* === Giai đoạn điều khiển bình thường === */
      FOC_UpdateCurrentLoopADC(&foc_pitch, raw_a, raw_b);
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
