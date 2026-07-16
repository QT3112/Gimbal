/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : FOC Motor Control — Cascade 3-Loop (Position/Velocity/Current)
 *
 * Kiến trúc điều khiển:
 *
 *   [TIM6 Ngắt @1ms]  — Vòng ngoài
 *        ├─ Đọc encoder AS5048A (Blocking SPI)
 *        ├─ PID vị trí → target_vel
 *        └─ FOC_RunVelocity() → tính Iq_ref (KHÔNG xuất PWM)
 *
 *   [ADC Injected ISR @~8.5kHz]  — Vòng trong (Current Loop)
 *        ├─ Đọc SO1, SO2 → Ia, Ib
 *        └─ FOC_UpdateCurrentLoop() → Clarke→Park→PID_dq→InvPark→SVPWM→PWM
 *
 *   [while(1)] — Chỉ in Serial (100ms), không chạm vào motor
 *
 * ⚠️  FOC_CURRENT_SENSING_ENABLED: Bật/tắt Current Loop
 *     Khi tắt: Voltage-Mode fallback (tương thích code cũ)
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "foc.h"
#include "math.h"
#include "stdint.h"
#include "stdio.h"
#include "as5048a.h"
#include "pid_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_PERIOD      4249.0f
#define PI              3.14159265359f
#define DEG_TO_RAD      (PI / 180.0f)
#define RAD_TO_DEG      (180.0f / PI)

/* --- Thông số motor --- */
#define MOTOR_POLE_PAIRS    14       /* Số cặp cực — chỉnh theo motor thực tế */
#define VOLTAGE_LIMIT       6.0f     /* Giới hạn điện áp phase [V] = 1/2 V_BUS. Nếu nguồn 12V -> 6.0f */
#define TS_S                0.001f   /* Chu kỳ điều khiển: 1ms (TIM6 ngắt @ 1000Hz) */

/* --- Align encoder --- */
#define ALIGN_VD            3.0f     /* Điện áp Vd khi align [V]. 3.0V là đủ cho motor ~10 ohm */
#define ALIGN_DURATION_MS   600U     /* Thời gian giữ Vd để rotor lock [ms] */

/* --- PID vòng vị trí (outer loop) --- */
/* LƯU Ý KIẾN TRÚC CASCADE:                                                  */
/*   Vòng current bên trong FOC đóng vai trò Damping tự nhiên rất tốt.       */
/*   KHÔNG dùng KD ở outer loop — sẽ tạo "double-derivative" gây mất ổn định */
#define POS_KP              0.8f     /* Tăng nếu về chậm, giảm nếu dao động */
#define POS_KI              0.0f     /* Chỉ bật sau khi KP đã ổn định */
#define POS_KD              0.0f     /* Tắt — inner current loop đã damping */
#define POS_VEL_MAX         1.5f     /* Giới hạn vận tốc [rad/s] */

/* --- PID vòng tốc độ (middle loop) --- */
/* Khi CURRENT_SENSING_ENABLED: out_min/max là giới hạn Iq [A]               */
/* Khi không: out_min/max là giới hạn Vq [V] (Voltage-Mode fallback)         */
#define VEL_KP              3.20f    /* Đã được nhân 40 lần do VOLTAGE_LIMIT tăng từ 0.15 lên 6.0 */
#define VEL_KI              0.0f
#define VEL_KD              0.0f
#define VEL_LPF_ALPHA       0.95f    /* Lọc nhiễu tốc độ (0=không lọc, 0.99=lọc mạnh) */

/* --- PID vòng dòng điện (inner loop) --- */
/* Điểm khởi đầu an toàn: Kp=0.5, Ki=100.0 */
#define CUR_KP              0.5f
#define CUR_KI              100.0f
#define CUR_KD              0.0f

/* --- Cấu hình cảm biến dòng DRV8302 --- */
/* Bỏ comment dòng dưới khi phần cứng ADC đã sẵn sàng (SO1/SO2 đã hàn dây)  */
#define FOC_CURRENT_SENSING_ENABLED
#define SHUNT_RESISTANCE    0.005f   /* Điện trở shunt [Ω] — đo lại trên board */
#define CURRENT_SENSE_GAIN  10.0f    /* Hệ số khuếch đại DRV8302 GAIN=GND: 10, GAIN=VCC: 40 */
#define CURRENT_LIMIT_A     1.0f     /* Giới hạn dòng tối đa bảo vệ [A] - Hạ xuống 1.0A để test an toàn */

/* Chu kỳ Current Loop = 1 / f_PWM
 * f_PWM = f_TIM / (2 * ARR) với Center-Aligned = 170MHz / (2 * 10000) = 8500Hz */
#define TS_CURRENT_S        (1.0f / 8500.0f)

/* --- Ngưỡng giữ vị trí --- */
#define HOLD_THRESHOLD_RAD  (2.0f * DEG_TO_RAD)   /* ±2° = coi như đến đích */

/* ==========================================================================
 * CHẾ ĐỘ TEST ĐỌC DÒNG ĐIỆN
 * Bỏ comment dòng dưới để bật test mode:
 *   - Motor tắt hoàn toàn (không phát PWM)
 *   - ADC Injected đọc SO1/SO2 liên tục
 *   - In Ia, Ib, offset thực tế ra Serial mỗi 200ms
 * Mục tiêu: Xác nhận Ia ≈ 0A, Ib ≈ 0A khi motor đứng yên
 * ========================================================================== */
// #define TEST_CURRENT_SENSE

/* --- AS5048A SPI: lệnh đọc góc (2 frame pipeline theo datasheet) --- */
/* Frame 1: gửi READ 0x3FFF, Frame 2: gửi NOP 0xC000 để lấy dữ liệu frame 1 */
#define AS5048A_CMD_READ_ANGLE   0xFFFFU   /* READ(1) + Addr(0x3FFF) + parity tính sẵn */
#define AS5048A_CMD_NOP          0xC000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* --- AS5048A Encoder --- */
AS5048A_Handle_t encoder;
volatile uint8_t encoder_ready = 0;

/* --- Trạng thái điều khiển --- */
typedef enum {
    STATE_ALIGN = 0,    /* Đang khóa rotor */
    STATE_RUN,          /* Vòng vị trí đang hoạt động */
} CtrlState_t;
volatile CtrlState_t ctrl_state = STATE_ALIGN;

/* --- FOC & PID handles --- */
FOC_Handle_t foc;
PID_Handle_t pid_pos;

/* --- Biến chia sẻ giữa ISR và while(1) (volatile để tránh lỗi cache) --- */
volatile float g_angle_rad  = 0.0f;
volatile float g_pos_error  = 0.0f;
volatile float g_target_vel = 0.0f;
volatile float g_Vq_ref     = 0.0f;

/* --- Biến debug Current Loop --- */
volatile float g_Ia     = 0.0f;   /* Dòng pha A đo được [A] */
volatile float g_Ib     = 0.0f;   /* Dòng pha B đo được [A] */
volatile float g_Id     = 0.0f;   /* Dòng d-axis sau Park [A] */
volatile float g_Iq     = 0.0f;   /* Dòng q-axis sau Park [A] */

/* Biến dùng cho test mode: lưu ADC raw và điện áp thực tế */
volatile uint32_t g_raw_A   = 0;      /* ADC thô kênh A (0-4095) */
volatile uint32_t g_raw_B   = 0;      /* ADC thô kênh B (0-4095) */
volatile float    g_vsen_A  = 0.0f;   /* Điện áp SO1 [V] */
volatile float    g_vsen_B  = 0.0f;   /* Điện áp SO2 [V] */

/* Offset tự động calib lúc khởi động */
volatile float g_offset_A   = 1.65f;
volatile float g_offset_B   = 1.65f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline float wrap_angle(float a);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  Đưa góc về [-π, +π] (shortest path)
 */
static inline float wrap_angle(float a)
{
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
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
  MX_DMA_Init();
  MX_USB_Device_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */

  /* --- Khởi động PWM 3 pha --- */
#ifndef TEST_CURRENT_SENSE
  /* Chế độ bình thường: bật PWM */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); /* Bật Channel 4 để làm trigger ADC (TRGO) */
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
#else
  /* Test mode: CHỈ cần TIM1 chạy để phát TRGO trigger cho ADC.
   * KHÔNG bật PWM channel → motor không quay, an toàn tuyệt đối.
   * TIM1 Base phải chạy để TRGO hoạt động. */
  HAL_TIM_Base_Start(&htim1);  /* Chỉ chạy bộ đếm, không phát PWM ra chân */
  printf("[TEST] Current Sense Test Mode — Motor OFF\r\n");
#endif

  /* Enable gate driver */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,  GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);  /* M-OC */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,  GPIO_PIN_SET);    /* OC-ADJ */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);    /* M-PWM */

  /* --- Khởi tạo AS5048A Encoder (SPI1, CS=PC4) --- */
  {
    AS5048A_Status_t enc_ret = AS5048A_Init(&encoder, &hspi1, GPIOC, GPIO_PIN_4);
    printf("[AS5048A] Init=%d (0=OK)\r\n", enc_ret);
    if (enc_ret == AS5048A_OK) {
      encoder_ready = 1;
      AS5048A_ReadDiagnostics(&encoder);
      printf("[AS5048A] AGC=%u CompH=%u CompL=%u COF=%u OCF=%u\r\n",
             encoder.agc_value, encoder.comp_high, encoder.comp_low,
             encoder.cordic_overflow, encoder.offset_comp_finished);
    }
  }

  /* --- Khởi tạo FOC --- */
  FOC_Init(&foc,
           &htim1,
           TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD,
           MOTOR_POLE_PAIRS,
           VOLTAGE_LIMIT,
           TS_S);

#ifdef FOC_CURRENT_SENSING_ENABLED
  /* True FOC: Velocity PID xuất Iq [A], giới hạn bởi CURRENT_LIMIT_A */
  FOC_SetPID_Vel(&foc, VEL_KP, VEL_KI, VEL_KD, -CURRENT_LIMIT_A, CURRENT_LIMIT_A);
  /* Current PID (trục d và q) */
  FOC_SetPID_D(&foc, CUR_KP, CUR_KI, CUR_KD, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc, CUR_KP, CUR_KI, CUR_KD, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetCurrentLimit(&foc, CURRENT_LIMIT_A);
  /* TẠM THỜI TẮT CURRENT LOOP ĐỂ CHẠY VOLTAGE-MODE NHƯNG VẪN ĐO DÒNG */
  // FOC_EnableCurrentLoop(&foc, TS_CURRENT_S);
  /* Hiệu chỉnh ADC nội bộ của STM32G4 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* Khởi động ADC Injected một lần */
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  /* =========================================================================
   * CALIBRATE CURRENT SENSOR OFFSET (Auto-Calibration)
   * ========================================================================= */
  printf("[ISNS] Calibrating zero-current offset...\r\n");
  float sum_A = 0.0f, sum_B = 0.0f;
  for (int i = 0; i < 500; i++) {
      sum_A += g_vsen_A;  /* Mẫu được ADC ISR cập nhật liên tục ở background */
      sum_B += g_vsen_B;
      HAL_Delay(1);
  }
  g_offset_A = sum_A / 500.0f;
  g_offset_B = sum_B / 500.0f;
  printf("[ISNS] Offset A: %.3fV | Offset B: %.3fV\r\n", g_offset_A, g_offset_B);

#else
  /* Voltage-Mode: Velocity PID xuất Vq [V] trực tiếp (fallback) */
  FOC_SetPID_Vel(&foc, VEL_KP, VEL_KI, VEL_KD, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
#endif
  FOC_SetLPF_Vel(&foc, VEL_LPF_ALPHA);

  /* --- Khởi tạo PID vị trí --- */
  PID_Init(&pid_pos, POS_KP, POS_KI, POS_KD, -POS_VEL_MAX, POS_VEL_MAX);

  /* =========================================================================
   * PHASE 1: ALIGN — Lock rotor, dùng Blocking SPI (trước khi bật DMA)
   * =========================================================================
   * Quá trình Align cần SPI Blocking thông thường vì ta cần đọc góc
   * chính xác ngay sau khi rotor lock. Sau khi Align xong mới bật DMA.
   */
  printf("[ALIGN] Bat dau khoa motor de can chinh...\r\n");

  /* Đọc lần đầu để lấy góc hiện tại, tránh velocity spike */
  if (encoder_ready) {
    AS5048A_ReadAngle(&encoder);
  }
  FOC_Start(&foc, encoder.angle_rad);

  uint32_t align_start = HAL_GetTick();
  while ((HAL_GetTick() - align_start) < ALIGN_DURATION_MS) {
    FOC_AlignD(&foc, ALIGN_VD);
    HAL_Delay(5);
  }

  /* Đọc góc sau khi rotor lock và lưu offset */
  if (encoder_ready && AS5048A_ReadAngle(&encoder) == AS5048A_OK) {
    FOC_CalibrateAngle(&foc, encoder.angle_rad);
    printf("[ALIGN] Thanh cong! Offset = %.4f rad (%.2f deg)\r\n",
           foc.angle_offset, foc.angle_offset * RAD_TO_DEG);
  } else {
    printf("[ALIGN] Loi doc encoder sau khi lock!\r\n");
  }

  /* =========================================================================
   * PHASE 2: Bật TIM6 để kích hoạt vòng điều khiển thời gian thực
   * =========================================================================
   * Từ đây, mọi việc tính toán đều diễn ra trong các hàm Callback ngắt,
   * không cần làm gì thêm trong while(1) ngoài việc in Serial.
   */
  ctrl_state = STATE_RUN;
  printf("[RUN] Bat dau dieu khien vi tri. Target: 0.0 rad\r\n");

  /* Bật TIM6: sẽ gọi HAL_TIM_PeriodElapsedCallback mỗi 1ms */
  HAL_TIM_Base_Start_IT(&htim6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#ifdef TEST_CURRENT_SENSE
  /* =========================================================================
   * TEST MODE: Đọc và in dòng điện mỗi 200ms
   * Motor TẮT — chỉ quan sát ADC
   *
   * Kết quả kỳ vọng khi motor đứng yên (Duty=50%):
   *   Ia ≈ 0.00 A  (±0.05A là chấp nhận được)
   *   Ib ≈ 0.00 A
   *   V_SO1 ≈ 1.65V, V_SO2 ≈ 1.65V (= VREF/2)
   *
   * Nếu lệch nhiều (±0.2A trở lên):
   *   → Đo V_SO1, V_SO2 bằng đồng hồ → cập nhật VREF_OFFSET
   *   → Kiểm tra GAIN pin DRV8302 (GND=10V/V, VCC=40V/V)
   *   → Kiểm tra giá trị SHUNT_RESISTANCE
   * ========================================================================= */
  printf("[TEST] ADC Calibration done. Waiting for readings...\r\n");
  HAL_Delay(500);
  while (1) {
      /* Đọc snapshot an toàn từ các biến volatile */
      uint32_t ra   = g_raw_A;
      uint32_t rb   = g_raw_B;
      float    va   = g_vsen_A;
      float    vb   = g_vsen_B;
      float    ia   = g_Ia;
      float    ib   = g_Ib;

      printf("[ISNS] RAW: A=%4lu B=%4lu | V_SO: A=%.3fV B=%.3fV | I: A=%+6.3fA B=%+6.3fA\r\n",
             ra, rb, va, vb, ia, ib);

      HAL_Delay(200);
      /* USER CODE END WHILE */
      /* USER CODE BEGIN 3 */
  }
#else
  while (1) {

    /* Đọc các biến volatile được ISR cập nhật và in ra Serial */
    /* In chậm (100ms) để không chiếm CPU của vòng điều khiển */
    float angle  = g_angle_rad;
    float err    = g_pos_error;
    float vel    = g_target_vel;
    float vq     = g_Vq_ref;

    if (fabsf(err) < HOLD_THRESHOLD_RAD) {
      printf("[HOLD] Ang: %6.2f | Ia: %5.2f | Ib: %5.2f | Ic: %5.2f | Id: %5.2f | Iq: %5.2f\r\n",
             angle * RAD_TO_DEG, g_Ia, g_Ib, -(g_Ia + g_Ib), g_Id, g_Iq);
    } else {
      printf("[HOME] Ang: %6.2f | Ia: %5.2f | Ib: %5.2f | Ic: %5.2f | Id: %5.2f | Iq: %5.2f\r\n",
             angle * RAD_TO_DEG, g_Ia, g_Ib, -(g_Ia + g_Ib), g_Id, g_Iq);
    }

    HAL_Delay(100); /* 10Hz in Serial, không ảnh hưởng đến FOC đang chạy ở 1000Hz */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#endif /* TEST_CURRENT_SENSE */
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

/**
 * @brief  TIM6 Period Elapsed Callback — Chạy mỗi 1ms (Vòng Velocity + Position)
 *
 * Nhiệm vụ:
 *   1. Đọc góc encoder AS5048A (Blocking SPI, ~20µs)
 *   2. PID vị trí (outer) → target_vel
 *   3. FOC_RunVelocity() → tính Iq_ref (True FOC) hoặc xuất PWM (Voltage-Mode)
 *
 * Lưu ý: Khi FOC_CURRENT_SENSING_ENABLED, PWM được quản lý bởi ADC ISR.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;
    if (ctrl_state != STATE_RUN) return;

    /* --- Bước 1: Đọc góc encoder (Blocking SPI) --- */
    /* BỎ QUA ĐỂ TEST OPEN LOOP */
    // if (!encoder_ready || AS5048A_ReadAngle(&encoder) != AS5048A_OK) return;
    // float angle_rad = encoder.angle_rad;

    /* --- Bước 2: PID vòng vị trí (outer loop) --- */
    /* BỎ QUA */
    // float pos_error  = wrap_angle(0.0f - angle_rad);
    // float target_vel = PID_Update(&pid_pos, pos_error, TS_S);

    /* --- Bước 3: FOC velocity loop --- */
    /* BỎ QUA */
    // FOC_RunVelocity(&foc, angle_rad, target_vel);

    /* === TEST OPEN LOOP THEO YÊU CẦU === */
    float target_vel_elec = 20.0f; /* Vận tốc điện: 10 rad/s */
    float target_vq = 0.5f;        /* Điện áp: 1.0V (chỉnh tùy motor) */
    FOC_RunOpenLoop(&foc, target_vel_elec, target_vq);

    /* Cập nhật biến chia sẻ để while(1) in ra */
    g_angle_rad  = foc.angle_elec; /* Hiển thị góc điện thay vì góc cơ */
    g_pos_error  = 0.0f;
    g_target_vel = target_vel_elec;
    g_Vq_ref     = foc.Vq_ref;
}

#ifdef FOC_CURRENT_SENSING_ENABLED
/**
 * @brief  ADC Injected Conversion Complete Callback — Vòng Current (~20kHz)
 *
 * Được kích hoạt tự động bởi TIM1 TRGO (Center-Aligned Update Event)
 * đúng lúc Counter = 0 (Mosfet cầu dưới mở hoàn toàn) → ADC sạch nhất.
 *
 * Pipeline (cả 2 chân đều trên ADC1):
 *   ADC1 INJECTED_RANK_1 → PA0 (ADC1_IN1) → Ia (SO1, Phase A)
 *   ADC1 INJECTED_RANK_2 → PA1 (ADC1_IN2) → Ib (SO2, Phase B)
 *   FOC_UpdateCurrentLoop() → Clarke→Park→PID_dq→InvPark→SVPWM→PWM
 *
 * KHÔNG cần gọi HAL_ADCEx_InjectedStart_IT() lại ở cuối:
 * TIM1 TRGO sẽ tự động kích hoạt lần đọc tiếp theo mỗi chu kỳ PWM.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    /* --- Đọc ADC thô — cả 2 rank đều từ ADC1 (PA0 và PA1) --- */
    uint32_t raw_A = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1); /* PA0 → Ia */
    uint32_t raw_B = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2); /* PA1 → Ib */

    /* --- Quy đổi ADC → Điện áp → Dòng điện [A] ---
     * V_sense = (raw / 4095) * 3.3V
     * DRV8302 shunt voltage drops when current flows OUT to the motor.
     * Therefore, Ia = - (V_sense - V_offset) / (Gain * R_shunt)
     */
    float v_A = ((float)raw_A / 4095.0f) * 3.3f;
    float v_B = ((float)raw_B / 4095.0f) * 3.3f;

    float Ia = -(v_A - g_offset_A) / (CURRENT_SENSE_GAIN * SHUNT_RESISTANCE);
    float Ib = -(v_B - g_offset_B) / (CURRENT_SENSE_GAIN * SHUNT_RESISTANCE);

    /* Lưu giá trị thô và điện áp để while(1) có thể in ra debug */
    g_raw_A  = raw_A;
    g_raw_B  = raw_B;
    g_vsen_A = v_A;
    g_vsen_B = v_B;
    g_Ia     = Ia;
    g_Ib     = Ib;

#ifndef TEST_CURRENT_SENSE
    /* Chế độ bình thường: chạy Current Loop FOC */
    FOC_UpdateCurrentLoop(&foc, Ia, Ib);
    g_Id = foc.Id_meas;
    g_Iq = foc.Iq_meas;
#endif
    /* KHÔNG gọi HAL_ADCEx_InjectedStart_IT() — TIM1 TRGO tự trigger */
}
#endif /* FOC_CURRENT_SENSING_ENABLED */

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
