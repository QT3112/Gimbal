/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "spi.h"
#include "stm32g4xx_hal_gpio.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5048a.h"
#include "foc_v1.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_MODE_VELOCITY 1
#define TEST_MODE_POSITION 2

/* Chọn chế độ thử nghiệm: TEST_MODE_VELOCITY hoặc TEST_MODE_POSITION */
#define TEST_MODE TEST_MODE_POSITION
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

AS5048A_Handle_t pitch_enc;
FOC_Handle_t foc_pitch;

/* Biến lưu góc và vận tốc mục tiêu (rad & rad/s) */
volatile float target_pitch_angle = 0.0f;
volatile float target_velocity_rad_s = 0.0f;
float test_angle_elec = 0.0f;

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

  /* 2. Cấu hình Tham số PID & LPF Tối ưu (Siêu mượt & Triệt tiêu sai số góc) */
  /* Position PID: Kp = 6.0f, Ki = 0.4f để triệt tiêu sai số góc ma sát tĩnh 2.5
   * deg */
  FOC_SetPID_POS(&foc_pitch, 6.0f, 0.4f, 0.0f, -3.0f, 3.0f);
  /* Velocity PID: Kp_vel = 0.12f, Ki_vel = 0.4f cho đáp ứng siêu mượt */
  FOC_SetPID_VEL(&foc_pitch, 0.12f, 0.4f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  /* LPF Vel: alpha = 0.96f triệt tiêu hoàn toàn nhiễu bước nhảy lượng hóa 0.77
   * rad/s */
  FOC_SetLPF_Vel(&foc_pitch, 0.96f);

  /* 3. Bật FOC và thực hiện Alignment D-axis để đồng bộ góc encoder */
  foc_pitch.enabled = 1;
  FOC_AlignD(&foc_pitch, 0.5f); /* Áp 0.5V vào trục D để giữ rotor */
  HAL_Delay(1000); /* Chờ 1 giây cho rotor định vị ổn định */

  /* Lưu offset góc điện tương ứng với vị trí cơ hiện tại của encoder */
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);

  /* 4. Kích hoạt Chế độ Vòng lặp tương ứng với TEST_MODE */
#if (TEST_MODE == TEST_MODE_VELOCITY)
  foc_pitch.velocity_loop_enabled = 1;
  foc_pitch.position_loop_enabled = 0;
  foc_pitch.current_loop_enabled = 0;
#elif (TEST_MODE == TEST_MODE_POSITION)
  /* Chạy Cascade Position-Velocity: Bật cả 2 flag */
  foc_pitch.velocity_loop_enabled = 1;
  foc_pitch.position_loop_enabled = 1;
  foc_pitch.current_loop_enabled = 0;
  target_pitch_angle =
      pitch_enc.angle_rad; /* Khởi tạo vị trí đích tại mốc hiện tại */
#endif

  HAL_TIM_Base_Start_IT(&htim7);

  /* Khởi tạo giá trị mặc định: GUI sẽ điều khiển từ đây */
  uint32_t last_print_time = HAL_GetTick();

  while (1) {
    uint32_t now = HAL_GetTick();

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
  }
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

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI1) {
    HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);

    if (enc_phase == 0) {
      /* Phase 0 xong: vừa gửi READ ANGLE, bỏ qua dữ liệu nhận về
       * → Ngay lập tức bắt đầu Phase 1 để lấy dữ liệu thực */
      enc_phase = 1;
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&enc_tx_nop_cmd,
                                  (uint8_t *)&enc_rx_angle, 1);
    } else {
      /* Phase 1 xong: enc_rx_angle chứa góc thực của frame trước */
      enc_phase = 0;
      enc_busy = 0;

      uint16_t data = enc_rx_angle;
      log_enc_raw = data; /* Lưu raw để debug */

      if (AS5048A_CheckParity(data)) {
        /* Chỉ kiểm tra parity - bỏ qua EF flag vì nó được set do lỗi
         * khởi động SPI, không phải lỗi đo đạc góc. Dữ liệu góc
         * trong bits[13:0] hợp lệ nếu parity đúng. */
        pitch_enc.raw_angle = data & AS5048A_DATA_MASK;
        pitch_enc.angle_rad =
            (float)pitch_enc.raw_angle * (6.28318530718f / AS5048A_MAX_VALUE);
        pitch_enc.angle_deg =
            (float)pitch_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);
      }
    }
  }
}

/* Hàm ngắt Timer định kỳ: TIM6 (1kHz I/O Trigger) & TIM7 (2kHz Central Control
 * Loop) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    /* TIM6: Khởi động Phase 0 của 2-frame AS5048A pipeline */
    if (!enc_busy) {
      enc_busy = 1;
      enc_phase = 0;
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin,
                        GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)&enc_tx_read_cmd,
                                  (uint8_t *)&enc_rx_dummy, 1);
    }
  } else if (htim->Instance == TIM7) {
    /* Chạy Vòng lặp kín theo chế độ hiện tại (có thể đổi runtime từ GUI) */
    if (g_mode == TEST_MODE_VELOCITY) {
      FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, target_velocity_rad_s);
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
