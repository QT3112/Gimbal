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

  /* 2. Cấu hình Tham số PID & LPF Tối ưu */
  /* Position PID: Kp = 4.0f, Ki = 0.05f, giới hạn tốc độ mục tiêu [-3.0, 3.0]
   * rad/s */
  FOC_SetPID_POS(&foc_pitch, 4.0f, 0.05f, 0.0f, -3.0f, 3.0f);
  /* Velocity PID: Kp_vel = 0.15f, Ki_vel = 0.5f cho đáp ứng mượt */
  FOC_SetPID_VEL(&foc_pitch, 0.15f, 0.5f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  /* LPF Vel: alpha = 0.93f để lọc sạch gai nhiễu vi phân encoder */
  FOC_SetLPF_Vel(&foc_pitch, 0.93f);

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

  uint32_t last_step_time = HAL_GetTick();
  uint8_t step_state = 0;

  while (1) {
    uint32_t now = HAL_GetTick();

#if (TEST_MODE == TEST_MODE_VELOCITY)
    /* Kịch bản test step response vận tốc tự động mỗi 3 giây */
    if (now - last_step_time >= 3000) {
      last_step_time = now;
      step_state = (step_state + 1) % 3;
      if (step_state == 0) {
        target_velocity_rad_s = 3.0f; /* Quay thuận 3.0 rad/s (~28.6 RPM) */
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

#elif (TEST_MODE == TEST_MODE_POSITION)
    /* Kịch bản test step response vị trí tự động mỗi 4 giây: 0 rad -> +90 deg
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
    float err_pos_deg =
        (target_pitch_angle - pitch_enc.angle_rad) * (180.0f / 3.14159265f);

    /* Print Telemetry (10Hz) để giám sát đáp ứng vị trí */
    printf("TargetPos: %.1f deg | Enc: %.1f deg | Err: %.1f deg | Vq: %.2fV | "
           "VelFilt: %.2f\r\n",
           target_pos_deg, pitch_enc.angle_deg, err_pos_deg, foc_pitch.Vq_ref,
           foc_pitch.velocity_mech);
#endif

    HAL_Delay(100); // 10Hz print rate
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
    /* Chạy Vòng lặp kín tương ứng với TEST_MODE */
#if (TEST_MODE == TEST_MODE_VELOCITY)
    FOC_VelocityLoop(&foc_pitch, pitch_enc.angle_rad, target_velocity_rad_s);
#elif (TEST_MODE == TEST_MODE_POSITION)
    FOC_PositionLoop(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
#endif
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
