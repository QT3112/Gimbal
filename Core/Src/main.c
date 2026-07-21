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
#include "foc.h"
#include "as5048a.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
  ENC_STATE_IDLE = 0,
  ENC_STATE_PITCH_CMD,
  ENC_STATE_PITCH_READ
} Encoder_State_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* ----------------- CHỌN CHẾ ĐỘ HOẠT ĐỘNG ----------------- */
#define USE_POSITION_LOOP 1  /* 1: Chạy vòng vị trí (giữ góc), 0: Chạy vòng tốc độ/dòng điện */

AS5048A_Handle_t pitch_enc;
FOC_Handle_t foc_pitch;

/* Biến lưu góc mục tiêu (rad) */
volatile float target_pitch_angle = 0.0f;

float pitch_offset_a = 0.0f;
float pitch_offset_b = 0.0f;

/* Biến lưu trữ raw ADC để debug */
volatile uint32_t log_raw_ia = 0;
volatile uint32_t log_raw_ib = 0;

/* SPI DMA Buffers & States cho Encoder */
volatile Encoder_State_t enc_state = ENC_STATE_IDLE;
/* Khi dùng SPI 16-bit, buffer phải là mảng uint16_t. DMA sẽ truyền 16-bit nguyên khối.
 * Frame 1: 0xFFFF (Đọc góc), Frame 2: 0xC000 (NOP để lấy kết quả) */
uint16_t enc_tx_buf[2] = { 0xFFFF, 0xC000 };
uint16_t enc_rx_buf[2];

/* Cấu hình phần cứng mạch dòng */
#define GAIN_DRV   10.0f
#define SHUNT_RES  0.005f
#define VOLTAGE_LIMIT 6.0f
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
  /* USER CODE BEGIN 2 */
  /* --- SETUP PHẦN CỨNG PITCH MOTOR --- */
  
  /* Bật DRV8302 (EN_GATE - PB1) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(10);

  /* Cấu hình Encoder CS ban đầu ở mức cao (PB12) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_Delay(10);

  AS5048A_Init(&pitch_enc, &hspi1, GPIOB, GPIO_PIN_12);

  /* Khởi tạo FOC (Ts vòng ngoài = 0.0005s vì TIM6 chạy ở 2kHz) */
  FOC_Init(&foc_pitch, &htim1, TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, 
           PWM_PERIOD, MOTOR_POLE_PAIRS, VOLTAGE_LIMIT, 0.0005f);
           
  /* 1. Vòng lặp Dòng điện (Current Loop) ở 20kHz */
  FOC_SetPID_D(&foc_pitch, 0.05f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_pitch, 0.05f, 100.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetCurrentLimit(&foc_pitch, 0.2f); // Giới hạn dòng (Quyết định lực cứng tối đa) - 0.35A là khá mềm

  /* 2. Vòng lặp Tốc độ (Velocity Loop) - Nhận target_vel và xuất ra Iq_ref */
  FOC_SetLPF_Vel(&foc_pitch, 0.9f);      // Bộ lọc LPF (alpha = 0.9 -> lọc mạnh 90% nhiễu)
  FOC_SetPID_Vel(&foc_pitch, 0.05f, 1.0f, 0.0f, -0.35f, 0.35f);

  FOC_SetPID_Pos(&foc_pitch, 2.0f, 0.01f, 0.0f, -3.0f, 3.0f);
 
  /* Bật vòng lặp dòng điện tại tần số 20kHz */
  FOC_EnableCurrentLoop(&foc_pitch, 1.0f / 20000.0f);

  /* Calibrate ADC offsets trước khi xuất PWM */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_Delay(10);

  /* Start PWM 3 Pha + Kích hoạt Trigger ADC từ TIM1_CH4 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); // Trigger ADC (TRGO)
  HAL_Delay(10);

  /* Tính toán dòng offset */
  float sum_a = 0.0f, sum_b = 0.0f;
  uint8_t adc_timeout_err = 0;
  HAL_ADCEx_InjectedStart(&hadc1);
  for (int i = 0; i < 1000; i++) {
    uint32_t start_wait = HAL_GetTick();
    while (!__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_JEOS)) {
        if (HAL_GetTick() - start_wait > 5) {
            adc_timeout_err = 1;
            break;
        }
    }
    if (adc_timeout_err) break;
    
    __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOS);
    sum_a += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1) * 3.3f / 4096.0f;
    sum_b += (float)HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2) * 3.3f / 4096.0f;
  }
  HAL_ADCEx_InjectedStop(&hadc1);
  
  if (adc_timeout_err) {
      printf("[ERROR] ADC Calibration Timeout! Kiem tra Trigger TIM1_CH4.\r\n");
  } else {
      pitch_offset_a = sum_a / 1000.0f;
      pitch_offset_b = sum_b / 1000.0f;
      printf("[INFO] Calibrated Offset A: %.3fV | B: %.3fV\r\n", pitch_offset_a, pitch_offset_b);
      
      /* KIỂM TRA AN TOÀN: Nếu DRV8302 chưa được cấp nguồn (hoặc lỗi), offset sẽ ~0V thay vì 1.65V */
      if (pitch_offset_a < 1.0f || pitch_offset_a > 2.3f || pitch_offset_b < 1.0f || pitch_offset_b > 2.3f) {
          printf("\r\n[FATAL ERROR] ADC Offset qua thap (0V) hoac khong hop le! \r\n");
          printf("--> NGUYEN NHAN: Ban CHUA BAT NGUON 12V/24V cho mach DRV8302 truoc khi STM32 khoi dong!\r\n");
          printf("--> CACH SUA: Hay bat nguon Motor truoc, sau do an nut RESET tren STM32 de Hieu chuan lai.\r\n");
          while(1) {
              HAL_Delay(1000); // HALT he thong de bao ve
          }
      }
  }

  /* =========================================================
   * BƯỚC 3: ALIGN VÀ DÒ CHIỀU ENCODER (Auto-calibration)
   * ========================================================= */
  printf("[INFO] Aligning Motor and Detecting Direction...\r\n");
  
  foc_pitch.enabled = 1; // Bật FOC để cấp PWM
  
  /* 1. Cấp nguồn Vd = 3.0V tại góc điện 0 độ để khoá Rotor vào vị trí mốc */
  FOC_AlignD(&foc_pitch, 3.0f); 
  HAL_Delay(800); // Đợi rotor ổn định
  
  /* Đọc góc Encoder ban đầu */
  float start_angle = 0.0f;
  if (AS5048A_ReadAngle(&pitch_enc) == AS5048A_OK) {
      start_angle = (float)pitch_enc.raw_angle * (6.28318530718f / 16384.0f);
  }
  
  /* 2. Quét từ trường điện đi 1 vòng tròn (2*PI) để xem Encoder chạy theo chiều nào */
  printf("[INFO] Sweeping electrical field...\r\n");
  for (int i = 1; i <= 100; i++) {
      foc_pitch.angle_elec = (float)i * (6.28318530718f / 100.0f);
      FOC_Update(&foc_pitch); // Cập nhật PWM theo Vd và góc điện mới
      HAL_Delay(5);           // Tốc độ quét (quét 1 vòng mất 500ms)
  }
  HAL_Delay(500); // Chờ rotor ổn định ở điểm cuối
  
  /* Đọc góc Encoder lúc sau */
  float end_angle = 0.0f;
  if (AS5048A_ReadAngle(&pitch_enc) == AS5048A_OK) {
      end_angle = (float)pitch_enc.raw_angle * (6.28318530718f / 16384.0f);
  }
  
  /* 3. Tính toán độ lệch góc và phán xét chiều quay */
  float delta_angle = end_angle - start_angle;
  if (delta_angle > 3.14159265f) delta_angle -= 6.28318530718f;
  if (delta_angle < -3.14159265f) delta_angle += 6.28318530718f;
  
  /* Chẩn đoán số Pole Pairs thực tế: 
   * 1 vòng điện = 360 / PolePairs độ cơ học 
   * VD: delta_angle ~ 0.44 rad (25 độ) => PolePairs = 14 */
  float delta_deg = delta_angle * (180.0f / 3.14159265f);
  printf("[INFO] Sweep Delta: %.2f deg (%.2f rad)\r\n", delta_deg, delta_angle);
  
  if (delta_angle > 0.0f) {
      foc_pitch.sensor_direction = 1;
      printf("[INFO] Sensor Direction: FORWARD (1)\r\n");
  } else {
      foc_pitch.sensor_direction = -1;
      printf("[INFO] Sensor Direction: REVERSE (-1)\r\n");
  }
  
  /* 4. Khóa Rotor lại vị trí 0 độ điện lần nữa để lấy mốc (Offset) chuẩn xác */
  FOC_AlignD(&foc_pitch, 3.0f);
  HAL_Delay(500);
  
  /* Cập nhật lại góc hiện tại với chiều quay vừa phát hiện */
  if (AS5048A_ReadAngle(&pitch_enc) == AS5048A_OK) {
      float final_rad = (float)pitch_enc.raw_angle * (6.28318530718f / 16384.0f);
      if (foc_pitch.sensor_direction == -1) {
          pitch_enc.angle_rad = 6.28318530718f - final_rad;
      } else {
          pitch_enc.angle_rad = final_rad;
      }
      printf("[INFO] Encoder OK! Angle: %.2f rad\r\n", pitch_enc.angle_rad);
  }
  
  /* Lưu offset */
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);
  printf("[INFO] FOC Align Complete. Offset: %.2f rad\r\n", foc_pitch.angle_offset);

  /* Setup góc mặc định 45 độ (0.785398 rad) để motor tự động đi về góc này */
  target_pitch_angle = 45.0f * (3.14159265f / 180.0f);
  printf("[INFO] Target Angle set to: %.2f rad (45 deg)\r\n", target_pitch_angle);
  FOC_Start(&foc_pitch, pitch_enc.angle_rad);

  HAL_ADCEx_InjectedStart_IT(&hadc1);    // Trigger đọc dòng phản hồi 
  HAL_TIM_Base_Start_IT(&htim6);         // Ngắt TIMER6 để đọc SPI - 2kHz 
  HAL_TIM_Base_Start_IT(&htim7);         // Ngắt TIMER7 để xử lý thuật toán FOC (position, velocity) - 2kHz

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_print = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (HAL_GetTick() - last_print >= 100) {
      last_print = HAL_GetTick();
      
      /* Tính toán lại I_alpha, I_beta để xuất log */
      FOC_AlphaBeta_t I_ab = FOC_Clarke(foc_pitch.Ia, foc_pitch.Ib, foc_pitch.Ic);
      
      /* In ra Terminal ở dạng dễ đọc trực tiếp */
      printf("[ENCODER] Pitch Angle: %.2f rad | %.2f deg\r\n", pitch_enc.angle_rad, pitch_enc.angle_deg);
      printf("[ADC RAW] raw_ia: %lu, raw_ib: %lu\r\n", log_raw_ia, log_raw_ib);
      printf("[PHASE I] Ia: %.3f, Ib: %.3f, Ic: %.3f\r\n", foc_pitch.Ia, foc_pitch.Ib, foc_pitch.Ic);
      printf("[CLARKE ] I_alpha: %.3f, I_beta: %.3f\r\n", I_ab.alpha, I_ab.beta);
      printf("[PARK   ] Id_meas: %.3f, Iq_meas: %.3f\r\n", foc_pitch.Id_meas, foc_pitch.Iq_meas);
      printf("----------------------------------------\r\n");
    }
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


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    if (enc_state == ENC_STATE_PITCH_CMD) {
      /* Kéo CS lên Mức Cao kết thúc Frame 1 */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); 
      
      /* Chuyển sang Bước 2: Gửi NOP (0xC000) để clock dữ liệu về */
      enc_state = ENC_STATE_PITCH_READ;
      
      /* Kéo CS xuống Mức Thấp để gửi Frame 2 */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
      HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&enc_tx_buf[1], (uint8_t*)&enc_rx_buf[1], 1);
      
    } else if (enc_state == ENC_STATE_PITCH_READ) {
      /* Kéo CS lên Mức Cao kết thúc Frame 2 */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); 
      
      /* Dữ liệu góc trả về nằm ở Frame 2 (enc_rx_buf[1]) của mảng nhận */
      uint16_t rx_val = enc_rx_buf[1];
      
      /* Kiểm tra chẵn lẻ (Parity) và cờ lỗi (Error Flag) 
       * LOẠI TRỪ các giá trị glitch rác (0x0000, 0xFFFF) vì chúng vô tình thoả mãn Parity! */
      if (rx_val != 0x0000 && rx_val != 0xFFFF && AS5048A_CheckParity(rx_val) && !(rx_val & AS5048A_EF_BIT)) {
        uint16_t new_raw = rx_val & AS5048A_DATA_MASK;
        
        /* Tính delta cơ học thô để lọc glitch */
        int32_t delta_raw = (int32_t)new_raw - (int32_t)pitch_enc.raw_angle;
        if (delta_raw > 8192) delta_raw -= 16384;
        if (delta_raw < -8192) delta_raw += 16384;
        
        /* Lọc Glitch: Nếu nhảy > 150 raw (khoảng 3.3 độ/0.5ms) thì coi là nhiễu.
         * Dùng bộ đếm reject_count để tránh bị khoá vĩnh viễn nếu góc thực sự bị dịch chuyển. */
        static uint8_t startup_skip = 10;
        static uint8_t reject_count = 0;
        
        if (startup_skip > 0 || abs(delta_raw) < 150 || reject_count > 10) {
            if (startup_skip > 0) startup_skip--;
            pitch_enc.raw_angle = new_raw;
            reject_count = 0;
        
            /* Tự động đảo chiều (dựa vào Auto-Calibration) */
            if (foc_pitch.sensor_direction == -1) {
                pitch_enc.angle_deg = 360.0f - ((float)pitch_enc.raw_angle * (360.0f / 16384.0f));
                pitch_enc.angle_rad = 6.28318530718f - ((float)pitch_enc.raw_angle * (6.28318530718f / 16384.0f));
            } else {
                pitch_enc.angle_deg = ((float)pitch_enc.raw_angle * (360.0f / 16384.0f));
                pitch_enc.angle_rad = ((float)pitch_enc.raw_angle * (6.28318530718f / 16384.0f));
            }
        } else {
            reject_count++;
        }

        /* CHỈ LƯU GÓC TẠI ĐÂY (Ngắt SPI kết thúc cực nhanh, giải phóng CPU) 
         * Việc tính toán FOC_RunPosition sẽ do TIM7 (chạy sau đó) đảm nhận. */
      }
      
      enc_state = ENC_STATE_IDLE;
    }
  }
}

/* Hàm ngắt Timer định kỳ (TIM6 cho đọc SPI, TIM7 cho tính PID Vị trí) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    if (enc_state == ENC_STATE_IDLE) {
      enc_state = ENC_STATE_PITCH_CMD;
      /* Bước 1: Kéo CS xuống Mức Thấp để gửi Lệnh Đọc Góc (0xFFFF) */
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
      HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&enc_tx_buf[0], (uint8_t*)&enc_rx_buf[0], 1);
#if USE_POSITION_LOOP
      FOC_RunPosition(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
#else
      FOC_RunVelocity(&foc_pitch, pitch_enc.angle_rad, 0.0f);
#endif
    }
  }
  else if (htim->Instance == TIM7) {
      /* Ngắt TIM7 (1kHz hoặc 2kHz) chuyên dụng để tính toán PID Vị trí và Tốc độ.
       * Ngắt này có Priority thấp hơn SPI và ADC, nên nó sẽ "thong thả" tính toán
       * mà không làm ảnh hưởng đến thời gian thực của ADC (20kHz). */
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    /* Đọc dòng pha Ia và Ib từ cảm biến (Rank 1 & Rank 2 trên PA0, PA1) */
    uint32_t raw_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    
    log_raw_ia = raw_ia;
    log_raw_ib = raw_ib;
    float volt_a = (float)raw_ia * 3.3f / 4096.0f;
    float volt_b = (float)raw_ib * 3.3f / 4096.0f;
    float current_a = (volt_a - pitch_offset_a) / (GAIN_DRV * SHUNT_RES);
    float current_b = (volt_b - pitch_offset_b) / (GAIN_DRV * SHUNT_RES);
    float true_elec = pitch_enc.angle_rad * foc_pitch.pole_pairs - foc_pitch.angle_offset;
    true_elec = fmodf(true_elec, 6.28318530718f);
    if (true_elec < 0.0f) true_elec += 6.28318530718f;
  
    float phase_err = true_elec - foc_pitch.angle_elec;
    if (phase_err > 3.14159265f) phase_err -= 6.28318530718f;
    if (phase_err < -3.14159265f) phase_err += 6.28318530718f;
    
    foc_pitch.angle_elec += phase_err * 0.1f + (foc_pitch.velocity_mech * foc_pitch.pole_pairs * (1.0f / 20000.0f));
    
    if (foc_pitch.angle_elec >= 6.28318530718f) foc_pitch.angle_elec -= 6.28318530718f;
    else if (foc_pitch.angle_elec < 0.0f) foc_pitch.angle_elec += 6.28318530718f;
    FOC_UpdateCurrentLoop(&foc_pitch, current_a, current_b);
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
