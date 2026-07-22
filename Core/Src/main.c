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
#include "foc_v1.h"
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
  /* 1. Bật ngắt Timer 6 kích hoạt đọc SPI DMA encoder ở tần số 1kHz */
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(100); // Chờ lấy mẫu vài frame góc ban đầu từ AS5048A

  /* 2. Khởi tạo FOC Handle cho Pitch Motor */
  FOC_Init(&foc_pitch, &htim1,
           TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3,
           PWM_PERIOD, MOTOR_POLE_PAIRS,
           VOLTAGE_LIMIT, 2.0f,
           0.001f, 0.00005f);

  /* Cấu hình cảm biến dòng & chiều encoder */
  FOC_ConfigureCurrentSense(&foc_pitch, SHUNT_RES, GAIN_DRV, 3.3f);
  FOC_SetSensorDirection(&foc_pitch, 1);

  /* Cấu hình các bộ PID:
   * - Position PID: P=15.0, I=0.0, D=0.2 (Giới hạn tốc độ mục tiêu ±20 rad/s)
   * - Velocity PID: P=0.2, I=1.5, D=0.001 (Giới hạn điện áp torque ±VOLTAGE_LIMIT)
   */
  FOC_SetPID_POS(&foc_pitch, 15.0f, 0.0f, 0.2f, -20.0f, 20.0f);
  FOC_SetPID_VEL(&foc_pitch, 0.2f, 1.5f, 0.001f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_D(&foc_pitch, 1.0f, 50.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetPID_Q(&foc_pitch, 1.0f, 50.0f, 0.0f, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
  FOC_SetLPF_Vel(&foc_pitch, 0.9f);

  /* Bật PWM các kênh điều khiển motor */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

  foc_pitch.enabled = 1;

  /* 3. Căn chỉnh góc Align D-Axis: Áp điện áp Vd = 1.5V kéo rotor về góc điện 0 trong 1.5s */
  FOC_AlignD(&foc_pitch, 1.5f);
  HAL_Delay(1500);

  /* Lưu góc encoder tại vị trí cân bằng D-Axis làm offset zero */
  FOC_CalibrateAngle(&foc_pitch, pitch_enc.angle_rad);

  /* Trả điện áp về 0 */
  FOC_SetVoltage(&foc_pitch, 0.0f, 0.0f);
  HAL_Delay(200);

  /* 4. Đặt vị trí mục tiêu 45 độ cơ khí (0.785398 rad) và kích hoạt FOC Cascade Loop */
  target_pitch_angle = 45.0f * (FOC_PI / 180.0f);
  FOC_Start(&foc_pitch, pitch_enc.angle_rad);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    printf("Target: 45.0 deg | Enc: %.2f deg (%.3f rad) | Vq: %.2fV\r\n", 
           pitch_enc.angle_deg, pitch_enc.angle_rad, foc_pitch.Vq_ref);
    HAL_Delay(100); // 10Hz print rate
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


/* SPI DMA Buffers cho Encoder AS5048A */
uint16_t enc_tx_cmd = 0xFFFF;
uint16_t enc_rx_val = 0;
volatile uint8_t enc_busy = 0;

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1) {
    /* Đã hoàn thành nhận 1 word 16-bit từ AS5048A -> Kéo CS lên HIGH */
    HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_SET);
    enc_busy = 0;

    uint16_t data = enc_rx_val;
    if (AS5048A_CheckParity(data) && !(data & AS5048A_EF_BIT)) {
      pitch_enc.raw_angle = data & AS5048A_DATA_MASK;
      pitch_enc.angle_rad = (float)pitch_enc.raw_angle * (6.28318530718f / AS5048A_MAX_VALUE);
      pitch_enc.angle_deg = (float)pitch_enc.raw_angle * (360.0f / AS5048A_MAX_VALUE);

      /* Thực thi vòng lặp vị trí FOC (Cascade Control Loop) tại tần số 1kHz */
      if (foc_pitch.enabled) {
        FOC_PositionLoop(&foc_pitch, pitch_enc.angle_rad, target_pitch_angle);
      }
    }
  }
}

/* Hàm ngắt Timer định kỳ (TIM6 1kHz cho đọc SPI DMA) */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    if (!enc_busy) {
      enc_busy = 1;
      HAL_GPIO_WritePin(ENC_PITCH_CS_GPIO_Port, ENC_PITCH_CS_Pin, GPIO_PIN_RESET);
      HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t*)&enc_tx_cmd, (uint8_t*)&enc_rx_val, 1);
    }
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if (hadc->Instance == ADC1) {
    log_raw_ia = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    log_raw_ib = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);

    FOC_UpdateCurrentLoopADC(&foc_pitch, log_raw_ia, log_raw_ib);
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
