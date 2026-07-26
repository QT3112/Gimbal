/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DRV1_SO1_Pin GPIO_PIN_0
#define DRV1_SO1_GPIO_Port GPIOA
#define DRV1_SO2_Pin GPIO_PIN_1
#define DRV1_SO2_GPIO_Port GPIOA
#define DRV2_SO1_Pin GPIO_PIN_2
#define DRV2_SO1_GPIO_Port GPIOA
#define DRV2_SO2_Pin GPIO_PIN_3
#define DRV2_SO2_GPIO_Port GPIOA
#define DRV3_INH_B_Pin GPIO_PIN_4
#define DRV3_INH_B_GPIO_Port GPIOA
#define DRV3_SO2_Pin GPIO_PIN_5
#define DRV3_SO2_GPIO_Port GPIOA
#define ENC_SPI1_MISO_Pin GPIO_PIN_6
#define ENC_SPI1_MISO_GPIO_Port GPIOA
#define ENC_SPI1_MOSI_Pin GPIO_PIN_7
#define ENC_SPI1_MOSI_GPIO_Port GPIOA
#define DRV3_INH_C_Pin GPIO_PIN_0
#define DRV3_INH_C_GPIO_Port GPIOB
#define DRV3_SO1_Pin GPIO_PIN_2
#define DRV3_SO1_GPIO_Port GPIOB
#define ENC_PITCH_CS_Pin GPIO_PIN_12
#define ENC_PITCH_CS_GPIO_Port GPIOB
#define ENC_ROLL_CS_Pin GPIO_PIN_13
#define ENC_ROLL_CS_GPIO_Port GPIOB
#define ENC_YAW_CS_Pin GPIO_PIN_14
#define ENC_YAW_CS_GPIO_Port GPIOB
#define IMU_PAYLOAD_CS_Pin GPIO_PIN_15
#define IMU_PAYLOAD_CS_GPIO_Port GPIOB
#define IMU_FRAME_CS_Pin GPIO_PIN_6
#define IMU_FRAME_CS_GPIO_Port GPIOC
#define DRV1_INH_A_Pin GPIO_PIN_8
#define DRV1_INH_A_GPIO_Port GPIOA
#define DRV1_INH_B_Pin GPIO_PIN_9
#define DRV1_INH_B_GPIO_Port GPIOA
#define DRV1_INH_C_Pin GPIO_PIN_10
#define DRV1_INH_C_GPIO_Port GPIOA
#define DRV2_INH_B_Pin GPIO_PIN_14
#define DRV2_INH_B_GPIO_Port GPIOA
#define DRV2_INH_A_Pin GPIO_PIN_15
#define DRV2_INH_A_GPIO_Port GPIOA
#define IMU_SPI3_SCK_Pin GPIO_PIN_10
#define IMU_SPI3_SCK_GPIO_Port GPIOC
#define IMU_SPI3_MISO_Pin GPIO_PIN_11
#define IMU_SPI3_MISO_GPIO_Port GPIOC
#define ENC_SPI1_SCK_Pin GPIO_PIN_3
#define ENC_SPI1_SCK_GPIO_Port GPIOB
#define DRV3_INH_A_Pin GPIO_PIN_4
#define DRV3_INH_A_GPIO_Port GPIOB
#define IMU_SPI3_MOSI_Pin GPIO_PIN_5
#define IMU_SPI3_MOSI_GPIO_Port GPIOB
#define DRV2_INH_C_Pin GPIO_PIN_9
#define DRV2_INH_C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
