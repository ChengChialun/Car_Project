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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
//Error code is 2 bytes
#define OK							0
#define SYSTEM_ERROR				1
#define ERROR_I2C					10001
#define ERROR_I2C_BUSY  			10002

#define ERROR_IMU_INIT  			10003
#define ERROR_IMU_RESET  			10004
#define ERROR_IMU_WAKEUP  			10005
#define ERROR_IMU_GYRO_INIT 		10006
#define ERROR_IMU_ACCEL_INIT  		10007
#define ERROR_IMU_MAG_INIT  		10008
#define ERROR_IMU_BANK_SEL  		10009

#define ERROR_IMU_READ  			10011


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FRONT_RIGHT_TOF_Pin GPIO_PIN_2
#define FRONT_RIGHT_TOF_GPIO_Port GPIOC
#define FRONT_RIGHT_TOF_EXTI_IRQn EXTI2_IRQn
#define FRONT_LEFT_TOF_Pin GPIO_PIN_3
#define FRONT_LEFT_TOF_GPIO_Port GPIOC
#define FRONT_LEFT_TOF_EXTI_IRQn EXTI3_IRQn
#define ADC_VCC_Pin GPIO_PIN_4
#define ADC_VCC_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_5
#define STBY_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_4
#define AIN1_GPIO_Port GPIOC
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOC
#define BIN1_Pin GPIO_PIN_0
#define BIN1_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_1
#define BIN2_GPIO_Port GPIOB
#define Front_Ultrasonic_Pin GPIO_PIN_12
#define Front_Ultrasonic_GPIO_Port GPIOB
#define Rear_Ultrasonic_Pin GPIO_PIN_13
#define Rear_Ultrasonic_GPIO_Port GPIOB
#define REAR_RIGHT_TOF_Pin GPIO_PIN_14
#define REAR_RIGHT_TOF_GPIO_Port GPIOB
#define REAR_RIGHT_TOF_EXTI_IRQn EXTI15_10_IRQn
#define REAR_LEFT_TOF_Pin GPIO_PIN_15
#define REAR_LEFT_TOF_GPIO_Port GPIOB
#define REAR_LEFT_TOF_EXTI_IRQn EXTI15_10_IRQn
#define FLASH_CS_Pin GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_6
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
