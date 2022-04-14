/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//typedef struct {
//	char s1 = "Err";
//	char s2 = "Standby";
//	char s3 = "Driving";
//	char s4 = "Turning";
//} status_str;

//typedef struct {
//	uint32_t IR1, IR2, IR3, IR4, IR5;
//} ir_t;
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
uint8_t SPIRead(uint8_t address);
void SPIWrite(uint8_t address, uint8_t data);
void gyroInit(void);
uint8_t get_bt_msg(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IR3_Pin GPIO_PIN_3
#define IR3_GPIO_Port GPIOF
#define IR2_Pin GPIO_PIN_5
#define IR2_GPIO_Port GPIOF
#define IR1_Pin GPIO_PIN_10
#define IR1_GPIO_Port GPIOF
#define IR5_Pin GPIO_PIN_0
#define IR5_GPIO_Port GPIOC
#define IR4_Pin GPIO_PIN_3
#define IR4_GPIO_Port GPIOC
#define SCK_Pin GPIO_PIN_5
#define SCK_GPIO_Port GPIOA
#define SDO_Pin GPIO_PIN_6
#define SDO_GPIO_Port GPIOA
#define SDA_Pin GPIO_PIN_7
#define SDA_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_12
#define IN1_GPIO_Port GPIOF
#define ENB_Pin GPIO_PIN_13
#define ENB_GPIO_Port GPIOF
#define ENA_Pin GPIO_PIN_15
#define ENA_GPIO_Port GPIOF
#define SPI_INT_Pin GPIO_PIN_9
#define SPI_INT_GPIO_Port GPIOE
#define IN3_Pin GPIO_PIN_11
#define IN3_GPIO_Port GPIOE
#define IN4_Pin GPIO_PIN_13
#define IN4_GPIO_Port GPIOE
#define SPI_CS_Pin GPIO_PIN_14
#define SPI_CS_GPIO_Port GPIOD
#define IN2_Pin GPIO_PIN_15
#define IN2_GPIO_Port GPIOD
#define BT_RX_Pin GPIO_PIN_9
#define BT_RX_GPIO_Port GPIOG
#define BT_TX_Pin GPIO_PIN_14
#define BT_TX_GPIO_Port GPIOG
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
