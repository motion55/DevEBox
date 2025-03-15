/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "spi.h"
#include "i2c.h"
#include "debug_console.h"
#include "stm32_adafruit_lcd.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//extern LCD_DrvTypeDef  *lcd_drv;

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
void DebugMain(uint32_t val);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOA
#define DCMI_PWRDN_Pin GPIO_PIN_7
#define DCMI_PWRDN_GPIO_Port GPIOA
#define DCMI_RESET_Pin GPIO_PIN_4
#define DCMI_RESET_GPIO_Port GPIOC
#define LCD_BL_Pin GPIO_PIN_0
#define LCD_BL_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_1
#define LCD_RS_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_12
#define LCD_CS_GPIO_Port GPIOB
#define LCD_SCK_Pin GPIO_PIN_13
#define LCD_SCK_GPIO_Port GPIOB
#define LCD_MISO_Pin GPIO_PIN_14
#define LCD_MISO_GPIO_Port GPIOB
#define LCD_MOSI_Pin GPIO_PIN_15
#define LCD_MOSI_GPIO_Port GPIOB
#define DCMI_XCLK_Pin GPIO_PIN_8
#define DCMI_XCLK_GPIO_Port GPIOA
#define LCD_SCL_Pin GPIO_PIN_8
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_9
#define LCD_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define	DISPL_DC_GPIO_Port	LCD_RS_GPIO_Port
#define	DISPL_DC_Pin      	LCD_RS_Pin

#ifdef LCD_CS_GPIO_Port
#define	DISPL_CS_GPIO_Port	LCD_CS_GPIO_Port
#endif
#ifdef LCD_CS_Pin
#define	DISPL_CS_Pin    	LCD_CS_Pin
#endif

#define	DISPL_LED_GPIO_Port	LCD_BL_GPIO_Port
#define	DISPL_LED_Pin		LCD_BL_Pin

#define	CAMERA_RESET_GPIO_Port	DCMI_RESET_GPIO_Port
#define	CAMERA_RESET_Pin		DCMI_RESET_Pin

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
