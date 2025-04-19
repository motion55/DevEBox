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
#include <stm32_adafruit_lcd.h>

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
#define TFT_INT_Pin GPIO_PIN_3
#define TFT_INT_GPIO_Port GPIOE
#define NRESET_OUT_Pin GPIO_PIN_13
#define NRESET_OUT_GPIO_Port GPIOC
#define PUSH_BUTTON1_Pin GPIO_PIN_1
#define PUSH_BUTTON1_GPIO_Port GPIOC
#define PUSH_BUTTON2_Pin GPIO_PIN_2
#define PUSH_BUTTON2_GPIO_Port GPIOC
#define PUSH_BUTTON3_Pin GPIO_PIN_3
#define PUSH_BUTTON3_GPIO_Port GPIOC
#define PUSH_BUTTON4_Pin GPIO_PIN_0
#define PUSH_BUTTON4_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_1
#define LED_GPIO_Port GPIOA
#define UART2_TX_Pin GPIO_PIN_2
#define UART2_TX_GPIO_Port GPIOA
#define SERVO_OUT2_Pin GPIO_PIN_3
#define SERVO_OUT2_GPIO_Port GPIOA
#define SERVO_OUT1_Pin GPIO_PIN_5
#define SERVO_OUT1_GPIO_Port GPIOA
#define DCMI_PWDN_Pin GPIO_PIN_7
#define DCMI_PWDN_GPIO_Port GPIOA
#define ADC_INP_Pin GPIO_PIN_4
#define ADC_INP_GPIO_Port GPIOC
#define ADC_INN_Pin GPIO_PIN_5
#define ADC_INN_GPIO_Port GPIOC
#define TFT_BL_Pin GPIO_PIN_0
#define TFT_BL_GPIO_Port GPIOB
#define TFT_RS_Pin GPIO_PIN_1
#define TFT_RS_GPIO_Port GPIOB
#define FMC_D4_Pin GPIO_PIN_7
#define FMC_D4_GPIO_Port GPIOE
#define FMC_D5_Pin GPIO_PIN_8
#define FMC_D5_GPIO_Port GPIOE
#define FMC_D6_Pin GPIO_PIN_9
#define FMC_D6_GPIO_Port GPIOE
#define FMC_D7_Pin GPIO_PIN_10
#define FMC_D7_GPIO_Port GPIOE
#define FMC_D8_Pin GPIO_PIN_11
#define FMC_D8_GPIO_Port GPIOE
#define FMC_D9_Pin GPIO_PIN_12
#define FMC_D9_GPIO_Port GPIOE
#define FMC_D10_Pin GPIO_PIN_13
#define FMC_D10_GPIO_Port GPIOE
#define FMC_D11_Pin GPIO_PIN_14
#define FMC_D11_GPIO_Port GPIOE
#define FMC_D12_Pin GPIO_PIN_15
#define FMC_D12_GPIO_Port GPIOE
#define TFT_CS_Pin GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOB
#define TFT_SCK_Pin GPIO_PIN_13
#define TFT_SCK_GPIO_Port GPIOB
#define TFT_MISO_Pin GPIO_PIN_14
#define TFT_MISO_GPIO_Port GPIOB
#define TFT_MOSI_Pin GPIO_PIN_15
#define TFT_MOSI_GPIO_Port GPIOB
#define FMC_D13_Pin GPIO_PIN_8
#define FMC_D13_GPIO_Port GPIOD
#define FMC_D14_Pin GPIO_PIN_9
#define FMC_D14_GPIO_Port GPIOD
#define FMC_D15_Pin GPIO_PIN_10
#define FMC_D15_GPIO_Port GPIOD
#define FMC_DC_Pin GPIO_PIN_11
#define FMC_DC_GPIO_Port GPIOD
#define FMC_D0_Pin GPIO_PIN_14
#define FMC_D0_GPIO_Port GPIOD
#define FMC_D1_Pin GPIO_PIN_15
#define FMC_D1_GPIO_Port GPIOD
#define FAN_OUT_Pin GPIO_PIN_8
#define FAN_OUT_GPIO_Port GPIOC
#define DCMI_XCLK_Pin GPIO_PIN_8
#define DCMI_XCLK_GPIO_Port GPIOA
#define UART1_TX_Pin GPIO_PIN_9
#define UART1_TX_GPIO_Port GPIOA
#define UART1_RX_Pin GPIO_PIN_10
#define UART1_RX_GPIO_Port GPIOA
#define SPI1_NCS_Pin GPIO_PIN_15
#define SPI1_NCS_GPIO_Port GPIOA
#define FMC_D2_Pin GPIO_PIN_0
#define FMC_D2_GPIO_Port GPIOD
#define FMC_D3_Pin GPIO_PIN_1
#define FMC_D3_GPIO_Port GPIOD
#define TFT_TE_Pin GPIO_PIN_2
#define TFT_TE_GPIO_Port GPIOD
#define FMC_NRD_Pin GPIO_PIN_4
#define FMC_NRD_GPIO_Port GPIOD
#define FMC_NWR_Pin GPIO_PIN_5
#define FMC_NWR_GPIO_Port GPIOD
#define UART2_RX_Pin GPIO_PIN_6
#define UART2_RX_GPIO_Port GPIOD
#define FMC_NCS_Pin GPIO_PIN_7
#define FMC_NCS_GPIO_Port GPIOD
#define SPI1_SCK_Pin GPIO_PIN_3
#define SPI1_SCK_GPIO_Port GPIOB
#define SPI1_MISO_Pin GPIO_PIN_4
#define SPI1_MISO_GPIO_Port GPIOB
#define SPI1_MOSI_Pin GPIO_PIN_5
#define SPI1_MOSI_GPIO_Port GPIOB
#define LCD_SCL_Pin GPIO_PIN_8
#define LCD_SCL_GPIO_Port GPIOB
#define LCD_SDA_Pin GPIO_PIN_9
#define LCD_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define	LCD_RS_GPIO_Port	TFT_RS_GPIO_Port
#define	LCD_RS_Pin      	TFT_RS_Pin

#ifdef TFT_CS_GPIO_Port
#define	LCD_CS_GPIO_Port	TFT_CS_GPIO_Port
#endif
#ifdef TFT_CS_Pin
#define	LCD_CS_Pin    	TFT_CS_Pin
#endif

#define	LCD_BL_GPIO_Port	TFT_BL_GPIO_Port
#define	LCD_BL_Pin 	    	TFT_BL_Pin

#define	CAMERA_RESET_GPIO_Port	NRESET_OUT_GPIO_Port
#define	CAMERA_RESET_Pin		NRESET_OUT_Pin

#define CAMERA_PWDN_GPIO_Port 	DCMI_PWDN_GPIO_Port
#define CAMERA_PWDN_Pin     	DCMI_PWDN_Pin

#define	huart_ADC	huart2
#define	huart_debug	huart1

#define	DMA_BUFFER_SECTION	\
		__attribute__((section(".DMABufferSection"))) __attribute__((aligned(32)))

#define	CAM_BUFFER_SECTION	\
		__attribute__((section(".CAM_Buffer_section")))

#define	SERVO_TIMER_HANDLE	htim2

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
