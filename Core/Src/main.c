/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "bdma.h"
#include "dcmi.h"
#include "dma.h"
#include "dma2d.h"
#include "i2c.h"
#include "memorymap.h"
#include "quadspi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include "debug_console.h"
#include "ov7670.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define	_USE_TFT_	1
#define	_USE_LCD_	1

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint32_t frame_counter;
extern uint8_t img_buffer[];
char LCD_Busy = 0;
uint32_t new_frame_counter = 0;

void LCD_IO_DmaTxCpltCallback(SPI_HandleTypeDef *hspi)
{
	LCD_Busy = 0;
}

void RefreshCamera()
{
	if ((!LCD_Busy)&&(new_frame_counter!=frame_counter))
	{
		if ((frame_counter-new_frame_counter)>1)
		{
			DebugPrint("\r\n %8lX %8lX", frame_counter, new_frame_counter);
		}
		LCD_Busy = 0;
		new_frame_counter = frame_counter;
		BSP_LCD_DrawRGB16Image(0, 0, 320, 240, (uint16_t *)img_buffer);
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SetServoPos(uint8_t servo_no, int16_t pos)
{
	if (pos>1000)
		pos = 1000;
	if (pos<-1000)
		pos = -1000;

	uint32_t duty = 300000l + (200l * pos);
	switch (servo_no) {
	case 1:
		((TIM_HandleTypeDef*)&SERVO_TIMER_HANDLE)->Instance->CCR1 = duty;
		break;
	case 2:
		((TIM_HandleTypeDef*)&SERVO_TIMER_HANDLE)->Instance->CCR4 = duty;
		break;
	}
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_BDMA_Init();
  MX_DMA_Init();
  MX_QUADSPI_Init();
  MX_ADC3_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();
  MX_DCMI_Init();
  MX_SPI2_Init();
  MX_DMA2D_Init();
  MX_I2C1_Init();
  MX_FMC_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
#if	_USE_TFT_
	BSP_LCD_Init();
	BSP_LCD_DisplayOn();
	{
		uint32_t size = 320l * 240 * 2;
		memset(img_buffer, 0, size);
	}
#endif
#if	_USE_LCD_
	lcd_init();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(CAMERA_PWDN_GPIO_Port, CAMERA_PWDN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CAMERA_RESET_GPIO_Port, CAMERA_RESET_Pin, GPIO_PIN_SET);

	DebugInit();
	{
		OV7670_Init(&hdcmi, &hi2c_dcmi, 0, 0);
		OV7670_Start();
	}

	{
		HAL_TIM_Base_Start(&htim2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
		SetServoPos(1,0);
		SetServoPos(2,0);
	}

	int counter = 0;
	int position = 1000;

	while (1)
	{
		counter++;
		if (counter>10)
		{
			counter = 0;
			position = -position;
			SetServoPos(1,position);
			SetServoPos(2,position);
			DebugPrint("\r\n SetServoPos(1,%d);", position);
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(100);
		DebugTask();
		RefreshCamera();
		//HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(100);
		DebugTask();
		RefreshCamera();
		uint32_t ADC_val = CalcTemperature();
		if (ADC_val > 0)
		{
			char buffer[40];
			sprintf(buffer, "\r\n ADC_val = %10ld", ADC_val);
			HAL_UART_Transmit(&huart_ADC, (uint8_t*) buffer, strlen(buffer), 100);
#if	_USE_TFT_
//			uint8_t *header = (uint8_t*) "      ADC_Val     ";
//			BSP_LCD_DisplayStringAtLine(2, header);
//			sprintf(buffer, "   %10ld   ", ADC_val);
//			BSP_LCD_DisplayStringAtLine(4, (uint8_t*) buffer);
#endif
#if	_USE_LCD_
			lcd_put_cur(0, 0);
			lcd_send_string("    ADC_Val     ");
			lcd_put_cur(1, 0);
			sprintf(buffer, "   %8ld   ", ADC_val);
			lcd_send_string(buffer);
#endif
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 20;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_5);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 192;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void DebugMain(uint32_t val)
{
	switch (val)
	{
	case 0:
	{
		DebugPrint("\r\n OV7670_Init(&hdcmi, &hi2c_dcmi, 0, 0);");
		OV7670_Init(&hdcmi, &hi2c_dcmi, 0, 0);
		OV7670_Start();
	}
		break;
	case 1:
	{
		uint8_t temp1, temp2;
		if (ov7670_read(OV7670_REG_PID, &temp1) == HAL_OK)
		{
			ov7670_read(OV7670_REG_VER, &temp2);
			DebugPrint("\r\n ReadID %02X %02X", temp1, temp2);
		}
		else
		{
			DebugPrint("\r\n ReadID Error!");
		}
	}
		break;
	case 2:
	{
		if (OV7670_isDriverBusy())
		{
			DebugPrint("\r\n OV7670_isDriver is Busy");
		}
		else
		{
			DebugPrint("\r\n OV7670_isDriver is NOY Busy");
		}
	}
		break;
	case 3:
	{
		DebugPrint("\r\n Start Servos");
		HAL_TIM_Base_Start(&htim2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	}
		break;
	case 4:
	{
		DebugPrint("\r\n Clearing img_buffer");
		uint32_t size = 320l * 240 * 2;
		memset(img_buffer, 0, size);
	}
		break;
	case 5:
	{
		DebugPrint("\r\n BSP_LCD_DrawRGB16Image");
		BSP_LCD_DrawRGB16Image(0, 0, 320, 240, (uint16_t *)img_buffer);
	}
		break;
	case 6:
	case 9:
	{
		uint8_t servo_no = (val - 3)/3;
		DebugPrint("\r\n SetServoPos(%d,1000);", servo_no);
		SetServoPos(servo_no,1000);
	}
		break;
	case 7:
	case 10:
	{
		uint8_t servo_no = (val - 4)/3;
		DebugPrint("\r\n SetServoPos(%d,0);", servo_no);
		SetServoPos(servo_no,0);
	}
		break;
	case 8:
	case 11:
	{
		uint8_t servo_no = (val - 5)/3;
		DebugPrint("\r\n SetServoPos(%d,-1000);", servo_no);
		SetServoPos(servo_no,-1000);
	}
		break;
	}
}
/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x38000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_64KB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = 0x30000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
