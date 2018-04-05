/**
 ******************************************************************************
 * @file    ../stm32f103-hal-rtc-time-date/src/main.c
 * @author  Erwin Ouyang
 * @version 1.0
 * @date    7 Feb 2017
 * @brief   This example shows how to use RTC HAL API to configure time and
 *          date.
 ******************************************************************************
 * Copyright (c) 2017 Erwin Ouyang
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef uartHandle;
RTC_HandleTypeDef rtcHandle;
RTC_TimeTypeDef currentTime;
RTC_DateTypeDef currentDate;

/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void UART_Config(void);
void RTC_Config(void);
void RTC_TimeDate_Config(void);
void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*## HAL initialization ##################################################*/
	HAL_Init();

	/*## System clocks initialization ########################################*/
	/* Set the SYSCLK at maximum frequency (72 MHz) */
	RCC_SystemClock_Config();

	/*## GPIO initialization #################################################*/
	GPIO_Output_Config();

	/*## UART initialization #################################################*/
	UART_Config();

	/*## RTC initialization ##################################################*/
	RTC_Config();
	/* Time and date configuration */
	RTC_TimeDate_Config();

	/*## Main loop ###########################################################*/
	while (1)
	{
		HAL_RTC_GetTime(&rtcHandle, &currentTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&rtcHandle, &currentDate, RTC_FORMAT_BIN);
		/* Print current date and time (date time format: yyyy-MM-dd hh:mm:ss)*/
		printf("20%i-%i-%i %i:%i:%i\n", currentDate.Year, currentDate.Month,
				currentDate.Date, currentTime.Hours, currentTime.Minutes,
				currentTime.Seconds);
		HAL_Delay(1000);
	}
}

/**
  * @brief  Re-targets the C library printf function to the UART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&uartHandle, (uint8_t*)&ch, 1, 0xFFFF);

	return ch;
}

/**
  * @brief  System clock configuration:
  *             System clock source = PLL (HSE)
  *             SYSCLK(Hz)          = 72000000
  *             HCLK(Hz)            = 72000000
  *             AHB prescaler       = 1
  *             APB1 prescaler      = 2
  *             APB2 prescaler      = 1
  *             HSE frequency(Hz)   = 8000000
  *             HSE PREDIV1         = 1
  *             PLLMUL              = 9
  *             Flash latency(WS)   = 2
  * @param  None
  * @retval None
  */
void RCC_SystemClock_Config(void)
{
	RCC_ClkInitTypeDef rccClkInit;
	RCC_OscInitTypeDef rccOscInit;

	/*## STEP 1: Configure HSE and PLL #######################################*/
	rccOscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	rccOscInit.HSEState       = RCC_HSE_ON;
	rccOscInit.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	rccOscInit.PLL.PLLState   = RCC_PLL_ON;
	rccOscInit.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	rccOscInit.PLL.PLLMUL     = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&rccOscInit) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Configure SYSCLK, HCLK, PCLK1, and PCLK2 ####################*/
	rccClkInit.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	rccClkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	rccClkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	rccClkInit.APB2CLKDivider = RCC_HCLK_DIV1;
	rccClkInit.APB1CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&rccClkInit, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  GPIO configuration:
  *             GPIO  = GPIOB
  *             Pin   = PB6, PB7, PB8, PB9
  *             Mode  = Output push-pull
  *             Speed = Low
  * @param  None
  * @retval None
  */
void GPIO_Output_Config(void)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	gpioInit.Pin   = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	gpioInit.Mode  = GPIO_MODE_OUTPUT_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpioInit);
}

/**
  * @brief  UART configuration:
  *             UART             = UART2
  *             Baud rate        = 9600
  *             Word length      = 8
  *             Stop bits        = 1
  *             Parity           = None
  *             Mode             = TX
  *             Hardware control = None
  * @param  None
  * @retval None
  */
void UART_Config(void)
{
	UART_InitTypeDef UART_InitStruct;

	/*## STEP 1: Configure UART ##############################################*/
	UART_InitStruct.BaudRate   = 9600;
	UART_InitStruct.WordLength = UART_WORDLENGTH_8B;
	UART_InitStruct.StopBits   = UART_STOPBITS_1;
	UART_InitStruct.Parity 	   = UART_PARITY_NONE;
	UART_InitStruct.Mode 	   = UART_MODE_TX;
	UART_InitStruct.HwFlowCtl  = UART_HWCONTROL_NONE;
	uartHandle.Instance        = USART2;
	uartHandle.Init	           = UART_InitStruct;
	if (HAL_UART_Init(&uartHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  UART MSP configuration callback.
 * @param  None
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA2 for UART TX */
	gpioInit.Mode  = GPIO_MODE_AF_PP;
	gpioInit.Pin   = GPIO_PIN_2;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
 	HAL_GPIO_Init(GPIOA, &gpioInit);
}

/**
  * @brief  RTC configuration
  * @param  None
  * @retval None
  */
void RTC_Config(void)
{
	/*## STEP 1: Configure RTC ###############################################*/
	rtcHandle.Instance          = RTC;
	rtcHandle.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
	rtcHandle.Init.OutPut       = RTC_OUTPUTSOURCE_NONE;
	if (HAL_RTC_Init(&rtcHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  RTC MSP configuration callback.
 * @param  hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
	RCC_OscInitTypeDef rccOscInit;
	RCC_PeriphCLKInitTypeDef rccPeriphCLKInit;

	/*## STEP 1: Configure LSI ###############################################*/
	rccOscInit.OscillatorType = RCC_OSCILLATORTYPE_LSI;
	rccOscInit.PLL.PLLState   = RCC_PLL_NONE;
	rccOscInit.LSIState       = RCC_LSI_ON;
	if(HAL_RCC_OscConfig(&rccOscInit) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Configure RCC peripheral ####################################*/
	rccPeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	rccPeriphCLKInit.RTCClockSelection    = RCC_RTCCLKSOURCE_LSI;
	if(HAL_RCCEx_PeriphCLKConfig(&rccPeriphCLKInit) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_RCC_RTC_ENABLE();
}

/**
  * @brief  Time and date configuration
  * @param  None
  * @retval None
  */
void RTC_TimeDate_Config(void)
{
	RTC_TimeTypeDef rtcTimeStruct;
	RTC_DateTypeDef rtcDateStruct;

	/*## STEP 1: Configure time ##############################################*/
	/* Set time: 23:59:55 */
	rtcTimeStruct.Hours   = 0x23;
	rtcTimeStruct.Minutes = 0x59;
	rtcTimeStruct.Seconds = 0x55;
	if (HAL_RTC_SetTime(&rtcHandle, &rtcTimeStruct, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Configure date ##############################################*/
	/* Set date: Monday, December 31th 2016 */
	rtcDateStruct.Year    = 0x16;
	rtcDateStruct.Month   = RTC_MONTH_DECEMBER;
	rtcDateStruct.Date    = 0x31;
	rtcDateStruct.WeekDay = RTC_WEEKDAY_SATURDAY;
	if(HAL_RTC_SetDate(&rtcHandle, &rtcDateStruct, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
	/* Turn red LED on */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	while (1);
}

/******************************** END OF FILE *********************************/
/******************************************************************************/
