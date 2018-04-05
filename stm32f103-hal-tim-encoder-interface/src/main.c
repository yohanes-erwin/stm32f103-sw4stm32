/**
  ******************************************************************************
  * @file    ../stm32f103-hal-tim-encoder-interface/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    1 Feb 2017
  * @brief   This example code shows how to use TIM HAL API to count and
  *          determinate the rotation direction of rotary encoder.
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
TIM_HandleTypeDef timHandle;

uint16_t encoderCount;
uint8_t encoderDirection;

/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void UART_Config(void);
void TIM_Config(void);
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

	/*## TIM initialization ##################################################*/
	TIM_Config();

	/*## Main loop ###########################################################*/
	while (1)
	{
		/* Get encoder count from TIM CNT register */
		encoderCount = __HAL_TIM_GET_COUNTER(&timHandle);
		encoderDirection = __HAL_TIM_IS_TIM_COUNTING_DOWN(&timHandle);

		/* Send to serial monitor application */
		printf("Count,Direction=%i,%i\n", encoderCount, encoderDirection);
		HAL_Delay(500);
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
	/*## STEP 1: Configure UART ##############################################*/
	uartHandle.Instance        = USART2;
	uartHandle.Init.BaudRate   = 9600;
	uartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	uartHandle.Init.StopBits   = UART_STOPBITS_1;
	uartHandle.Init.Parity     = UART_PARITY_NONE;
	uartHandle.Init.Mode       = UART_MODE_TX;
	uartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
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
	gpioInit.Pin   = GPIO_PIN_2;
	gpioInit.Mode  = GPIO_MODE_AF_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpioInit);
}

/**
  * @brief  TIM configuration:
  *             TIM                = TIM1
  *             Prescaler          = 0
  *             Counter mode       = Up
  *             Period             = 0xFFFF
  *             Clock division     = 0
  *             Repetition counter = 0
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
	TIM_Encoder_InitTypeDef timEncoderInit;

	/*## STEP 1: Configure TIM ###############################################*/
	/* Configure TIM base */
	timHandle.Instance               = TIM1;
	timHandle.Init.Prescaler         = 0;
	timHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	timHandle.Init.Period            = 0xFFFF;
	timHandle.Init.ClockDivision     = 0;
	timHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&timHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM encoder interface */
	timEncoderInit.EncoderMode  = TIM_ENCODERMODE_TI12;
	timEncoderInit.IC1Polarity  = TIM_ICPOLARITY_RISING;
	timEncoderInit.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	timEncoderInit.IC1Prescaler = TIM_ICPSC_DIV1;
	timEncoderInit.IC1Filter    = 0;
	timEncoderInit.IC2Polarity  = TIM_ICPOLARITY_FALLING;
	timEncoderInit.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	timEncoderInit.IC2Prescaler = TIM_ICPSC_DIV1;
	timEncoderInit.IC2Filter    = 0;
	if(HAL_TIM_Encoder_Init(&timHandle, &timEncoderInit) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start TIM ###################################################*/
	if(HAL_TIM_Encoder_Start(&timHandle, TIM_CHANNEL_ALL) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  TIM MSP configuration callback.
  * @param  None
  * @retval None
  */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_TIM1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA8 for TIM1 CH1 and PA9 for TIM1 CH2 */
	gpioInit.Pin  = GPIO_PIN_8 | GPIO_PIN_9;
	gpioInit.Mode = GPIO_MODE_AF_INPUT;
	gpioInit.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &gpioInit);
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
