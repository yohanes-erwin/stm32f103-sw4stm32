/**
  ******************************************************************************
  * @file    ../stm32f103-hal-tim-pwm-input/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    5 Feb 2017
  * @brief   This example code shows how to use TIM HAL API to measure the
  *          frequency and duty cycle of an external signal.
  ******************************************************************************
  * Copyright(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright
  *      notice, this list of conditions and the following disclaimer in the
  *      documentation and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its
  *      contributors may be used to endorse or promote products derived from
  *      this software without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  * POSSIBILITY OF SUCH DAMAGE.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef uartHandle;
TIM_HandleTypeDef timPwmHandle;
TIM_HandleTypeDef timIcHandle;

/* Captured value */
__IO uint32_t ic1Value = 0;
/* Duty Cycle value */
__IO uint32_t dutyCycle = 0;
/* Frequency value */
__IO uint32_t frequency = 0;

/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void UART_Config(void);
void TIM_PWM_Config(void);
void TIM_IC_Config(void);
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
	TIM_PWM_Config();
	TIM_IC_Config();

	/*## Main loop ###########################################################*/
	while (1)
	{
		/* Send frequency and duty cycle to serial monitor application */
		printf("Frequency,DutyCycle=%iHz,%i%%\n", (int)frequency,
				(int)dutyCycle);
		HAL_Delay(2000);
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
  *             TIM                = TIM4
  *             Prescaler          = 18
  *             Counter mode       = Up
  *             Period             = 1024
  *             Clock division     = 0
  *             Repetition counter = 0
  * @param  None
  * @retval None
  */
void TIM_PWM_Config(void)
{
	TIM_OC_InitTypeDef timOcInit;

	/*## STEP 1: Configure TIM ###############################################*/
	/* Configure TIM base */
	timPwmHandle.Instance               = TIM4;
	/* TIM4CLK = CK_INT = 72 MHz
	 * Prescaler = 18
	 * CK_PSC = CK_CNT = clock counter = 72 MHz/18 = 4 MHz (0.25us) */
	timPwmHandle.Init.Prescaler         = 18 - 1;
	timPwmHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	/* ARR = counter overflow = period = 1024 count
	 * PWM signal period = 0.25us * 1024 = 256us (3906.25 Hz) */
	timPwmHandle.Init.Period            = 1024 - 1;
	timPwmHandle.Init.ClockDivision     = 0;
	timPwmHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&timPwmHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM PWM */
	timOcInit.OCMode     = TIM_OCMODE_PWM1;
	timOcInit.Pulse      = 511;
	timOcInit.OCPolarity = TIM_OCPOLARITY_HIGH;
	timOcInit.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_OC_ConfigChannel(&timPwmHandle, &timOcInit, TIM_CHANNEL_2)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start TIM ###################################################*/
	if (HAL_TIM_PWM_Start(&timPwmHandle, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  TIM configuration:
 *             TIM                = TIM3
 *             Prescaler          = 0
 *             Counter mode       = Up
 *             Period             = 0xFFFF
 *             Repetition counter = 0
 * @param  None
 * @retval None
 */
void TIM_IC_Config(void)
{
	TIM_IC_InitTypeDef timIcInit;
	TIM_SlaveConfigTypeDef timSlaveConfig;

	/*## STEP 1: Configure TIM ###############################################*/
	/* Configure TIM base */
	timIcHandle.Instance               = TIM3;
	timIcHandle.Init.Prescaler         = 0;
	timIcHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	timIcHandle.Init.Period            = 0xFFFF;
	timIcHandle.Init.ClockDivision     = 0;
	timIcHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&timIcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM input capture */
	timIcInit.ICPrescaler = TIM_ICPSC_DIV1;
	timIcInit.ICFilter    = 0;
	/* Configure the input capture of channel 1 */
	timIcInit.ICPolarity  = TIM_ICPOLARITY_RISING;
	timIcInit.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&timIcHandle, &timIcInit, TIM_CHANNEL_1)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/* Configure the input capture of channel 2 */
	timIcInit.ICPolarity  = TIM_ICPOLARITY_FALLING;
	timIcInit.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&timIcHandle, &timIcInit, TIM_CHANNEL_2)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM slave reset mode */
	timSlaveConfig.SlaveMode        = TIM_SLAVEMODE_RESET;
	timSlaveConfig.InputTrigger     = TIM_TS_TI1FP1;
	timSlaveConfig.TriggerPolarity  = TIM_TRIGGERPOLARITY_RISING;
	timSlaveConfig.TriggerPrescaler = TIM_CLOCKPRESCALER_DIV1;
	timSlaveConfig.TriggerFilter    = 0;
	if (HAL_TIM_SlaveConfigSynchronization(&timIcHandle, &timSlaveConfig)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start TIM ###################################################*/
	if (HAL_TIM_IC_Start_IT(&timIcHandle, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Start_IT(&timIcHandle, TIM_CHANNEL_2) != HAL_OK)
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
	__HAL_RCC_TIM3_CLK_ENABLE();
	__HAL_RCC_TIM4_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA6 for TIM3 CH1 input */
	gpioInit.Pin  = GPIO_PIN_6 | GPIO_PIN_7;
	gpioInit.Mode = GPIO_MODE_AF_INPUT;
	gpioInit.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &gpioInit);
	/* Configure PB7 for TIM4 CH2 output */
	gpioInit.Pin   = GPIO_PIN_7;
	gpioInit.Mode  = GPIO_MODE_AF_PP;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioInit);

	/*## STEP 3: Configure NVIC ##############################################*/
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief  Input capture callback in non blocking mode.
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		/* Get the input capture value */
		ic1Value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

		if (ic1Value != 0)
		{
			/* Duty cycle computation */
			dutyCycle = ((HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)) * 100)
					/ ic1Value;

			/* Frequency computation */
			/* TIM3CLK = 72 MHz */
			frequency = 72000000 / ic1Value;

			/* PWM signal frequency from TIM4 CH2 = 3906.25Hz */
			if ((frequency == 3906) && (dutyCycle == 24))
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			}
			else
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			}
		}
		else
		{
			dutyCycle = 0;
			frequency = 0;
		}
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
