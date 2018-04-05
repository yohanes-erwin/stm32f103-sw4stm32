/**
  ******************************************************************************
  * @file    ../stm32f103-hal-adc-analog-watchdog/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    5 Feb 2017
  * @brief   This example shows how to use ADC HAL API to convert data with
  *          analog watchdog based on interrupt.
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
ADC_HandleTypeDef adcHandle;

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void ADC_Config(void);
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

	/*## ADC initialization ##################################################*/
	ADC_Config();

	/*## Main loop ###########################################################*/
	while (1);
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
  * @brief  ADC configuration:
  *             ADC                   = ADC1
  *             Data align            = Align right
  *             Scan mode             = Disable
  *             Continuous conversion = Enable
  *             External trigger      = Software start
  *             Channel               = Channel 0
  *             Channel rank          = 1
  *             Channel sampling time = 239.5 cycle
  *             Analog watchdog mode  = Single regular
  *             Interrupt mode        = Enable
  *             High threshold        = 3072
  *             Low threshold         = 1024
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
	ADC_ChannelConfTypeDef adcChannelConf;
	ADC_AnalogWDGConfTypeDef adcAnalogWDGConf;

	/*## STEP 1: Configure ADC ###############################################*/
	adcHandle.Instance                = ADC1;
	adcHandle.Init.DataAlign          = ADC_DATAALIGN_RIGHT;
	adcHandle.Init.ScanConvMode       = ADC_SCAN_DISABLE;
	adcHandle.Init.ContinuousConvMode = ENABLE;
	adcHandle.Init.ExternalTrigConv   = ADC_SOFTWARE_START;
	if (HAL_ADC_Init(&adcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure ADC channel */
	adcChannelConf.Channel      = ADC_CHANNEL_0;
	adcChannelConf.Rank         = ADC_REGULAR_RANK_1;
	adcChannelConf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &adcChannelConf) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure ADC analog watchdog */
	adcAnalogWDGConf.WatchdogMode  = ADC_ANALOGWATCHDOG_SINGLE_REG;
	adcAnalogWDGConf.Channel       = ADC_CHANNEL_0;
	adcAnalogWDGConf.ITMode        = ENABLE;
	adcAnalogWDGConf.HighThreshold = 3072;
	adcAnalogWDGConf.LowThreshold  = 1024;
	if (HAL_ADC_AnalogWDGConfig(&adcHandle, &adcAnalogWDGConf) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start ADC ###################################################*/
	if (HAL_ADC_Start_IT(&adcHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  ADC MSP configuration callback.
 * @param  None
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
	RCC_PeriphCLKInitTypeDef rccPeriphCLKInit;
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	/* Configure ADC clock prescaler */
	__HAL_RCC_ADC1_CLK_ENABLE();
	rccPeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	rccPeriphCLKInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
	HAL_RCCEx_PeriphCLKConfig(&rccPeriphCLKInit);
	/* Configure RCC for GPIO */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA0 for ADC input */
	gpioInit.Pin  = GPIO_PIN_0;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioInit);

	/*## STEP 3: Configure NVIC ##############################################*/
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief  Analog watchdog callback in non blocking mode.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
	/* Toggle yellow LED */
	for (int i = 0; i < 6; i++)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
		HAL_Delay(250);
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
