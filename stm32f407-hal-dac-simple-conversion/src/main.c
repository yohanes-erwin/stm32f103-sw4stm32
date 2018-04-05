/**
  ******************************************************************************
  * @file    ../stm32f407-hal-dac-simple-conversion/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    6 Feb 2017
  * @brief   This example shows how to use DAC HAL API to convert data.
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
#include "stm32f4_discovery.h"
#include "stm32f4xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef dacHandle;

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void DAC_Config(void);
void DAC_Write(uint16_t dacVal);
void Error_Handler(void);

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/*## On-board LED initialization #########################################*/
	BSP_LED_Init(LED5);    // Initialize red LED

	/*## HAL initialization ##################################################*/
	HAL_Init();

	/*## System clocks initialization ########################################*/
	/* Set the SYSCLK at maximum frequency (168 MHz) */
	RCC_SystemClock_Config();

	/*## DAC initialization ##################################################*/
	DAC_Config();

	/*## Main loop ###########################################################*/
	while (1)
	{
		/* Increase LED brightness */
		for (int i = 145; i <= 200; i++)
		{
			DAC_Write(i);
			HAL_Delay(50);
		}
		/* Decrease LED brightness */
		for (int i = 200; i >= 145; i--)
		{
			DAC_Write(i);
			HAL_Delay(50);
		}
	}
}

/**
  * @brief  System clock configuration:
  *             System clock source = PLL (HSE)
  *             SYSCLK(Hz)          = 168000000
  *             HCLK(Hz)            = 168000000
  *             AHB prescaler       = 1
  *             APB1 prescaler      = 4
  *             APB2 prescaler      = 2
  *             HSE frequency(Hz)   = 8000000
  *             PLL_M               = 8
  *             PLL_N               = 336
  *             PLL_P               = 2
  *             PLL_Q               = 7
  *             VDD(V)              = 3.3
  *             Flash latency(WS)   = 5
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
	rccOscInit.PLL.PLLState   = RCC_PLL_ON;
	rccOscInit.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
	rccOscInit.PLL.PLLM       = 8;
	rccOscInit.PLL.PLLN       = 336;
	rccOscInit.PLL.PLLP       = RCC_PLLP_DIV2;
	rccOscInit.PLL.PLLQ       = 7;
	if(HAL_RCC_OscConfig(&rccOscInit) != HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Configure SYSCLK, HCLK, PCLK1, and PCLK2 ####################*/
	rccClkInit.ClockType      = RCC_CLOCKTYPE_SYSCLK |
			RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	rccClkInit.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
	rccClkInit.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	rccClkInit.APB1CLKDivider = RCC_HCLK_DIV4;
	rccClkInit.APB2CLKDivider = RCC_HCLK_DIV2;
	if(HAL_RCC_ClockConfig(&rccClkInit, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  DAC configuration:
  *             Instance      = DAC
  *             Trigger       = Software
  *             Output buffer = Enable
  * @param  None
  * @retval None
  */
void DAC_Config(void)
{
	DAC_ChannelConfTypeDef dacChannelConf;

	/*## STEP 1: Configure DAC ###############################################*/
	dacHandle.Instance = DAC;
	if(HAL_DAC_Init(&dacHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure DAC channel */
	dacChannelConf.DAC_Trigger      = DAC_TRIGGER_SOFTWARE;
	dacChannelConf.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if(HAL_DAC_ConfigChannel(&dacHandle, &dacChannelConf, DAC_CHANNEL_1)
			!= HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  DAC MSP configuration callback.
  * @param  hdac: DAC handle pointer
  * @retval None
  */
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC ###############################################*/
	__HAL_RCC_DAC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA4 for DAC_OUT1 */
	gpioInit.Pin  = GPIO_PIN_4;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioInit);
}

/**
  * @brief  Write DAC value to DAC data register.
  * @param  val: DAC value
  * @retval None
  */
void DAC_Write(uint16_t dacVal)
{
	/*## STEP 1: Set DAC value ###############################################*/
	if (HAL_DAC_SetValue(&dacHandle, DAC_CHANNEL_1, DAC_ALIGN_8B_R, dacVal)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start DAC ###################################################*/
	if(HAL_DAC_Start(&dacHandle, DAC_CHANNEL_1) != HAL_OK)
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
	BSP_LED_On(LED5);
	while (1);
}

/******************************** END OF FILE *********************************/
/******************************************************************************/
