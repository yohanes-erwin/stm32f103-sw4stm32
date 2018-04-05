/**
  ******************************************************************************
  * @file    ../stm32f103-hal-pwr-pvd/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    7 Feb 2017
  * @brief   This example shows how to use PWR HAL API to manage the PVD
  *          (Programmable Voltage Detector).
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
__IO uint32_t low_voltage = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void PWR_Config(void);
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

	/*## PWR initialization ##################################################*/
	PWR_Config();

	/*## Main loop ###########################################################*/
	while (1)
	{
		if (low_voltage)
		{
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
			HAL_Delay(500);
		}
	}
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
  * @brief  PWR configuration:
  *             PVD level = 2.7 V (level 5)
  *             PVD mode  = Interrupt rising falling
  * @param  None
  * @retval None
  */
void PWR_Config(void)
{
	PWR_PVDTypeDef pwrPvdInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_PWR_CLK_ENABLE();

	/*## STEP 2: Configure PWR ###############################################*/
	pwrPvdInit.PVDLevel = PWR_PVDLEVEL_5;
	pwrPvdInit.Mode     = PWR_PVD_MODE_IT_RISING_FALLING;
	HAL_PWR_ConfigPVD(&pwrPvdInit);
	HAL_PWR_EnablePVD();

	/*## STEP 3: Configure NVIC ##############################################*/
	HAL_NVIC_SetPriority(PVD_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(PVD_IRQn);
}

/**
  * @brief  PWR PVD interrupt callback.
  * @param  none
  * @retval none
  */
void HAL_PWR_PVDCallback(void)
{
	/* Update "toggle" global variable so that LED blinks when the voltage is
	 * below the target threshold. */
	low_voltage = (low_voltage+1) % 2;
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
