/**
  ******************************************************************************
  * @file    ../stm32f103-hal-systick-delay-non-blocking/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    25 Jan 2017
  * @brief   This example code shows how to use HAL API to blink an LED with
  *          SysTick timer in non-blocking mode.
  ******************************************************************************
  * Copyright(c) 2017 Erwin Ouyang
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
uint32_t now, tickstart;

/* Private function prototypes -----------------------------------------------*/
void GPIO_Output_Config(void);
void GPIO_Input_Config(void);

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

	/*## GPIO initialization #################################################*/
	GPIO_Output_Config();
	GPIO_Input_Config();

	/* LED will be toggled relative to this point */
	tickstart = HAL_GetTick();

	/*## Main loop ###########################################################*/
	while (1)
	{
		/* Check if 3s has elapsed. If not, the CPU is not blocked and can do
		 * other tasks. */
		now = HAL_GetTick();
		/* If 3s has elapsed. */
		if ((now-tickstart) >= 3000)
		{
			/* Toggle an LED. */
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
			/* Reference the next toggling to this point. */
			tickstart = now;
		}

		/* If button is pressed. */
		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET)
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		}
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
  * @brief  GPIO configuration:
  *             GPIO = GPIOC
  *             Pin  = PC15
  *             Mode = Input
  *             Pull = Pull-up
  * @param  None
  * @retval None
  */
void GPIO_Input_Config(void)
{
	GPIO_InitTypeDef gpioInit;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	gpioInit.Pin  = GPIO_PIN_15;
	gpioInit.Mode = GPIO_MODE_INPUT;
	gpioInit.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &gpioInit);
}

/******************************** END OF FILE *********************************/
/******************************************************************************/
