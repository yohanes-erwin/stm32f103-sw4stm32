/**
  ******************************************************************************
  * @file    ../stm32f103-hal-i2c-two-boards-slave-basic-dma/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    6 Feb 2017
  * @brief   This example shows how to use I2C HAL API to transmit and receive
  *          a data buffer with a communication process based on DMA transfer.
  *          The communication is done using two STM32 boards.
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
#define SLAVE_ADDR  0x88
#define BUFFER_SIZE 44

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef i2cHandle;

/* Buffer used for transmission */
uint8_t txBuffer[] = "I2C two boards communication based on DMA.\r\n";
/* Buffer used for reception */
uint8_t rxBuffer[BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void GPIO_Input_Config(void);
void I2C_Config(void);
uint8_t BufferCmp(uint8_t* pBuff1, uint8_t* pBuff2, uint16_t len);
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
	GPIO_Input_Config();

	/*## I2C initialization ##################################################*/
	I2C_Config();

	/*## I2C communication ###################################################*/
	/* Put I2C in reception process */
	if(HAL_I2C_Slave_Receive_DMA(&i2cHandle, (uint8_t *)rxBuffer, BUFFER_SIZE)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/* Wait for the end of the transfer */
	/* Application may perform other tasks while transfer operation is ongoing
	 * instead of just waiting till the end of the transfer */
	while (HAL_I2C_GetState(&i2cHandle) != HAL_I2C_STATE_READY);

	/* Start the transmission process */
	if(HAL_I2C_Slave_Transmit_DMA(&i2cHandle, (uint8_t*)txBuffer, BUFFER_SIZE)
			!= HAL_OK)
	{
		Error_Handler();
	}

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

/**
  * @brief  I2C slave configuration:
  *             I2C               = I2C2
  *             Clock speed       = 100 kHz
  *             Duty cycle        = 2
  *             Own address       = 0x88
  *             Addressing mode   = 7-bit
  *             Dual address mode = Disable
  *             General call mode = Disable
  *             No stretch mode   = Disable
  * @param  None
  * @retval None
  */
void I2C_Config(void)
{
	/*## STEP 1: Configure I2C ###############################################*/
	i2cHandle.Instance             = I2C2;
	i2cHandle.Init.ClockSpeed      = 100000;
	i2cHandle.Init.DutyCycle       = I2C_DUTYCYCLE_2;
	i2cHandle.Init.OwnAddress1     = SLAVE_ADDR;
	i2cHandle.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	i2cHandle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2cHandle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2cHandle.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
	if(HAL_I2C_Init(&i2cHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  I2C MSP configuration callback.
  * @param  hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef gpioInit;
	static DMA_HandleTypeDef dmaTxHandle;
	static DMA_HandleTypeDef dmaRxHandle;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PB10 and PB11 for I2C2_SCL and I2C2_SDA */
	gpioInit.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
	gpioInit.Mode  = GPIO_MODE_AF_OD;
	gpioInit.Pull  = GPIO_PULLUP;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioInit);

	/*## STEP 3: Configure DMA ###############################################*/
	/* Configure DMA for I2C2_TX */
	dmaTxHandle.Instance                 = DMA1_Channel4;
	dmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	dmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	dmaTxHandle.Init.Mode                = DMA_NORMAL;
	dmaTxHandle.Init.Priority            = DMA_PRIORITY_LOW;
	HAL_DMA_Init(&dmaTxHandle);
	__HAL_LINKDMA(hi2c, hdmatx, dmaTxHandle);
	/* Configure DMA for I2C2_RX */
	dmaRxHandle.Instance                 = DMA1_Channel5;
	dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	dmaRxHandle.Init.Mode                = DMA_NORMAL;
	dmaRxHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaRxHandle);
	__HAL_LINKDMA(hi2c, hdmarx, dmaRxHandle);

	/*## STEP 4: Configure NVIC ##############################################*/
	/* Configure NVIC for DMA1 channel 4 (I2C2_TX) */
	HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
	/* Configure NVIC for DMA1 channel 5 (I2C2_RX) */
	HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
  * @brief  Tx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @retval None
  */
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Turn yellow LED off */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

	/* Compare the sent and received buffers */
	if(BufferCmp(txBuffer, rxBuffer, BUFFER_SIZE))
	{
		/* Turn red LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
	}
	else
	{
		/* Turn green LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	}
}

/**
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle
  * @retval None
  */
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Turn yellow LED on */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
}

/**
  * @brief  I2C error callback.
  * @param  I2cHandle: I2C handle
  * @retval None
  */
 void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *I2cHandle)
{
	 /* Turn red LED on */
	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}

/**
  * @brief  Compare two buffers.
  * @param  pBuff1: buffer to be compared
  *         pBuff2: buffer to be compared
  *         len   : buffer length
  * @retval 0: pBuff1 identical to pBuff2
  *         1: pBuff1 differs from pBuff2
  */
uint8_t BufferCmp(uint8_t* pBuff1, uint8_t* pBuff2, uint16_t len)
{
	while (len--)
	{
		if((*pBuff1) != *pBuff2)
		{
			return 1;
		}
		pBuff1++;
		pBuff2++;
	}
	return 0;
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
