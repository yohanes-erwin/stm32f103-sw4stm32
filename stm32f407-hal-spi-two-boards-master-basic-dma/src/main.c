/**
  ******************************************************************************
  * @file    ../stm32f407-hal-spi-two-boards-master-basic-dma/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    6 Feb 2017
  * @brief   This example shows how to use SPI HAL API to transmit and receive
  *          a data buffer with a communication process based on DMA transfer.
  *          The communication is done using two STM32 Boards.
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
#define BUFFER_SIZE 44

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef spiHandle;

/* Buffer used for transmission */
uint8_t txBuffer[] = "SPI two boards communication based on DMA.\r\n";
/* Buffer used for reception */
uint8_t rxBuffer[BUFFER_SIZE];

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void SPI_Config(void);
void SPI_EnableSlave(void);
void SPI_DisableSlave(void);
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
	/*## On-board LED initialization #########################################*/
	BSP_LED_Init(LED5);    // Initialize red LED
	BSP_LED_Init(LED4);    // Initialize green LED
	BSP_LED_Init(LED3);    // Initialize orange LED

	/*## On-board button initialization ######################################*/
	BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);

	/*## HAL initialization ##################################################*/
	HAL_Init();

	/*## System clocks initialization ########################################*/
	/* Set the SYSCLK at maximum frequency (168 MHz) */
	RCC_SystemClock_Config();

	/*## SPI initialization ##################################################*/
	SPI_Config();

	/*## SPI communication ###################################################*/
	/* Wait for user button press before starting the communication */
	while (BSP_PB_GetState(BUTTON_KEY) == 0);

	/* Enable slave device */
	SPI_EnableSlave();

	/* Start the SPI communication process */
	if (HAL_SPI_TransmitReceive_DMA(&spiHandle, txBuffer, rxBuffer, BUFFER_SIZE)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/* Wait for the end of the transfer */
	/* Application may perform other tasks while transfer operation is ongoing
	 * instead of just waiting till the end of the transfer */
	while (HAL_SPI_GetState(&spiHandle) != HAL_SPI_STATE_READY);

	/* Disable slave device */
	SPI_DisableSlave();

	/*## Main loop ###########################################################*/
	while (1);
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
  * @brief  SPI master configuration:
  *             SPI                 = SPI1
  *             Mode                = Master
  *             Direction           = 2 lines
  *             Data size           = 8-bit
  *             Clock polarity      = High
  *             Clock phase         = First edge
  *             Slave select        = Software select
  *             Baud rate prescaler = 256
  *             First bit           = MSB
  *             TI mode             = Disable
  *             CRC calculation     = Disable
  * @param  None
  * @retval None
  */
void SPI_Config(void)
{
	/*## STEP 1: Configure SPI ###############################################*/
	spiHandle.Instance               = SPI1;
	spiHandle.Init.Mode              = SPI_MODE_MASTER;
	spiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
	spiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
	spiHandle.Init.CLKPolarity       = SPI_POLARITY_HIGH;
	spiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
	spiHandle.Init.NSS               = SPI_NSS_HARD_OUTPUT;
	spiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	spiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
	spiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
	spiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
	if(HAL_SPI_Init(&spiHandle) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief  SPI MSP configuration callback.
  * @param  hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef gpioInit;
	static DMA_HandleTypeDef dmaTxHandle;
	static DMA_HandleTypeDef dmaRxHandle;

	/*## STEP 1: Configure RCC ###############################################*/
	__HAL_RCC_SPI1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_DMA2_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA3 for SPI1 slave 1 select */
	gpioInit.Pin       = GPIO_PIN_3;
	gpioInit.Mode      = GPIO_MODE_OUTPUT_PP;
	gpioInit.Pull      = GPIO_NOPULL;
	gpioInit.Speed     = GPIO_SPEED_FREQ_HIGH;;
	HAL_GPIO_Init(GPIOA, &gpioInit);
	SPI_DisableSlave();
	/* Configure PA5, PA6, and PA7 for SPI1_SCK, SPI1_MISO, and SPI1_MOSI */
	gpioInit.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
	gpioInit.Mode      = GPIO_MODE_AF_PP;
	gpioInit.Pull      = GPIO_PULLUP;
	gpioInit.Speed     = GPIO_SPEED_FREQ_HIGH;
	gpioInit.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(GPIOA, &gpioInit);

	/*## STEP 3: Configure DMA ###############################################*/
	/* Configure DMA for SPI1_TX */
	dmaTxHandle.Instance                 = DMA2_Stream3;
	dmaTxHandle.Init.Channel             = DMA_CHANNEL_3;
	dmaTxHandle.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	dmaTxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaTxHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaTxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dmaTxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	dmaTxHandle.Init.Mode                = DMA_NORMAL;
	dmaTxHandle.Init.Priority            = DMA_PRIORITY_LOW;
	dmaTxHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	dmaTxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	dmaTxHandle.Init.MemBurst            = DMA_MBURST_INC4;
	dmaTxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;
	__HAL_LINKDMA(hspi, hdmatx, dmaTxHandle);
	HAL_DMA_Init(&dmaTxHandle);
	/* Configure DMA for SPI1_RX */
	dmaRxHandle.Instance                 = DMA2_Stream2;
	dmaRxHandle.Init.Channel             = DMA_CHANNEL_3;
	dmaRxHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaRxHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaRxHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaRxHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	dmaRxHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	dmaRxHandle.Init.Mode                = DMA_NORMAL;
	dmaRxHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	dmaRxHandle.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	dmaRxHandle.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	dmaRxHandle.Init.MemBurst            = DMA_MBURST_INC4;
	dmaRxHandle.Init.PeriphBurst         = DMA_PBURST_INC4;
	__HAL_LINKDMA(hspi, hdmarx, dmaRxHandle);
	HAL_DMA_Init(&dmaRxHandle);

	/*## STEP 4: Configure NVIC ##############################################*/
	/* Configure NVIC for DMA2 stream 3 (SPI1_TX) */
	HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
	/* Configure NVIC for DMA2 stream 2 (SPI1_RX) */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
  * @brief  TxRx transfer completed callback.
  * @param  hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	/* Compare the sent and received buffer */
	if(BufferCmp(txBuffer, rxBuffer, BUFFER_SIZE))
	{
		/* Turn red LED on */
		BSP_LED_On(LED5);
	}
	else
	{
		/* Turn green LED on */
		BSP_LED_On(LED4);
	}
}

/**
  * @brief  SPI error callback.
  * @param  hspi: SPI handle pointer
  * @retval None
  */
 void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
	/* Turn red LED on */
	BSP_LED_On(LED5);
}

/**
  * @brief  Enable SPI slave device.
  * @param  None
  * @retval None
  */
void SPI_EnableSlave(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

/**
  * @brief  Disable SPI slave device.
  * @param  None
  * @retval None
  */
void SPI_DisableSlave(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
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
	BSP_LED_On(LED5);
	while (1);
}

/******************************** END OF FILE *********************************/
/******************************************************************************/
