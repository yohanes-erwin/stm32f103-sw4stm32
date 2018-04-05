/**
  ******************************************************************************
  * @file    ../stm32f103-hal-i2c-eeprom-ic-dma/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    6 Feb 2016
  * @brief   This example shows how to use I2C HAL API to transmit and receive
  *          data bytes with a communication process based on DMA transfer.
  *          The communication is done with the AT24C08 I2C EEPROM IC.
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
/* AT24C08 device address (8-bit left aligned)
 * 1010 + A2(hard-wired to GND) */
#define EEPROM_DEV_ADDR 0xA0

/* Private macro -------------------------------------------------------------*/
#define COUNT_OF(__BUFFER__) (sizeof(__BUFFER__)/sizeof(*(__BUFFER__)))

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef uartHandle;
I2C_HandleTypeDef i2cHandle;

/* Text to be stored in EEPROM (max 15 characters) */
uint8_t text[] = "AT24C08 EEPROM";
uint8_t eepromData[COUNT_OF(text)];

/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void GPIO_Input_Config(void);
void UART_Config(void);
void I2C_Config(void);
void EEPROM_WritePage(uint8_t devAddr, uint16_t ee10bAddr, uint8_t *eeData,
		uint8_t size);
void EEPROM_ReadPage(uint8_t devAddr, uint16_t ee10bAddr, uint8_t *eeData,
		uint8_t size);
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

	/*## UART initialization #################################################*/
	UART_Config();

	/*## I2C initialization ##################################################*/
	I2C_Config();

	/*## I2C communication ###################################################*/
	/* Wait for user button press before starting the communication */
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET);

	/* Write data bytes to EEPROM page 0 */
	EEPROM_WritePage(EEPROM_DEV_ADDR, 0x0000, text, COUNT_OF(text));

	/* Wait until the EEPROM is ready for a new operation */
	HAL_Delay(5);

	/* Read data bytes from EEPROM page 0 */
	EEPROM_ReadPage(EEPROM_DEV_ADDR, 0x0000, eepromData, COUNT_OF(text));

	/*## Main loop ###########################################################*/
	while (1);
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
  * @brief  I2C configuration:
  *             I2C               = I2C2
  *             Clock speed       = 100 kHz
  *             Duty cycle        = 2
  *             Own address       = 0x00
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
	i2cHandle.Init.OwnAddress1     = 0x00;
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
  * @brief  Rx Transfer completed callback.
  * @param  I2cHandle: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	/* Compare EEPROM data */
	if(BufferCmp(eepromData, text, COUNT_OF(text)))
	{
		/* Turn yellow LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		printf("%s\n", text);
		printf("%s\n", eepromData);
	}
	else
	{
		/* Turn green LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
		printf("%s\n", text);
		printf("%s\n", eepromData);
	}
}

/**
  * @brief  Write a data byte to EEPROM.
  * @param  devAddr  : EEPROM device address
  *         ee10bAddr: EEPROM data address
  *         eeData   : EEPROM data pointer
  *         size     : EEPROM data size
  * @retval None
  */
void EEPROM_WritePage(uint8_t devAddr, uint16_t ee10bAddr, uint8_t *eeData,
		uint8_t size)
{
	uint8_t i2cData[2];

	/* Device address (5-bit) and P1 P0 address (2-bit) (8-bit left aligned) */
	i2cData[0] = devAddr | (((uint8_t)ee10bAddr >> 7) & 0xFE);
	/* The rest of EEPROM address */
	i2cData[1] = (uint8_t)ee10bAddr;

	if (HAL_I2C_Mem_Write_DMA(&i2cHandle, i2cData[0], i2cData[1],
			I2C_MEMADD_SIZE_8BIT, eeData, size) != HAL_OK)
	{
		Error_Handler();
	}
	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&i2cHandle) != HAL_I2C_STATE_READY);
}

/**
  * @brief  Read a data byte from EEPROM.
  * @param  devAddr  : EEPROM device address
  *         ee10bAddr: EEPROM data address
  *         eeData   : EEPROM data pointer
  *         size     : EEPROM data size
  * @retval None
  */
void EEPROM_ReadPage(uint8_t devAddr, uint16_t ee10bAddr, uint8_t *eeData,
		uint8_t size)
{
	uint8_t i2cData[2];

	/* Device address (5-bit) and P1 P0 address (2-bit) (8-bit left aligned) */
	i2cData[0] = devAddr | (((uint8_t)ee10bAddr >> 7) & 0xFE);
	/* The rest of EEPROM address */
	i2cData[1] = (uint8_t)ee10bAddr;

	if (HAL_I2C_Mem_Read_DMA(&i2cHandle, i2cData[0], i2cData[1],
			I2C_MEMADD_SIZE_8BIT, eeData, size) != HAL_OK)
	{
		Error_Handler();
	}
	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&i2cHandle) != HAL_I2C_STATE_READY);
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
