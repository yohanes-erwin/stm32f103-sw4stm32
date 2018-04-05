/**
  ******************************************************************************
  * @file    ../stm32f103-hal-i2c-eeprom-ic-polling/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    6 Feb 2017
  * @brief   This example shows how to use I2C HAL API to transmit and receive
  *          a data byte with a communication process based on polling.
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
/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef i2cHandle;

uint8_t eepromData;

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void GPIO_Input_Config(void);
void I2C_Config(void);
void EEPROM_WriteByte(uint8_t devAddr, uint16_t ee10bAddr, uint8_t eeData);
void EEPROM_ReadByte(uint8_t devAddr, uint16_t ee10bAddr, uint8_t *eeData);
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
	/* Wait for user button press before starting the communication */
	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_SET);

	/* Write data 0x68 to EEPROM address 0x0001 */
	EEPROM_WriteByte(EEPROM_DEV_ADDR, 0x0001, 0x68);

	/* Wait until the EEPROM is ready for a new operation */
	HAL_Delay(5);

	/* Read a data byte from EEPROM address 0x0001 */
	EEPROM_ReadByte(EEPROM_DEV_ADDR, 0x0001, &eepromData);

	/* Compare EEPROM data */
	if (eepromData == 0x68)
	{
		/* Turn green LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else
	{
		/* Turn yellow LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
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

	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_I2C2_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PB10 and PB11 for I2C2_SCL and I2C2_SDA */
	gpioInit.Pin   = GPIO_PIN_10 | GPIO_PIN_11;
	gpioInit.Mode  = GPIO_MODE_AF_OD;
	gpioInit.Pull  = GPIO_PULLUP;
	gpioInit.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpioInit);
}

/**
  * @brief  Write a data byte to EEPROM.
  * @param  devAddr: EEPROM device address
  *         ee10bAddr: EEPROM data address
  *         eeData: EEPROM data
  * @retval None
  */
void EEPROM_WriteByte(uint8_t devAddr, uint16_t ee10bAddr, uint8_t eeData)
{
	uint8_t i2cData[3];

	/* Device address (5-bit), P1 P0 address (2-bit), and write bit (1-bit) */
	i2cData[0] = devAddr | (((uint8_t)ee10bAddr >> 7) & 0xFE);
	/* The rest of EEPROM address */
	i2cData[1] = (uint8_t)ee10bAddr;
	/* A data byte to be stored in EEPROM */
	i2cData[2] = eeData;

	/* Transmit device address + write followed by word address and a data byte
	 * to be stored in EEPROM */
	if (HAL_I2C_Master_Transmit(&i2cHandle, i2cData[0], &i2cData[1], 2, 1000)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&i2cHandle) != HAL_I2C_STATE_READY);
}

/**
  * @brief  Read a data byte from EEPROM.
  * @param  devAddr: EEPROM device address
  *         ee10bAddr: EEPROM data address
  *         eeData: EEPROM data pointer
  * @retval None
  */
void EEPROM_ReadByte(uint8_t devAddr, uint16_t ee10bAddr, uint8_t *eeData)
{
	uint8_t i2cData[2];

	/* Device address (5-bit), P1 P0 address (2-bit), and write bit (1-bit) */
	i2cData[0] = devAddr | (((uint8_t)ee10bAddr >> 7) & 0xFE);
	/* The rest of EEPROM address */
	i2cData[1] = (uint8_t)ee10bAddr;

	/* Transmit device address + write followed by word address */
	if (HAL_I2C_Master_Transmit(&i2cHandle, i2cData[0], &i2cData[1], 1, 1000)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&i2cHandle) != HAL_I2C_STATE_READY);

	/* Device address (5-bit), P1 P0 address (2-bit), and read bit (1-bit) */
	i2cData[0] = devAddr | (((uint8_t)ee10bAddr >> 7) | 0x01);

	/* Transmit device address + read and receive a data byte */
	if (HAL_I2C_Master_Receive(&i2cHandle, i2cData[0], eeData, 1, 1000)
			!= HAL_OK)
	{
		Error_Handler();
	}
	/* Wait for the end of the transfer */
	while (HAL_I2C_GetState(&i2cHandle) != HAL_I2C_STATE_READY);
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
