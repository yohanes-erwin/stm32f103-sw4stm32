/**
  ******************************************************************************
  * @file    ../stm32f103-hal-adc-internal-channel/src/main.c
  * @author  Erwin Ouyang
  * @version 1.0
  * @date    5 Feb 2017
  * @brief   This example shows how to use ADC HAL API to convert data from
  *          internal channel (temperature sensor and Vrefint).
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
/* Value of analog voltage supply Vdda (mV) */
#define VDD_APPLI                    ((uint16_t)3300)
/* Max ADC value with a full range of 12 bits */
#define RANGE_12BITS                 ((uint16_t)4095)
/* Internal temperature sensor, parameter V25 (mV).
 * Refer to device datasheet min/typ/max values */
#define INTERNAL_TEMPSENSOR_V25      ((int16_t)1430)
/* Internal temperature sensor, parameter Avg_Slope (uV/Celsius).
 * Refer to device datasheet min/typ/max values */
#define INTERNAL_TEMPSENSOR_AVGSLOPE ((int16_t)4300)

/* Private macro -------------------------------------------------------------*/
/**
  * @brief  Computation of temperature from the internal temperature sensor
  *         measurement by ADC.
  *         Computation formula (refer to device datasheet):
  *         Temperature (Celcius) = {(V25-Vsense)*1000/Avg_Slope} + 25
  *         with V25 (mV)               = temperature sensor @25C and Vdda 3.3V
  *              Vsense (mV)            = temperature sensor voltage
  *              Avg_Slope (uV/Celsius) = temperature sensor slope
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale value if using a different
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value
  *            if using a different analog voltage supply value).
  * @param  ADC_VAL: Temperature sensor digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_TEMPERATURE(ADC_VAL) \
	((((int32_t)(INTERNAL_TEMPSENSOR_V25-(((ADC_VAL)*VDD_APPLI)/RANGE_12BITS)) \
	* 1000) / INTERNAL_TEMPSENSOR_AVGSLOPE) + 25)

/**
  * @brief  Computation of voltage (mV) from ADC measurement digital
  *         value on range 12 bits.
  *         Calculation validity conditioned to settings:
  *          - ADC resolution 12 bits (need to scale value if using a different
  *            resolution).
  *          - Power supply of analog voltage Vdda 3.3V (need to scale value
  *            if using a different analog voltage supply value).
  * @param  ADC_VAL: Digital value measured by ADC
  * @retval None
  */
#define COMPUTATION_12BITS_TO_VOLTAGE(ADC_VAL) \
	(((ADC_VAL) * VDD_APPLI) / RANGE_12BITS)

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef uartHandle;
ADC_HandleTypeDef adcHandle;

uint16_t adcValue[2];
int16_t tempSensorValue;
uint16_t vRefIntValue;

/* Private function prototypes -----------------------------------------------*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void UART_Config(void);
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

	/*## UART initialization #################################################*/
	UART_Config();

	/*## ADC initialization ##################################################*/
	ADC_Config();

	/*## Main loop ###########################################################*/
	while (1)
	{
		/* Start ADC conversion */
		HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)adcValue, 2);
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
  * @brief  ADC configuration:
  *             ADC                   = ADC1
  *             Data align            = Align right
  *             Scan mode             = Enable
  *             Continuous conversion = Disable
  *             Number of conversion  = 2
  *             Discontinuous mode    = Disable
  *             External trigger      = Software start
  *             Channel               = Channel 0, channel 1
  *             Channel rank          = 1, 2
  *             Channel sampling time = 239.5 cycle, 239.5 cycle
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
	ADC_ChannelConfTypeDef adcChannelConf;

	/*## STEP 1: Configure ADC ###############################################*/
	adcHandle.Instance                   = ADC1;
	adcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	adcHandle.Init.ScanConvMode          = ADC_SCAN_ENABLE;
	adcHandle.Init.ContinuousConvMode    = DISABLE;
	adcHandle.Init.NbrOfConversion       = 2;
	adcHandle.Init.DiscontinuousConvMode = DISABLE;
	adcHandle.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	if (HAL_ADC_Init(&adcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure ADC channel */
	adcChannelConf.Channel      = ADC_CHANNEL_TEMPSENSOR;
	adcChannelConf.Rank         = ADC_REGULAR_RANK_1;
	adcChannelConf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &adcChannelConf) != HAL_OK)
	{
		Error_Handler();
	}
	adcChannelConf.Channel      = ADC_CHANNEL_VREFINT;
	adcChannelConf.Rank         = ADC_REGULAR_RANK_2;
	adcChannelConf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &adcChannelConf) != HAL_OK)
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
	static DMA_HandleTypeDef dmaHandle;

	/*## STEP 1: Configure RCC peripheral ####################################*/
	/* Configure ADC clock prescaler */
	__HAL_RCC_ADC1_CLK_ENABLE();
	rccPeriphCLKInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	rccPeriphCLKInit.AdcClockSelection    = RCC_ADCPCLK2_DIV6;
	HAL_RCCEx_PeriphCLKConfig(&rccPeriphCLKInit);
	/* Configure RCC for GPIO */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/* Configure RCC for DMA */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/*## STEP 2: Configure GPIO ##############################################*/
	/* Configure PA0 and PA1 for ADC input */
	gpioInit.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpioInit);

	/*## STEP 3: Configure DMA ###############################################*/
	dmaHandle.Instance                 = DMA1_Channel1;
	dmaHandle.Init.Direction           = DMA_PERIPH_TO_MEMORY;
	dmaHandle.Init.PeriphInc           = DMA_PINC_DISABLE;
	dmaHandle.Init.MemInc              = DMA_MINC_ENABLE;
	dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
	dmaHandle.Init.Mode                = DMA_NORMAL;
	dmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaHandle);
	__HAL_LINKDMA(hadc, DMA_Handle, dmaHandle);

	/*## STEP 4: Configure NVIC ##############################################*/
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief  Conversion complete callback in non blocking mode.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* Computation of ADC conversions raw data to physical values */
	tempSensorValue = COMPUTATION_TEMPERATURE(adcValue[0]);
	vRefIntValue = COMPUTATION_12BITS_TO_VOLTAGE(adcValue[1]);

	/* Send to serial monitor application */
	printf("Temperature=%iC\n", tempSensorValue);
	printf("VREFINT=%imV\n", vRefIntValue);
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
