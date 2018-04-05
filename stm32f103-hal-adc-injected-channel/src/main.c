/**
 ******************************************************************************
 * @file    ../stm32f103-hal-adc-injected-channel/src/main.c
 * @author  Erwin Ouyang
 * @version 1.0
 * @date    5 Feb 2017
 * @brief   This example shows how to use ADC HAL API to convert data from
 *          regular channel and injected channel.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef timHandle;
ADC_HandleTypeDef adcHandle;

uint16_t regularAdcValue;
uint16_t injectedAdcValue;

/* Private function prototypes -----------------------------------------------*/
void RCC_SystemClock_Config(void);
void GPIO_Output_Config(void);
void TIM_Config(void);
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

	/*## TIM initialization ##################################################*/
	TIM_Config();

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
  * @brief  TIM configuration:
  *             TIM                = TIM4
  *             Prescaler          = 7200
  *             Counter mode       = Up
  *             Period             = 20000
  *             Clock division     = 0
  *             Repetition counter = 0
  * @param  None
  * @retval None
  */
void TIM_Config(void)
{
	TIM_MasterConfigTypeDef timMasterConfig;

	/*## STEP 1: Configure TIM ###############################################*/
	/* Configure TIM base */
	timHandle.Instance               = TIM4;
	/* TIM1CLK = CK_INT = 72 MHz
	 * Prescaler = 7200
	 * CK_PSC = CK_CNT = clock counter = 72 MHz/7200 = 10 kHz (0.1ms) */
	timHandle.Init.Prescaler         = 7200 - 1;
	timHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	/* ARR = counter overflow = period = 20000 count
	 * TIM interrupt period = 0.1ms * 20000 = 2s */
	timHandle.Init.Period            = 20000 - 1;
	timHandle.Init.ClockDivision     = 0;
	timHandle.Init.RepetitionCounter = 0;
	if (HAL_TIM_Base_Init(&timHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure TIM TRGO */
	timMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	timMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&timHandle, &timMasterConfig)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start TIM ###################################################*/
	if (HAL_TIM_Base_Start(&timHandle) != HAL_OK)
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
	/*## STEP 1: Configure RCC peripheral ####################################*/
	__HAL_RCC_TIM4_CLK_ENABLE();
}

/**
  * @brief  ADC configuration:
  *             ADC                   = ADC1
  *             Data align            = Align right
  *             Scan mode             = Disable
  *             Continuous conversion = Enable
  *             External trigger      = Software start
  *             Channel               = Channel 0, channel 1
  *             Channel rank          = 1, 1 (injected)
  *             Channel sampling time = 239.5 cycle, 1.5 cycle
  * @param  None
  * @retval None
  */
void ADC_Config(void)
{
	ADC_ChannelConfTypeDef adcChannelConf;
	ADC_InjectionConfTypeDef adcInjectionConf;

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
	/* Configure ADC regular channel */
	adcChannelConf.Channel      = ADC_CHANNEL_0;
	adcChannelConf.Rank         = ADC_REGULAR_RANK_1;
	adcChannelConf.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&adcHandle, &adcChannelConf) != HAL_OK)
	{
		Error_Handler();
	}
	/* Configure ADC injected channel */
	adcInjectionConf.InjectedChannel       = ADC_CHANNEL_1;
	adcInjectionConf.InjectedRank          = ADC_INJECTED_RANK_1;
	adcInjectionConf.InjectedSamplingTime  = ADC_SAMPLETIME_1CYCLE_5;
	adcInjectionConf.InjectedOffset        = 0;
	adcInjectionConf.AutoInjectedConv      = DISABLE;
	adcInjectionConf.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T4_TRGO;
	if (HAL_ADCEx_InjectedConfigChannel(&adcHandle, &adcInjectionConf)
			!= HAL_OK)
	{
		Error_Handler();
	}

	/*## STEP 2: Start ADC ###################################################*/
	if (HAL_ADC_Start_DMA(&adcHandle, (uint32_t*)&regularAdcValue, 1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_ADCEx_InjectedStart_IT(&adcHandle) != HAL_OK)
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
	dmaHandle.Init.MemInc              = DMA_MINC_DISABLE;
	dmaHandle.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
	dmaHandle.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
	dmaHandle.Init.Mode                = DMA_CIRCULAR;
	dmaHandle.Init.Priority            = DMA_PRIORITY_HIGH;
	HAL_DMA_Init(&dmaHandle);
	__HAL_LINKDMA(hadc, DMA_Handle, dmaHandle);

	/*## STEP 4: Configure NVIC ##############################################*/
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief  Conversion complete callback in non blocking mode.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (regularAdcValue > 2047)
	{
		/* Turn yellow LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	}
	else
	{
		/* Turn yellow LED off */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}
}

/**
  * @brief  Injected conversion complete callback in non blocking mode.
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	injectedAdcValue = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
	if (injectedAdcValue > 2047)
	{
		/* Turn green LED on */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
	}
	else
	{
		/* Turn green LED off */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
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
