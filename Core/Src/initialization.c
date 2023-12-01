#include "initialization.h"

void system_clock_configuration(void);
void gpio_initialization(void);
void dma_initialization(void);
void adc_initialization(void);
void timers_initialization(void);
void watchdog_initialization(void);


extern uint16_t adc_data[0x06];

void system_clock_configuration(void)
{
	LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
	while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_1);
	LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
	LL_RCC_HSI_Enable();
	while(LL_RCC_HSI_IsReady() != 0x01);
	LL_RCC_HSI_SetCalibTrimming(16);
	LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_2);
	LL_RCC_PLL_Enable();
	while(LL_RCC_PLL_IsReady() != 0x01);
	LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
	LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
	LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
	LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
	while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);
	LL_Init1msTick(32000000);
	LL_SetSystemCoreClock(32000000);
}

void gpio_initialization(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {RESET};
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_GPIO_ResetOutputPin(INDICATION_GPIO_PORT, INDICATION_PIN);
	GPIO_InitStruct.Pin = INDICATION_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(INDICATION_GPIO_PORT, &GPIO_InitStruct);
}

void adc_initialization(void)
{
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {RESET};
	LL_ADC_InitTypeDef ADC_InitStruct = {RESET};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {RESET};
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**ADC GPIO Configuration
	PA0-CK_IN   ------> ADC_IN0
	PA1   ------> ADC_IN1
	PA4   ------> ADC_IN4
	PA7   ------> ADC_IN7
	*/
	GPIO_InitStruct.Pin = SOURCE_VOLTAGE_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(SOURCE_VOLTAGE_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ADC_HIGH_SHOULDER_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ADC_HIGH_SHOULDER_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ADC_LOW_SHOULDER_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ADC_LOW_SHOULDER_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = ADC_OUT_POWER_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(ADC_OUT_POWER_GPIO_PORT, &GPIO_InitStruct);

	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_0);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_1);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_4);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_7);
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);
	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM21_CH2;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_1RANK;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_39CYCLES_5);
	LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
	LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
	LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
	LL_ADC_DisableIT_EOC(ADC1);
	LL_ADC_DisableIT_EOS(ADC1);
	ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV1;
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);
	LL_ADC_EnableInternalRegulator(ADC1);
	uint32_t wait_loop_index;
	wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
	while(wait_loop_index != 0)	{
		wait_loop_index--;
	}
}

void timers_initialization(void)
{
	/*	timer 2: 50 kHz frequency initialization	*/
	LL_TIM_InitTypeDef TIM_InitStruct = {0};
	LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);
	TIM_InitStruct.Prescaler = 640;
	TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
	TIM_InitStruct.Autoreload = 65535;
	TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
	LL_TIM_Init(TIM2, &TIM_InitStruct);
	LL_TIM_DisableARRPreload(TIM2);
	LL_TIM_SetClockSource(TIM2, LL_TIM_CLOCKSOURCE_INTERNAL);
	LL_TIM_OC_EnablePreload(TIM2, LL_TIM_CHANNEL_CH3);
	TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
	TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
	TIM_OC_InitStruct.CompareValue = 32767;
	TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
	LL_TIM_OC_Init(TIM2, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
	LL_TIM_OC_DisableFast(TIM2, LL_TIM_CHANNEL_CH3);
	LL_TIM_SetTriggerOutput(TIM2, LL_TIM_TRGO_RESET);
	LL_TIM_DisableMasterSlaveMode(TIM2);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	/**TIM2 GPIO Configuration
	PA10   ------> TIM2_CH3
	*/
	GPIO_InitStruct.Pin = PWM_MCU_PIN;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
	LL_GPIO_Init(PWM_MCU_GPIO_PORT, &GPIO_InitStruct);

	/*	timer 21: 1 Hz frequency  initialization	*/
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM21);
	TIM_InitStruct.Prescaler = 64e3;
    TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload = 500;
    TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
    LL_TIM_Init(TIM21, &TIM_InitStruct);
    LL_TIM_DisableARRPreload(TIM21);
    LL_TIM_SetClockSource(TIM21, LL_TIM_CLOCKSOURCE_INTERNAL);
    LL_TIM_SetTriggerOutput(TIM21, LL_TIM_TRGO_OC2REF);
    LL_TIM_DisableMasterSlaveMode(TIM21);
}

void dma_initialization(void)
{
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t) &adc_data);
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, ADC1 -> DR);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/*
 *	watchdog timer: 1.15 Hz frequency  initialization
 * */
void watchdog_initialization(void)
{
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_WWDG);
	LL_WWDG_SetCounter(WWDG, 4020);
	LL_WWDG_Enable(WWDG);
	LL_WWDG_SetPrescaler(WWDG, LL_WWDG_PRESCALER_8);
//	LL_WWDG_SetWindow(WWDG, 64);
}
