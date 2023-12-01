#include "main.h"
#include "initialization.h"

static void periferia_initialization(void);

uint16_t adc_data[0x06] = {RESET};


int main(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	NVIC_SetPriority(SysTick_IRQn, 3);

	periferia_initialization();
	while (1)	{
	}
}


static void periferia_initialization(void)
{
	system_clock_configuration();
	gpio_initialization();
	adc_initialization();
	timers_initialization();
	dma_initialization();
	watchdog_initialization();
}




void Error_Handler(void)
{
  __disable_irq();
  while (1)	{
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
}
#endif /* USE_FULL_ASSERT */
