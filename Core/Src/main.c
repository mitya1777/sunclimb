#include "main.h"
#include "initialization.h"

static void periferia_initialization(void);
static void mppt_calculation(void);
static void electric_measurement(void);


typedef struct	{
	struct	{
		float v_source;
		float v_output;
	}	voltage	;
	struct	{
		float i_source;
		float i_output;
	}	current;
	struct	{
		float p_source;
		float p_output;
		float p_source_previous;
		float p_output_previous;
	}	power;
	float value_temperature;
}	adc_data_s;

adc_data_s measurement = {RESET};
float duty_cycle = RESET;
uint16_t adc_data[0x06] = {RESET};
uint16_t mppt_period_counter = MPPT_PERIOD_DEFAULT;
uint32_t system_flag = RESET;
volatile uint32_t ms_factor;
float r_shunt_1 = SHUNT_RESISTANCE;
float r_shunt_2 = SHUNT_RESISTANCE;

int main(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);
	NVIC_SetPriority(SysTick_IRQn, 3);

	periferia_initialization();
	pin_action(INDICATION_GPIO_PORT, INDICATION_PIN, SET);
	systick_delay(150);
	pin_action(INDICATION_GPIO_PORT, INDICATION_PIN, RESET);
	while(!(system_flag & _SYS_ADC_DATA_READY));

	while(1)	{
		if(system_flag)	{
			if(system_flag & _SYS_TIMER21_OVERFLOW)	{
				if(!(mppt_period_counter --))	{
					mppt_calculation();
					mppt_period_counter = MPPT_PERIOD_DEFAULT;
				}
				system_flag &= ~_SYS_TIMER21_OVERFLOW;
			}
		}
	}
}


static void periferia_initialization(void)
{
	system_clock_configuration();
	gpio_initialization();
	adc_initialization();
	timers_initialization();
	systick_initialization();
	dma_initialization();
	watchdog_initialization();
}

static void mppt_calculation(void)
{
	for(uint8_t i = 0x00; i < 0xFF; i ++)	{
		LL_ADC_REG_StartConversion(ADC1);
		while(!(system_flag & _SYS_ADC_DATA_READY));
		electric_measurement();
		if(measurement.power.p_source < measurement.power.p_source_previous)	{
			duty_cycle += 0.1;
			TIM2 -> CCR3 = duty_cycle * 655.35;
			TIM2 -> EGR |= TIM_EGR_UG;
		}
		else	{
			measurement.power.p_source_previous = measurement.power.p_source;
			break;
		}
		measurement.power.p_source_previous = measurement.power.p_source;
	}
	systick_delay(0x01);
}

static void electric_measurement(void)
{
	measurement.voltage.v_source = (float) (3.0 * (adc_data[0] / 0x0FFF));
	measurement.voltage.v_output = (float) (3.0 * (adc_data[2] / 0x0FFF));
	measurement.current.i_source = (float) ((3.0 / 0x0FFF / r_shunt_1) * (adc_data[0] - adc_data[1]));
	measurement.current.i_output = (float) ((3.0 / 0x0FFF / r_shunt_2) * (adc_data[2] - adc_data[3]));
	measurement.power.p_source = measurement.voltage.v_source * measurement.current.i_source;
	measurement.power.p_output = measurement.voltage.v_output * measurement.current.i_output;
}

void pin_action(GPIO_TypeDef *gpio_port, uint32_t gpio_pin, FlagStatus new_state)	{
	switch(new_state)	{
	case SET:
		gpio_port -> ODR |= gpio_pin;
		break;
	case RESET:
		gpio_port -> ODR &= ~gpio_pin;
		break;
	default:
		break;
	}
}

void systick_delay(uint32_t quantity)
{
	uint32_t temp = RESET;
	SysTick -> CTRL &= ~SysTick_CTRL_TICKINT_Msk;
	SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;
	while(quantity)	{
		SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk ;
		if(quantity > STEP_DELAY_MS)	{
			SysTick -> LOAD = (uint32_t) (STEP_DELAY_MS * ms_factor);
			quantity -= STEP_DELAY_MS;
	    }
	    else	{
	    	SysTick -> LOAD = (uint32_t) (quantity * ms_factor);
	    	quantity = RESET;
	    }
		SysTick -> VAL = RESET;
		SysTick -> CTRL |= SysTick_CTRL_ENABLE_Msk;
		do	{
			temp = SysTick -> CTRL;
		}	while((temp & 0x01) && !(temp & (1 << 16)));
		SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;
		SysTick -> VAL = RESET;
	}
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
