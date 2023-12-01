#ifndef __MAIN_H
#define __MAIN_H

//#include "stm32l0xx_ll_adc.h"
//#include "stm32l0xx_ll_dma.h"
//#include "stm32l0xx_ll_crs.h"
//#include "stm32l0xx_ll_rcc.h"
//#include "stm32l0xx_ll_bus.h"
//#include "stm32l0xx_ll_system.h"
//#include "stm32l0xx_ll_exti.h"
//#include "stm32l0xx_ll_cortex.h"
//#include "stm32l0xx_ll_utils.h"
//#include "stm32l0xx_ll_pwr.h"
//#include "stm32l0xx_ll_tim.h"
//#include "stm32l0xx_ll_wwdg.h"
//#include "stm32l0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

void Error_Handler(void);

#define SOURCE_VOLTAGE_GPIO_PORT					GPIOA
#define SOURCE_VOLTAGE_PIN							LL_GPIO_PIN_0
#define ADC_HIGH_SHOULDER_PORT						GPIOA
#define ADC_HIGH_SHOULDER_PIN						LL_GPIO_PIN_1
#define ADC_LOW_SHOULDER_GPIO_PORT					GPIOA
#define ADC_LOW_SHOULDER_PIN						LL_GPIO_PIN_4
#define ADC_OUT_POWER_GPIO_PORT						GPIOA
#define ADC_OUT_POWER_PIN							LL_GPIO_PIN_7
#define INDICATION_GPIO_PORT						GPIOA
#define INDICATION_PIN								LL_GPIO_PIN_9
#define PWM_MCU_GPIO_PORT							GPIOA
#define PWM_MCU_PIN									LL_GPIO_PIN_10

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
