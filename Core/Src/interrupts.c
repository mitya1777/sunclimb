#include "interrupts.h"



extern uint32_t system_flag;

void DMA1_Channel1_IRQHandler(void)
{



}

voidTIM21_IRQHandler(void)
{
	if(TIM21 -> SR & TIM_SR_UIF)	{
		TIM21 -> SR &= ~TIM_SR_UIF;
		system_flag |= _SYS_TIMER21_OVERFLOW;
	}
}


void NMI_Handler(void)
{
  while (1)	{
  }
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  while (1)	{
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{

}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

}
