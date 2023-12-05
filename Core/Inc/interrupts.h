#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H

#include "main.h"


void NMI_Handler(void);
void HardFault_Handler(void);
void SVC_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Channel1_IRQHandler(void);


#endif /* __INTERRUPTS_H */
