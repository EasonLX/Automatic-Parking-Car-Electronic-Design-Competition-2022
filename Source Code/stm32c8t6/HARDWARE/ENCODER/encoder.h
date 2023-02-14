#ifndef _ENCODER_H
#define	_ENCODER_H

#include "stm32f10x.h"
#include "sys.h" 

void TIM2_IRQHandler(void);
void TIM4_IRQHandler(void);
int Read_Speed(int TIMx);
void Encoder_TIM2_Init(void);
void Encoder_TIM4_Init(void);







#endif
