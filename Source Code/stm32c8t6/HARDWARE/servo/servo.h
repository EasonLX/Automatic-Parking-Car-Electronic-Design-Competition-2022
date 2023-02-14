#ifndef __SERVO_H
#define	__SERVO_H

#include "stm32f10x.h"

#define TIM3_ARR							  (2000-1)//周期
#define TIM3_PSC							  (720-1)//预分频值

#define TIM3_ARR							  (2000-1)//周期
#define TIM3_PSC							  (720-1)//预分频值

void TIM3_PWM_Init(uint16_t Arr,uint16_t Psc);









#endif

