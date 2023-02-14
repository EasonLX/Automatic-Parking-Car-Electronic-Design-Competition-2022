#ifndef _EXTI_H
#define	_EXTI_H

#include "stm32f10x.h"
#include "sys.h" 

#define INT PBin(5)   //PB5连接到MPU6050的中断引脚

void MPU6050_EXTI_Init(void);











#endif 
