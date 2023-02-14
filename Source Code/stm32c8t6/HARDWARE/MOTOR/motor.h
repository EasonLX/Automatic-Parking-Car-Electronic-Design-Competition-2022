#ifndef _MOTOR_H
#define	_MOTOR_H

#include "stm32f10x.h"
#include "sys.h"

#define Bin14  PBout(14)
#define Bin15  PBout(15)
#define Ain12  PBout(12)
#define Ain13  PBout(13)


#define PWMA   TIM1->CCR2  //PA9
#define PWMB   TIM1->CCR4  //PA11

void Motor_Init(void);
void Limit(int *motoA,int *motoB);
int  abs(int p);
void Load(int motor1,int motor2,uint16_t Target_Position);


void set_motor_enable(void);
void set_motor_disable(void);
void Set_PWM(int motor1,int motor2);

















#endif
