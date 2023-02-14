#include "servo.h"


void TIM3_PWM_Init(uint16_t Arr,uint16_t Psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //TIM3位于APB1总线上
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;//CH1-PA6
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //推挽复用输出
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);  
	
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseStructure.TIM_Period=Arr;
	TIM_TimeBaseStructure.TIM_Prescaler=Psc;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; 
     //TIM脉冲宽度调制模式2 pwm2在向上计数时 一旦TIM3_CNT<TIM3_CCR1时通道1为无效电平 否则为有效电平
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//有效电平为高电平，即输出极性CC2P=0
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//比较输出使能，即CC2E=1
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);//下一次更新事件时被更新
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	
	TIM_SetCompare1(TIM3,1850);
}









