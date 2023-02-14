#include "servo.h"


void TIM3_PWM_Init(uint16_t Arr,uint16_t Psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //TIM3λ��APB1������
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;//CH1-PA6
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP; //���츴�����
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);  
	
	TIM_TimeBaseStructure.TIM_ClockDivision=0;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseStructure.TIM_Period=Arr;
	TIM_TimeBaseStructure.TIM_Prescaler=Psc;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM2; 
     //TIM�����ȵ���ģʽ2 pwm2�����ϼ���ʱ һ��TIM3_CNT<TIM3_CCR1ʱͨ��1Ϊ��Ч��ƽ ����Ϊ��Ч��ƽ
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//��Ч��ƽΪ�ߵ�ƽ�����������CC2P=0
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//�Ƚ����ʹ�ܣ���CC2E=1
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3,TIM_OCPreload_Enable);//��һ�θ����¼�ʱ������
	TIM_ARRPreloadConfig(TIM3,ENABLE);
	TIM_Cmd(TIM3,ENABLE);
	
	TIM_SetCompare1(TIM3,1850);
}









