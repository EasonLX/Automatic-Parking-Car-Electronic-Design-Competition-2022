#include "pwm.h"



void PWM_Init_TIM1(u16 Psc,u16 Per)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO,ENABLE);//����ʱ��
	
	//���ø�����Ϊ�����������,���TIM1 CH1 CH4��PWM���岨��
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//��ʼ��GPIO--PA9��PA11Ϊ�����������  
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9 |GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	//ʱ����Ԫ��ʼ��
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);//��ʼ����ʱ����
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//TIM���ϼ���ģʽ 
	TIM_TimeBaseInitStruct.TIM_Period=Per;//��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	  arr
	TIM_TimeBaseInitStruct.TIM_Prescaler=Psc;//����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ       psc
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);//����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
	//��������Ƚ�
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;//��ʼ������Ƚ�
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;//�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStruct.TIM_Pulse=0;//���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OC2Init(TIM1,&TIM_OCInitStruct);
	TIM_OC4Init(TIM1,&TIM_OCInitStruct);
	
	//MOE �����ʹ��
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//�߼���ʱ��ר��--MOE�����ʹ��
	
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC1Ԥװ�ؼĴ���ʹ��
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC4Ԥװ�ؼĴ���ʹ��
	TIM_ARRPreloadConfig(TIM1,ENABLE);//TIM1��ARR��Ԥװ�ؼĴ���ʹ��
	
	TIM_Cmd(TIM1,ENABLE);//����ʱ����
}

