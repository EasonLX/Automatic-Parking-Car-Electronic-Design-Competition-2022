#include "pwm.h"



void PWM_Init_TIM1(u16 Psc,u16 Per)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO,ENABLE);//开启时钟
	
	//设置该引脚为复用输出功能,输出TIM1 CH1 CH4的PWM脉冲波形
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//初始化GPIO--PA9、PA11为复用推挽输出  
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9 |GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	//时基单元初始化
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);//初始化定时器。
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//TIM向上计数模式 
	TIM_TimeBaseInitStruct.TIM_Period=Per;//设置在下一个更新事件装入活动的自动重装载寄存器周期的值	  arr
	TIM_TimeBaseInitStruct.TIM_Prescaler=Psc;//设置用来作为TIMx时钟频率除数的预分频值  不分频       psc
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);//根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//设置输出比较
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;//初始化输出比较
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;//输出极性:TIM输出比较极性高
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStruct.TIM_Pulse=0;//设置待装入捕获比较寄存器的脉冲值
	TIM_OC2Init(TIM1,&TIM_OCInitStruct);
	TIM_OC4Init(TIM1,&TIM_OCInitStruct);
	
	//MOE 主输出使能
	TIM_CtrlPWMOutputs(TIM1,ENABLE);//高级定时器专属--MOE主输出使能
	
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC1预装载寄存器使能
	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);//ENABLE//OC4预装载寄存器使能
	TIM_ARRPreloadConfig(TIM1,ENABLE);//TIM1在ARR上预装载寄存器使能
	
	TIM_Cmd(TIM1,ENABLE);//开定时器。
}

