#include "key.h"

/**
 * 函数名:LED_BEEP_GPIO_Config
 * 描述:led，beep-GPIO配置函数
 * 输入:无
 * 输出:无
 */
void LED_BEEP_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOC,ENABLE);
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP; //GPIO推挽输出模式
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	BEEP_OFF;
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP; //GPIO推挽输出模式
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	LED_OFF;
}

/**
 * 函数名:EXTIX_Init
 * 描述:按键外部中断配置函数
 * 输入:无
 * 输出:无
 */
void EXTIX_Init(void)
{ 
   	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使用外部中断寄存器 需要开启AFIO时钟
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_0|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);


   //GPIOA.9/10/12	  中断线以及中断初始化配置 下降沿触发 
  	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource0);
  	EXTI_InitStructure.EXTI_Line= EXTI_Line0;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_Init(&EXTI_InitStructure);
	
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource4);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line4;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  	EXTI_Init(&EXTI_InitStructure);
		
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource12);
  	EXTI_InitStructure.EXTI_Line=EXTI_Line12;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
}




