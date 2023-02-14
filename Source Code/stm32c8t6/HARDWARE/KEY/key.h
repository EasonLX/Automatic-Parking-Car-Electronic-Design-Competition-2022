#ifndef __key_H
#define	__key_H

#include "stm32f10x.h"
#include "sys.h" 

//外部中断按键定义   KEY-PB0  KEY-PB4  KEY-PA12
#define kEY1 			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define kEY2 			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)
#define kEY3 			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)

//蜂鸣器定义      PA4
#define BEEP_ON         GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define BEEP_OFF        GPIO_SetBits(GPIOA,GPIO_Pin_4)


/* 直接操作寄存器的方法控制IO */
/* 直接操作寄存器的方法控制IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //输出为高电平		
#define digitalLo(p,i)		 {p->BRR=i;}	 //输出低电平
#define digitalToggle(p,i) 	 {p->ODR ^=i;}	 //输出反转状态



//LED控制        PC13 板载led
#define LED_TOGGLE      digitalToggle(GPIOC,GPIO_Pin_13)  //
#define LED_OFF         digitalHi(GPIOC,GPIO_Pin_13)
#define LED_ON      	digitalLo(GPIOC,GPIO_Pin_13)


/* 提供给其他C文件调用的函数 --------------------------------------------------------------------------------------------*/
void LED_BEEP_GPIO_Config(void);
void EXTIX_Init(void);




#endif
