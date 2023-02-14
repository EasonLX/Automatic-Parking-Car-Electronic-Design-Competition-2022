#ifndef __key_H
#define	__key_H

#include "stm32f10x.h"
#include "sys.h" 

//�ⲿ�жϰ�������   KEY-PB0  KEY-PB4  KEY-PA12
#define kEY1 			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0)
#define kEY2 			GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)
#define kEY3 			GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_12)

//����������      PA4
#define BEEP_ON         GPIO_ResetBits(GPIOA,GPIO_Pin_4)
#define BEEP_OFF        GPIO_SetBits(GPIOA,GPIO_Pin_4)


/* ֱ�Ӳ����Ĵ����ķ�������IO */
/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define	digitalHi(p,i)		 {p->BSRR=i;}	 //���Ϊ�ߵ�ƽ		
#define digitalLo(p,i)		 {p->BRR=i;}	 //����͵�ƽ
#define digitalToggle(p,i) 	 {p->ODR ^=i;}	 //�����ת״̬



//LED����        PC13 ����led
#define LED_TOGGLE      digitalToggle(GPIOC,GPIO_Pin_13)  //
#define LED_OFF         digitalHi(GPIOC,GPIO_Pin_13)
#define LED_ON      	digitalLo(GPIOC,GPIO_Pin_13)


/* �ṩ������C�ļ����õĺ��� --------------------------------------------------------------------------------------------*/
void LED_BEEP_GPIO_Config(void);
void EXTIX_Init(void);




#endif
