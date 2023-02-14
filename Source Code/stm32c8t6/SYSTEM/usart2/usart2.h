#ifndef _usart2_H
#define _usart2_H
#include "stm32f10x.h"
#include "sys.h"


extern u8 count_USART2;
extern char Usart_ReadBuff1[30];
extern u8 Usart_ReadOk;
//extern float data1,data2,data3,data4;
void Usart2_Init(u32 bound);
void SendWave(void);
void USART2_IRQHandler(void);
//void USART2_Send_Data(char data);
//void USART2_Send_String(char *String);
//void SendWave(void);

#endif
