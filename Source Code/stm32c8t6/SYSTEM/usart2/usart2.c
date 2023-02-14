#include "usart2.h"

void Usart2_Init(u32 bound)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	//PA2 TX
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	//PA3 RX
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate=bound;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode=USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_Parity=USART_Parity_No;
	USART_InitStruct.USART_StopBits=USART_StopBits_1;
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART2,&USART_InitStruct);
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);//开启中断接收
	USART_Cmd(USART2,ENABLE);
}

void USART2_Send_Data(char data)
{
	USART_SendData(USART2,data);//串口2发送数据
	while(USART_GetFlagStatus(USART2,USART_FLAG_TC)!=1);//等待发送完成
}

//一串
void USART2_Send_String(char *String)
{
	u16 len,j;
	
	len=strlen(String);
	for(j=0;j<len;j++)
	{
		USART2_Send_Data(*String++);
	}
}
//addr 要发送的数组的首地址
//size 数据长度
void Usart2_SendNByte(u8* addr,int size)
{
	while(size--)
	{
		while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == RESET);//等待发送完成，STM32标准库函数
		USART_SendData(USART2,*addr);//发送数据 ，STM32标准库函数
		addr++; //地址偏移
	}
}
//float data1,data2,data3,data4=0;
//void SendWave(void)
//{
//	u8 temp[] = {0x41,0x42,0x43,0x44,0x45}; //帧头
//	float channel[4];    //数据通道，float型
//	
//	data1=0.0;
//	data2=1.0;
//	data3=2.0;
//	channel[0] =  data1;    //通道赋值
//	channel[1] =  data2;
//	channel[2] =  data3;
//	channel[3] =  data4;
//	Usart2_SendNByte((u8*)temp,sizeof(temp)); //发送帧头
//	Usart2_SendNByte((u8*)channel,sizeof(channel)); //发送数据
//}

/// 重定向c库函数printf到USART1
int fputc(int ch, FILE *f)
{
        /* 发送一个字节数据到USART1 */
        USART_SendData(USART2, (uint8_t) ch);
        
        /* 等待发送完毕 */
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);        
    
        return (ch);
}

/// 重定向c库函数scanf到USART1
int fgetc(FILE *f)
{
        /* 等待串口1输入数据 */
        while (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == RESET);

        return (int)USART_ReceiveData(USART2);
}
char Usart_ReadBuff1[30] = {0};
u8 Usart_ReadOk = 0;
u8 count_USART2=0;
void USART2_IRQHandler(void)
{
//    uint8_t ch;
    u8 temp;

    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {     
        //ch = USART1->DR;
//		ch = USART_ReceiveData(USART2);
		temp=USART_ReceiveData(USART2);
		if(Usart_ReadOk==0)
		{
			Usart_ReadBuff1[count_USART2]=temp;
			count_USART2++;
			if(temp=='#')
			{
				count_USART2=0;
				Usart_ReadOk=1;
			}
		}
		
//         printf( "%c", ch );    //将接受到的数据直接返回打印
//		 printf( "%c", temp ); 
    } 
     
}
