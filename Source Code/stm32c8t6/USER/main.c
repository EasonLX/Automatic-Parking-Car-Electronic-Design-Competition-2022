/**
  *************************************************************************************************************************
  * @file    main.c
  * @author  aaXinLiu
  * @version V1.0
  * @date    2022-10-13
  * @brief   2022��ʮ�·�ȫ����ѧ��������ƾ���-�Զ�����ϵͳ
  *************************************************************************************************************************
  * @attention
  * 	
  *������1-�ҵ��      ������2-����    ����          MPU6050 	     KEY 	          SERVO   	 			 BEEP           
  *PA0/PA1---TIM2      PB6/PB7---TIM4    TXD-PB10      SCL-PB8       KEY1-PB0	      SigLine-PA6      SigLine-PA4   
  *���1               ���2             RXD-PB11     SDA-PB9	     KEY2-PB4		 		 		
  *PB14/PB15           PB12/PB13                       INT-PB5		   KEY3-PA12				 	
  *PWM1                PWM2																					
  *PA8                 PA11																									 
  *	
  *************************************************************************************************************************
  */ 

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sys.h" 
#define B 0.15f  //�������ּ�� ��λ��
#define L 0.163f //��ǰ���ּ�� ��λ��

// u8 Usart_GetData(float *D1,float *D2,float *D3)
//{
//	u8 flag=0;
//	if(Usart_ReadOk==1)
//	{
//		if(sscanf(Usart_ReadBuff1,"PID=%f,%f,%f#",D1,D2,D3)==3)
//		{
//			flag=1;
//		}
//		Usart_ReadOk=0;
//		memset(Usart_ReadBuff1,0,sizeof(Usart_ReadBuff1));
//	}
//	return flag;
//}
int main(void)	
{
//	float d1,d2,d3;
//	float angle;
//	int motor1=-200,motor2=-200;
	delay_init();
	NVIC_Config();//�ⲿ�ж�����	
	LED_BEEP_GPIO_Config();//LED��������ʼ��
	EXTIX_Init();//������ʼ��-�ⲿ�жϴ���
	Usart3_Init(115200);//����ͷ��Ӧ����3��ʼ��-115200
	Motor_Init();//�����ʼ��
	Encoder_TIM2_Init();//�������������
	Encoder_TIM4_Init();//�������������
	Usart2_Init(9600);
	PWM_Init_TIM1(0,7199);//10khzƵ���������
//	Set_PWM(motor1,motor2);
	PID_Param_Init();//pid������ʼ��	
	MPU6050_EXTI_Init();//MPU6050�ⲿ�жϳ�ʼ��	
	TIM3_PWM_Init(TIM3_ARR,TIM3_PSC);//50hz ����20ms-�����ʼ��
	MPU_Init();					//��ʼ��MPU6050
	BEEP_OFF;//��ʼ�������ر�
	while(mpu_dmp_init())
	{
		delay_ms(20);
	}
	///////////////////////////////////////////////test
	
	
//	Kinematic_Analysis(0.0,0.0,0.0,0.0);//С��ֹͣ 
//	
//	
////	PID.Motor1_Velocity_Target_Val=120;
////	PID.Motor2_Velocity_Target_Val=120;
//	
	while(Flag.Run_Step == 0)//�ȴ���ʼ����ָ��
	{
		Kinematic_Analysis(0.0,0.0,0.0,0.0);//С��ֹͣ 	
		LED_ON;
		delay_ms(20);
		LED_OFF;
		while(Param.ModeChoose==0){;}//�ȴ��������£�ȷ��ģʽ
		Flag.Run_Step=1;//��ʼ����
		Flag.Is_Go_straight=1;//��ֱ��				
	}		
	switch(Param.ModeChoose)
	{
		case BACK_PACKING:Usart3_SendString("startcnt1");break;	//����ģʽѡ�����������ʶ����	
		case SIDE_PACKING:Usart3_SendString("startcnt2");break; //����ģʽѡ�����෽ͣ��ʶ����	
		case BACK_SIDE_PACKING:Usart3_SendString("startcnt1");break;//����ģʽ��ѡ�����������ʶ����,�����ı�
		default:break;		
	}
	//////////////////////////////////////////////
	while(1)
	{
//		Param.Servo_Target_Angle=-80;
//		Param.Actual_Angle=-Yaw;
////		Kinematic_Analysis(-80,-Yaw,-100,-100);
//		angle=(1849.5-Param.Position_PID_PWM_OUT)/5*6;
//		PID.Motor1_Velocity_Target_Val=motor1*(1-B*tan(60)/2/L);  
//		PID.Motor2_Velocity_Target_Val=motor2*(1+B*tan(60)/2/L);
//		printf("%f,%f,%f\r\n",PID.Motor1_Velocity_Target_Val,PID.Motor2_Velocity_Target_Val,(float)motor1);
		Control_Proc();
//		if(Usart_GetData(&d1,&d2,&d3))  //��������
//		{
//			motor1=(int)d1;
//			motor2=(int)d2;
//		}
		
//		printf("%f,%f,%f,%f,%f,%d\r\n",PID.Motor2_Velocity_Kp,PID.Motor2_Velocity_Ki,PID.Motor2_Velocity_Kd,PID.Motor2_Velocity_Target_Val,PID.Motor2_Velocity_Actual_Val,Param.Motor2_PWM_Out);      //��ӡŷ����
//		printf("%f,%f,%f,%f,%f,%f\r\n",PID.Motor1_Velocity_Target_Val,PID.Motor2_Velocity_Target_Val,PID.Motor1_Velocity_Actual_Val,PID.Motor2_Velocity_Actual_Val,Param.Motor1_PWM_Out,Param.Motor2_PWM_Out);
//		printf("%f,%f,%f,%f,%f,%f,%f,%f\r\n",PID.Servo_Position_Kp,PID.Servo_Position_Ki,PID.Servo_Position_Kd,Param.Servo_Target_Angle,Position_PID_Angle_OUT,Position_PID_PWM_OUT,1850.0,Yaw);
		//���kp,ki,kd,Ŀ��Ƕ�,Ŀ��pwm��ʵ��pwm
//		printf("%f,%f,%f\r\n",Pitch,Roll,Yaw);      //��ӡŷ����
//		printf("%f,%f,%f\r\n",Param.Servo_Target_Angle,Param.Actual_Angle,Param.Position_PID_PWM_OUT);      //��ӡŷ����
//		if(count_USART2>29)
//		{
//			count_USART2=0;
//		}
	}
}

/**
 * ������:EXTI4_IRQHandler
 * ����:�ⲿ�ж�-KEY1�������
 * ����:��
 * ���:��
 */
void EXTI0_IRQHandler(void)
{
	delay_ms(10);//����
	if(kEY1==0)	 //����KEY1
	{		
		Param.ModeChoose=BACK_PACKING;//ֻ���е��������� 
	}
	EXTI_ClearITPendingBit(EXTI_Line0);  //���LINE10�ϵ��жϱ�־λ  
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//����
	if(kEY2==0)	 //����KEY2
	{				
		Param.ModeChoose=SIDE_PACKING;//ֻ���в෽������  
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  //���LINE10�ϵ��жϱ�־λ  
}


/**
 * ������:EXTI15_10_IRQHandler
 * ����:�ⲿ�ж�-KEY3�������
 * ����:��
 * ���:��
 */
void EXTI15_10_IRQHandler(void)
{
	delay_ms(10);//����
	if(kEY3==0)	 //����KEY3
	{	
		
//		Param.ModeChoose=BACK_SIDE_PACKING;//���������������Ͳ෽ͣ������  
	}
	EXTI_ClearITPendingBit(EXTI_Line12);  //���LINE10�ϵ��жϱ�־λ  
}
