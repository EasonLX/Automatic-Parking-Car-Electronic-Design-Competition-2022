/**
  *************************************************************************************************************************
  * @file    main.c
  * @author  aaXinLiu
  * @version V1.0
  * @date    2022-10-13
  * @brief   2022年十月份全国大学生电子设计竞赛-自动泊车系统
  *************************************************************************************************************************
  * @attention
  * 	
  *编码器1-右电机      编码器2-左电机    串口          MPU6050 	     KEY 	          SERVO   	 			 BEEP           
  *PA0/PA1---TIM2      PB6/PB7---TIM4    TXD-PB10      SCL-PB8       KEY1-PB0	      SigLine-PA6      SigLine-PA4   
  *电机1               电机2             RXD-PB11     SDA-PB9	     KEY2-PB4		 		 		
  *PB14/PB15           PB12/PB13                       INT-PB5		   KEY3-PA12				 	
  *PWM1                PWM2																					
  *PA8                 PA11																									 
  *	
  *************************************************************************************************************************
  */ 

/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sys.h" 
#define B 0.15f  //车左右轮间距 单位米
#define L 0.163f //车前后轮间距 单位米

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
	NVIC_Config();//外部中断配置	
	LED_BEEP_GPIO_Config();//LED蜂鸣器初始化
	EXTIX_Init();//按键初始化-外部中断触发
	Usart3_Init(115200);//摄像头对应串口3初始化-115200
	Motor_Init();//电机初始化
	Encoder_TIM2_Init();//电机编码器配置
	Encoder_TIM4_Init();//电机编码器配置
	Usart2_Init(9600);
	PWM_Init_TIM1(0,7199);//10khz频率驱动电机
//	Set_PWM(motor1,motor2);
	PID_Param_Init();//pid参数初始化	
	MPU6050_EXTI_Init();//MPU6050外部中断初始化	
	TIM3_PWM_Init(TIM3_ARR,TIM3_PSC);//50hz 周期20ms-舵机初始化
	MPU_Init();					//初始化MPU6050
	BEEP_OFF;//初始蜂鸣器关闭
	while(mpu_dmp_init())
	{
		delay_ms(20);
	}
	///////////////////////////////////////////////test
	
	
//	Kinematic_Analysis(0.0,0.0,0.0,0.0);//小车停止 
//	
//	
////	PID.Motor1_Velocity_Target_Val=120;
////	PID.Motor2_Velocity_Target_Val=120;
//	
	while(Flag.Run_Step == 0)//等待开始运行指令
	{
		Kinematic_Analysis(0.0,0.0,0.0,0.0);//小车停止 	
		LED_ON;
		delay_ms(20);
		LED_OFF;
		while(Param.ModeChoose==0){;}//等待按键按下，确定模式
		Flag.Run_Step=1;//开始运行
		Flag.Is_Go_straight=1;//走直线				
	}		
	switch(Param.ModeChoose)
	{
		case BACK_PACKING:Usart3_SendString("startcnt1");break;	//根据模式选择开启倒车入库识别函数	
		case SIDE_PACKING:Usart3_SendString("startcnt2");break; //根据模式选择开启侧方停车识别函数	
		case BACK_SIDE_PACKING:Usart3_SendString("startcnt1");break;//根据模式先选择开启倒车入库识别函数,后续改变
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
//		if(Usart_GetData(&d1,&d2,&d3))  //解析数据
//		{
//			motor1=(int)d1;
//			motor2=(int)d2;
//		}
		
//		printf("%f,%f,%f,%f,%f,%d\r\n",PID.Motor2_Velocity_Kp,PID.Motor2_Velocity_Ki,PID.Motor2_Velocity_Kd,PID.Motor2_Velocity_Target_Val,PID.Motor2_Velocity_Actual_Val,Param.Motor2_PWM_Out);      //打印欧拉角
//		printf("%f,%f,%f,%f,%f,%f\r\n",PID.Motor1_Velocity_Target_Val,PID.Motor2_Velocity_Target_Val,PID.Motor1_Velocity_Actual_Val,PID.Motor2_Velocity_Actual_Val,Param.Motor1_PWM_Out,Param.Motor2_PWM_Out);
//		printf("%f,%f,%f,%f,%f,%f,%f,%f\r\n",PID.Servo_Position_Kp,PID.Servo_Position_Ki,PID.Servo_Position_Kd,Param.Servo_Target_Angle,Position_PID_Angle_OUT,Position_PID_PWM_OUT,1850.0,Yaw);
		//输出kp,ki,kd,目标角度,目标pwm，实际pwm
//		printf("%f,%f,%f\r\n",Pitch,Roll,Yaw);      //打印欧拉角
//		printf("%f,%f,%f\r\n",Param.Servo_Target_Angle,Param.Actual_Angle,Param.Position_PID_PWM_OUT);      //打印欧拉角
//		if(count_USART2>29)
//		{
//			count_USART2=0;
//		}
	}
}

/**
 * 函数名:EXTI4_IRQHandler
 * 描述:外部中断-KEY1服务程序
 * 输入:无
 * 输出:无
 */
void EXTI0_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(kEY1==0)	 //按键KEY1
	{		
		Param.ModeChoose=BACK_PACKING;//只进行倒车入库操作 
	}
	EXTI_ClearITPendingBit(EXTI_Line0);  //清除LINE10上的中断标志位  
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(kEY2==0)	 //按键KEY2
	{				
		Param.ModeChoose=SIDE_PACKING;//只进行侧方入库操作  
	}
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE10上的中断标志位  
}


/**
 * 函数名:EXTI15_10_IRQHandler
 * 描述:外部中断-KEY3服务程序
 * 输入:无
 * 输出:无
 */
void EXTI15_10_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(kEY3==0)	 //按键KEY3
	{	
		
//		Param.ModeChoose=BACK_SIDE_PACKING;//进行连续倒车入库和侧方停车操作  
	}
	EXTI_ClearITPendingBit(EXTI_Line12);  //清除LINE10上的中断标志位  
}
