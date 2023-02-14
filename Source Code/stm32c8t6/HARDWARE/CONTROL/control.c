#include "control.h"

Param_InitTypedef Param;
Flag_InitTypedef  Flag;
float Pitch,Roll,Yaw; 		//欧拉角
float Yaw_Last;
u8 Driving_Mode=0;
#define B 0.15f
#define L 0.163f


/**
 * 函数名:EXTI9_5_IRQHandler
 * 描述:陀螺仪中断引脚10ms定时
 * 输入:无
 * 输出:无
 */
void EXTI9_5_IRQHandler(void)
{
	static uint16_t Time_Cnt,Timer_Cnt;

	if(EXTI_GetITStatus(EXTI_Line5)!=0)//一级判定
	{
		if(PBin(5)==0)//二级判定
		{
			EXTI_ClearITPendingBit(EXTI_Line5);//清除中断标志位	
			
			mpu_dmp_get_data(&Pitch,&Roll,&Yaw);//获取欧拉角
			
			Position_PID_Servo_Realize();//舵机pid计算		
		
			Param.UnitTime_Motor1Pluse=-(short)Read_Speed(2);//获取单位时间内脉冲数
			Param.UnitTime_Motor2Pluse=(short)Read_Speed(4);//获取单位时间内脉冲数
			Param.Motor1_PWM_Out=VelocityRing_MOTOR1_Control();//电机1PID计算
			Param.Motor2_PWM_Out=VelocityRing_MOTOR2_Control();//电机2PID计算			
			Limit(&Param.Motor1_PWM_Out,&Param.Motor2_PWM_Out);//限幅	
			Set_PWM(Param.Motor1_PWM_Out,Param.Motor2_PWM_Out);//装载	
					
			//定时	
			if(++Time_Cnt==10)//闪灯，检验程序是否复位成功
			{
				Time_Cnt=0;
			}		

			if(Flag.Start_Count==1) //开启计数
			{
				if(++Timer_Cnt==Param.Timer_threshold_value)//定时时间到
				{
					Timer_Cnt=0;//清零计数值
					Flag.Is_Timer_Up=1;//定时时间到标志位置1
				}
			}	
			
		}
	}
}
/**
 * 函数名:Kinematic_Analysis
 * 描述:小车运动分析函数
 * 输入:目标方向角度，实际角度（+ / - Yaw），velocity:输入期望速度(velocity1=>电机1 velocity2=>电机2) 
 * 输出:无
 */
void Kinematic_Analysis(float Target_Angle,float Actual_Angle,float velocity1,float velocity2)
{
	float angle;
	angle=(1849.5-Param.Position_PID_PWM_OUT)/5*6;
	
	Param.Servo_Target_Angle=Target_Angle;
	Param.Actual_Angle=Actual_Angle;
	switch(Driving_Mode)//差速模式选择
	{
		case 0://直行匀速
			PID.Motor1_Velocity_Target_Val=velocity1;
			PID.Motor2_Velocity_Target_Val=velocity2;			
		break;
		case 1://倒车右拐弯
			PID.Motor1_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);
		break;
		case 2://前进右拐弯
			PID.Motor1_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);
		break;
		case 3://倒车左拐弯
			PID.Motor1_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);
		break;
		case 4://前进左拐弯
			PID.Motor1_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);
		break;
		default://直行匀速
			PID.Motor1_Velocity_Target_Val=velocity1;
			PID.Motor2_Velocity_Target_Val=velocity2;
		break;
		
	}
	
//	//直行保持俩轮匀速
//	if((Flag.Is_Go_straight==1)||(Flag.Is_Start_Astern!=1))
//	{
//		PID.Motor1_Velocity_Target_Val=velocity1;
//		PID.Motor2_Velocity_Target_Val=velocity2;
//	}
//	//3.倒车保持俩轮差速
//	else if(Flag.Is_Start_Astern==1)
//	{
//		
//		PID.Motor1_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);  
//		PID.Motor2_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);	
//	}
//	else if(Flag.Is_Start_Turn_Straight==1)
//	{
//		PID.Motor1_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);  
//		PID.Motor2_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);
//	}
//	
}

void Control_Proc(void)
{
if(Param.ModeChoose==BACK_PACKING)
{	
	switch(Flag.Run_Step)
	{
		case 1://小车直行，同时检测串口数据等待停车
				if(Flag.Is_Go_straight==1)
				{
					Kinematic_Analysis(0,Yaw,100,100);
					openMv_Proc();//检测串口信息--停车
					if(Param.openMV_Data==1)
					{
						Flag.Is_Go_straight=0;//不执行直行函数
						Param.openMV_Data=0;						
						Flag.Run_Step=2;//跳转下一步			
					}
					
				}
		 break;
		 case 2:
				if(Flag.Start_Count==0)
				{
					Kinematic_Analysis(0,Yaw,100,100);
					Flag.Start_Count=1;
					Param.Timer_threshold_value=34 ;//计数周期为0.5s	
				}
				if(Flag.Is_Timer_Up==1)//定时时间到
				{
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Stop_Car=1;//开始停车
					Flag.Run_Step=3;//跳转下一步
				}
		 break;
		 case 3:
			 if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(0.0,0.0,0.0,0.0);
					Flag.Start_Count=1;
					Param.Timer_threshold_value=100 ;//计数周期为1s	
					BEEP_ON;//蜂鸣器响
				}
				if(Flag.Is_Timer_Up==1)//定时时间到
				{
					BEEP_OFF;//关蜂鸣器
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Stop_Car=0;//取消停车
					Flag.Is_Start_Astern=1;//开始倒车第一步
					Driving_Mode=1;//倒车右拐弯
					Flag.Run_Step=4;//跳转下一步
				}
												
		 break;
		 case 4:				
				//右转60°，倒车，差速转弯
					if((Flag.Is_Start_Astern==1)&&(Flag.Start_Count==0))
					{
						Kinematic_Analysis(-75,-Yaw,-350,-350);								
						Flag.Is_Turn_Car=1;//开始转弯
					}
					if(abs((int)Yaw)==75)//角度为-80°结束
					{
							Flag.Start_Count=0;//不开启计数
							Flag.Is_Timer_Up=0;//定时时间到标志位清零
							Flag.Is_Start_Astern=2;//开始倒车第二步
							Driving_Mode=0;//直行匀速
							Flag.Run_Step=5;//跳转下一步
					}	
		 break;
		case 5://车头归正，倒车，匀速，定时1.5s
				if((Flag.Is_Start_Astern==2)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(-70,-Yaw,-300,-300);				
					Flag.Start_Count=1;//开始计时
					Param.Timer_threshold_value=28;//定时					
				}
				if(Flag.Is_Timer_Up==1)//定时时间到
				{
						Flag.Start_Count=0;//不开启计数
						Flag.Is_Timer_Up=0;//定时时间到标志位清零
						Flag.Is_Start_Astern=0;//清零倒车步骤
						Flag.Is_Stop_Car=1;//停车
						Flag.Run_Step=6;//跳转下一步
				}	
		break;
		case 6://停车，蜂鸣器响，定时5s
				if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
				{
					Yaw_Last=Yaw;
					Kinematic_Analysis(0.0,0.0,0.0,0.0);
					Flag.Start_Count=1;//开始计时
					Param.Timer_threshold_value=500;//定时5s
					BEEP_ON;//蜂鸣器响
				}
				if(Flag.Is_Timer_Up==1)//定时时间到
				{
					BEEP_OFF;//关蜂鸣器
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Stop_Car=0;//不停车
					Driving_Mode=0;//直行匀速
					Flag.Run_Step=7;//跳转下一步
				}	
		break;
				/*出库*/
		case 7://直行，匀速,定时
				if(Flag.Is_Stop_Car==0)
				{
					Kinematic_Analysis(Yaw_Last-3,Yaw,300,300);
					Flag.Start_Count=1;//开始计时
					Param.Timer_threshold_value=29;//定时s
				}
				if(Flag.Is_Timer_Up==1)//定时时间到
				{
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Start_Turn_Straight=1;//前行转弯标志位置1
					Driving_Mode=2;//前进右拐弯
					Flag.Run_Step=8;//跳转下一步
				}	
		break;	
		case 8:		
		//右转60°，差速
				if((Flag.Is_Start_Turn_Straight==1)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(-0,Yaw,250,250);				
					Flag.Is_Turn_Car=1;//开始转弯				  
				}
				if(abs((int)Yaw)==0)//角度为0°结束
				{
					Flag.Is_Turn_Car=0;//不开启转弯	
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Start_Turn_Straight=0;//前行转弯标志位清零
					Flag.Is_Go_straight=1;//走直线
					Driving_Mode=0;	//直行匀速	
					Flag.Run_Step=9;//跳转下一步
				}	
		break;
		case 9://停车
				Kinematic_Analysis(0.0,0.0,0.0,0.0);			
		break;
	}
}
else if(Param.ModeChoose==SIDE_PACKING)
{
	switch(Flag.Run_Step)
	{
		case 1://小车直行，同时检测串口数据等待停车
				if(Flag.Is_Go_straight==1)
				{
					Kinematic_Analysis(0,Yaw,100,100);
					openMv_Proc();//检测串口信息--停车
					if(Param.openMV_Data==1)
					{
						Flag.Is_Go_straight=0;//不执行直行函数
						Param.openMV_Data=0;	
						Flag.Start_Count=0;
						Flag.Run_Step=2;//跳转下一步			
					}
					
				}
		 break;
		case 2:
			if(Flag.Start_Count==0)
			{
				Kinematic_Analysis(0,Yaw,100,100);
				Flag.Start_Count=1;
				Param.Timer_threshold_value=70;//计数周期为0.5s	
			}
			if(Flag.Is_Timer_Up==1)//定时时间到
			{
				Flag.Start_Count=0;//不开启计数
				Flag.Is_Timer_Up=0;//定时时间到标志位清零
				Flag.Is_Stop_Car=1;//开始停车
				Flag.Is_Go_straight=0;//不执行直行函数
				Flag.Is_Stop_Car=1;//开始停车
				Flag.Run_Step=3;//跳转下一步
			}

		break;	
		case 3:
			//停车1s,蜂鸣器响1s
			if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(0.0,0.0,0.0,0.0);
				Flag.Start_Count=1;//开始计数
				Param.Timer_threshold_value=100;//计数周期定为1s
				BEEP_ON;//蜂鸣器响
			}
			if(Flag.Is_Timer_Up==1)//定时时间到
			{
				BEEP_OFF;//关蜂鸣器
				Flag.Start_Count=0;//不开启计数
				Flag.Is_Timer_Up=0;//定时时间到标志位清零
				Flag.Is_Stop_Car=0;//取消停车
				Flag.Is_Start_Astern=1;//开始倒车第一步
				Driving_Mode=1;//倒车右拐弯
				Flag.Run_Step=4;//跳转下一步
			}
		break;
		case 4:
			//右转40°，倒车，差速转弯
			if((Flag.Is_Start_Astern==1)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(-30,-Yaw,-250,-250);								
				Flag.Is_Turn_Car=1;//开始转弯
			}
			if(abs((int)Yaw)==30)//角度为-40°结束
			{
					Yaw_Last=Yaw;
//					Flag.Is_Turn_Car=0;//不开启转弯
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Start_Astern=2;//开始倒车第二步
					Driving_Mode=0;//直行匀速	
					Flag.Run_Step=5;//跳转下一步
			}	
		break;
		case 5://回正，倒车，不差速转弯，定时1s
			if((Flag.Is_Start_Astern==2)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(Yaw_Last,-Yaw,-300,-300);								
				Flag.Start_Count=1;//开始计时
				Param.Timer_threshold_value=30;//定时1s					
			}
			if(Flag.Is_Timer_Up==1)//定时时间到
			{
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Is_Start_Astern=3;//倒车第三步
					Flag.Run_Step=6;//跳转下一步
			}		
		break;
		case 6:
			//车头归正，倒车，匀速，定时1.5s
			if((Flag.Is_Start_Astern==3)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(23,-Yaw,-300,-300);				
				Flag.Start_Count=1;//开始计时
				Param.Timer_threshold_value=40;//定时1.5s					
			}
			if(Flag.Is_Timer_Up==1)//定时时间到
			{
				Flag.Start_Count=0;//不开启计数
				Flag.Is_Timer_Up=0;//定时时间到标志位清零
				Flag.Is_Start_Astern=0;//清零倒车步骤
				Flag.Is_Stop_Car=1;//停车
				Flag.Run_Step=7;//跳转下一步
			}				
		break;
		case 7:
			//停车，蜂鸣器响，定时5s
			if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
			{
				Yaw_Last=Yaw;
				Kinematic_Analysis(0.0,0.0,0.0,0.0);
				Flag.Start_Count=1;//开始计时
				Param.Timer_threshold_value=500;//定时5s
				BEEP_ON;//蜂鸣器响
			}
			if(Flag.Is_Timer_Up==1)//定时时间到
			{
				BEEP_OFF;//关蜂鸣器
				Flag.Start_Count=0;//不开启计数
				Flag.Is_Timer_Up=0;//定时时间到标志位清零
				Flag.Is_Stop_Car=0;//不停车
				Driving_Mode=4;//前进左拐弯
				Flag.Run_Step=8;//跳转下一步
			}		
		break;
		case 8:
			//前进左转40°，差速
				if(Flag.Start_Count==0)
				{
					Kinematic_Analysis(40,Yaw,0,0);				
					Flag.Start_Count=1;//开始计时
					Param.Timer_threshold_value=15;//定时5s			  
				}
				if(Flag.Is_Timer_Up==1)//定时时间到
				{
					Flag.Start_Count=0;//不开启计数
					Driving_Mode=4;//前进左拐弯
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Flag.Run_Step=9;//跳转下一步
				}		
		break;
		case 9:
//			//前进左转40°，差速
				if(Flag.Start_Count==0)
				{
					Kinematic_Analysis(40,Yaw,180,180);							  
				}
				if(abs((int)Yaw)==40)//角度为0°结束
				{
//					Yaw_Last=Yaw;
					Flag.Start_Count=0;//不开启计数
					Flag.Is_Timer_Up=0;//定时时间到标志位清零
					Driving_Mode=0;		//直行匀速	
					Flag.Is_Go_straight=1;					
					Flag.Run_Step=10;//跳转下一步
				}

		break;
		case 10:
			if((Flag.Is_Go_straight==1)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(0,Yaw,200,200);
					Flag.Start_Count=1;
					Param.Timer_threshold_value=110;//计数周期为0.5s		
				}
			if(Flag.Is_Timer_Up==1)//定时时间到
			{
				Flag.Start_Count=0;//不开启计数
				Flag.Is_Timer_Up=0;//定时时间到标志位清零
				Flag.Run_Step=11;//跳转下一步
			}	
		break;
		
		case 11:
			Kinematic_Analysis(0.0,0.0,0.0,0.0);//停车
		break;
	}
}
}
















