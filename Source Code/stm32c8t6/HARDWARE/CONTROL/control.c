#include "control.h"

Param_InitTypedef Param;
Flag_InitTypedef  Flag;
float Pitch,Roll,Yaw; 		//ŷ����
float Yaw_Last;
u8 Driving_Mode=0;
#define B 0.15f
#define L 0.163f


/**
 * ������:EXTI9_5_IRQHandler
 * ����:�������ж�����10ms��ʱ
 * ����:��
 * ���:��
 */
void EXTI9_5_IRQHandler(void)
{
	static uint16_t Time_Cnt,Timer_Cnt;

	if(EXTI_GetITStatus(EXTI_Line5)!=0)//һ���ж�
	{
		if(PBin(5)==0)//�����ж�
		{
			EXTI_ClearITPendingBit(EXTI_Line5);//����жϱ�־λ	
			
			mpu_dmp_get_data(&Pitch,&Roll,&Yaw);//��ȡŷ����
			
			Position_PID_Servo_Realize();//���pid����		
		
			Param.UnitTime_Motor1Pluse=-(short)Read_Speed(2);//��ȡ��λʱ����������
			Param.UnitTime_Motor2Pluse=(short)Read_Speed(4);//��ȡ��λʱ����������
			Param.Motor1_PWM_Out=VelocityRing_MOTOR1_Control();//���1PID����
			Param.Motor2_PWM_Out=VelocityRing_MOTOR2_Control();//���2PID����			
			Limit(&Param.Motor1_PWM_Out,&Param.Motor2_PWM_Out);//�޷�	
			Set_PWM(Param.Motor1_PWM_Out,Param.Motor2_PWM_Out);//װ��	
					
			//��ʱ	
			if(++Time_Cnt==10)//���ƣ���������Ƿ�λ�ɹ�
			{
				Time_Cnt=0;
			}		

			if(Flag.Start_Count==1) //��������
			{
				if(++Timer_Cnt==Param.Timer_threshold_value)//��ʱʱ�䵽
				{
					Timer_Cnt=0;//�������ֵ
					Flag.Is_Timer_Up=1;//��ʱʱ�䵽��־λ��1
				}
			}	
			
		}
	}
}
/**
 * ������:Kinematic_Analysis
 * ����:С���˶���������
 * ����:Ŀ�귽��Ƕȣ�ʵ�ʽǶȣ�+ / - Yaw����velocity:���������ٶ�(velocity1=>���1 velocity2=>���2) 
 * ���:��
 */
void Kinematic_Analysis(float Target_Angle,float Actual_Angle,float velocity1,float velocity2)
{
	float angle;
	angle=(1849.5-Param.Position_PID_PWM_OUT)/5*6;
	
	Param.Servo_Target_Angle=Target_Angle;
	Param.Actual_Angle=Actual_Angle;
	switch(Driving_Mode)//����ģʽѡ��
	{
		case 0://ֱ������
			PID.Motor1_Velocity_Target_Val=velocity1;
			PID.Motor2_Velocity_Target_Val=velocity2;			
		break;
		case 1://�����ҹ���
			PID.Motor1_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);
		break;
		case 2://ǰ���ҹ���
			PID.Motor1_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);
		break;
		case 3://���������
			PID.Motor1_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);
		break;
		case 4://ǰ�������
			PID.Motor1_Velocity_Target_Val=velocity1*(1+B*tan(angle)/2/L);  
			PID.Motor2_Velocity_Target_Val=velocity1*(1-B*tan(angle)/2/L);
		break;
		default://ֱ������
			PID.Motor1_Velocity_Target_Val=velocity1;
			PID.Motor2_Velocity_Target_Val=velocity2;
		break;
		
	}
	
//	//ֱ�б�����������
//	if((Flag.Is_Go_straight==1)||(Flag.Is_Start_Astern!=1))
//	{
//		PID.Motor1_Velocity_Target_Val=velocity1;
//		PID.Motor2_Velocity_Target_Val=velocity2;
//	}
//	//3.�����������ֲ���
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
		case 1://С��ֱ�У�ͬʱ��⴮�����ݵȴ�ͣ��
				if(Flag.Is_Go_straight==1)
				{
					Kinematic_Analysis(0,Yaw,100,100);
					openMv_Proc();//��⴮����Ϣ--ͣ��
					if(Param.openMV_Data==1)
					{
						Flag.Is_Go_straight=0;//��ִ��ֱ�к���
						Param.openMV_Data=0;						
						Flag.Run_Step=2;//��ת��һ��			
					}
					
				}
		 break;
		 case 2:
				if(Flag.Start_Count==0)
				{
					Kinematic_Analysis(0,Yaw,100,100);
					Flag.Start_Count=1;
					Param.Timer_threshold_value=34 ;//��������Ϊ0.5s	
				}
				if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
				{
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Stop_Car=1;//��ʼͣ��
					Flag.Run_Step=3;//��ת��һ��
				}
		 break;
		 case 3:
			 if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(0.0,0.0,0.0,0.0);
					Flag.Start_Count=1;
					Param.Timer_threshold_value=100 ;//��������Ϊ1s	
					BEEP_ON;//��������
				}
				if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
				{
					BEEP_OFF;//�ط�����
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Stop_Car=0;//ȡ��ͣ��
					Flag.Is_Start_Astern=1;//��ʼ������һ��
					Driving_Mode=1;//�����ҹ���
					Flag.Run_Step=4;//��ת��һ��
				}
												
		 break;
		 case 4:				
				//��ת60�㣬����������ת��
					if((Flag.Is_Start_Astern==1)&&(Flag.Start_Count==0))
					{
						Kinematic_Analysis(-75,-Yaw,-350,-350);								
						Flag.Is_Turn_Car=1;//��ʼת��
					}
					if(abs((int)Yaw)==75)//�Ƕ�Ϊ-80�����
					{
							Flag.Start_Count=0;//����������
							Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
							Flag.Is_Start_Astern=2;//��ʼ�����ڶ���
							Driving_Mode=0;//ֱ������
							Flag.Run_Step=5;//��ת��һ��
					}	
		 break;
		case 5://��ͷ���������������٣���ʱ1.5s
				if((Flag.Is_Start_Astern==2)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(-70,-Yaw,-300,-300);				
					Flag.Start_Count=1;//��ʼ��ʱ
					Param.Timer_threshold_value=28;//��ʱ					
				}
				if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
				{
						Flag.Start_Count=0;//����������
						Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
						Flag.Is_Start_Astern=0;//���㵹������
						Flag.Is_Stop_Car=1;//ͣ��
						Flag.Run_Step=6;//��ת��һ��
				}	
		break;
		case 6://ͣ�����������죬��ʱ5s
				if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
				{
					Yaw_Last=Yaw;
					Kinematic_Analysis(0.0,0.0,0.0,0.0);
					Flag.Start_Count=1;//��ʼ��ʱ
					Param.Timer_threshold_value=500;//��ʱ5s
					BEEP_ON;//��������
				}
				if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
				{
					BEEP_OFF;//�ط�����
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Stop_Car=0;//��ͣ��
					Driving_Mode=0;//ֱ������
					Flag.Run_Step=7;//��ת��һ��
				}	
		break;
				/*����*/
		case 7://ֱ�У�����,��ʱ
				if(Flag.Is_Stop_Car==0)
				{
					Kinematic_Analysis(Yaw_Last-3,Yaw,300,300);
					Flag.Start_Count=1;//��ʼ��ʱ
					Param.Timer_threshold_value=29;//��ʱs
				}
				if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
				{
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Start_Turn_Straight=1;//ǰ��ת���־λ��1
					Driving_Mode=2;//ǰ���ҹ���
					Flag.Run_Step=8;//��ת��һ��
				}	
		break;	
		case 8:		
		//��ת60�㣬����
				if((Flag.Is_Start_Turn_Straight==1)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(-0,Yaw,250,250);				
					Flag.Is_Turn_Car=1;//��ʼת��				  
				}
				if(abs((int)Yaw)==0)//�Ƕ�Ϊ0�����
				{
					Flag.Is_Turn_Car=0;//������ת��	
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Start_Turn_Straight=0;//ǰ��ת���־λ����
					Flag.Is_Go_straight=1;//��ֱ��
					Driving_Mode=0;	//ֱ������	
					Flag.Run_Step=9;//��ת��һ��
				}	
		break;
		case 9://ͣ��
				Kinematic_Analysis(0.0,0.0,0.0,0.0);			
		break;
	}
}
else if(Param.ModeChoose==SIDE_PACKING)
{
	switch(Flag.Run_Step)
	{
		case 1://С��ֱ�У�ͬʱ��⴮�����ݵȴ�ͣ��
				if(Flag.Is_Go_straight==1)
				{
					Kinematic_Analysis(0,Yaw,100,100);
					openMv_Proc();//��⴮����Ϣ--ͣ��
					if(Param.openMV_Data==1)
					{
						Flag.Is_Go_straight=0;//��ִ��ֱ�к���
						Param.openMV_Data=0;	
						Flag.Start_Count=0;
						Flag.Run_Step=2;//��ת��һ��			
					}
					
				}
		 break;
		case 2:
			if(Flag.Start_Count==0)
			{
				Kinematic_Analysis(0,Yaw,100,100);
				Flag.Start_Count=1;
				Param.Timer_threshold_value=70;//��������Ϊ0.5s	
			}
			if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
			{
				Flag.Start_Count=0;//����������
				Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
				Flag.Is_Stop_Car=1;//��ʼͣ��
				Flag.Is_Go_straight=0;//��ִ��ֱ�к���
				Flag.Is_Stop_Car=1;//��ʼͣ��
				Flag.Run_Step=3;//��ת��һ��
			}

		break;	
		case 3:
			//ͣ��1s,��������1s
			if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(0.0,0.0,0.0,0.0);
				Flag.Start_Count=1;//��ʼ����
				Param.Timer_threshold_value=100;//�������ڶ�Ϊ1s
				BEEP_ON;//��������
			}
			if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
			{
				BEEP_OFF;//�ط�����
				Flag.Start_Count=0;//����������
				Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
				Flag.Is_Stop_Car=0;//ȡ��ͣ��
				Flag.Is_Start_Astern=1;//��ʼ������һ��
				Driving_Mode=1;//�����ҹ���
				Flag.Run_Step=4;//��ת��һ��
			}
		break;
		case 4:
			//��ת40�㣬����������ת��
			if((Flag.Is_Start_Astern==1)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(-30,-Yaw,-250,-250);								
				Flag.Is_Turn_Car=1;//��ʼת��
			}
			if(abs((int)Yaw)==30)//�Ƕ�Ϊ-40�����
			{
					Yaw_Last=Yaw;
//					Flag.Is_Turn_Car=0;//������ת��
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Start_Astern=2;//��ʼ�����ڶ���
					Driving_Mode=0;//ֱ������	
					Flag.Run_Step=5;//��ת��һ��
			}	
		break;
		case 5://������������������ת�䣬��ʱ1s
			if((Flag.Is_Start_Astern==2)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(Yaw_Last,-Yaw,-300,-300);								
				Flag.Start_Count=1;//��ʼ��ʱ
				Param.Timer_threshold_value=30;//��ʱ1s					
			}
			if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
			{
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Is_Start_Astern=3;//����������
					Flag.Run_Step=6;//��ת��һ��
			}		
		break;
		case 6:
			//��ͷ���������������٣���ʱ1.5s
			if((Flag.Is_Start_Astern==3)&&(Flag.Start_Count==0))
			{
				Kinematic_Analysis(23,-Yaw,-300,-300);				
				Flag.Start_Count=1;//��ʼ��ʱ
				Param.Timer_threshold_value=40;//��ʱ1.5s					
			}
			if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
			{
				Flag.Start_Count=0;//����������
				Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
				Flag.Is_Start_Astern=0;//���㵹������
				Flag.Is_Stop_Car=1;//ͣ��
				Flag.Run_Step=7;//��ת��һ��
			}				
		break;
		case 7:
			//ͣ�����������죬��ʱ5s
			if((Flag.Is_Stop_Car==1)&&(Flag.Start_Count==0))
			{
				Yaw_Last=Yaw;
				Kinematic_Analysis(0.0,0.0,0.0,0.0);
				Flag.Start_Count=1;//��ʼ��ʱ
				Param.Timer_threshold_value=500;//��ʱ5s
				BEEP_ON;//��������
			}
			if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
			{
				BEEP_OFF;//�ط�����
				Flag.Start_Count=0;//����������
				Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
				Flag.Is_Stop_Car=0;//��ͣ��
				Driving_Mode=4;//ǰ�������
				Flag.Run_Step=8;//��ת��һ��
			}		
		break;
		case 8:
			//ǰ����ת40�㣬����
				if(Flag.Start_Count==0)
				{
					Kinematic_Analysis(40,Yaw,0,0);				
					Flag.Start_Count=1;//��ʼ��ʱ
					Param.Timer_threshold_value=15;//��ʱ5s			  
				}
				if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
				{
					Flag.Start_Count=0;//����������
					Driving_Mode=4;//ǰ�������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Flag.Run_Step=9;//��ת��һ��
				}		
		break;
		case 9:
//			//ǰ����ת40�㣬����
				if(Flag.Start_Count==0)
				{
					Kinematic_Analysis(40,Yaw,180,180);							  
				}
				if(abs((int)Yaw)==40)//�Ƕ�Ϊ0�����
				{
//					Yaw_Last=Yaw;
					Flag.Start_Count=0;//����������
					Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
					Driving_Mode=0;		//ֱ������	
					Flag.Is_Go_straight=1;					
					Flag.Run_Step=10;//��ת��һ��
				}

		break;
		case 10:
			if((Flag.Is_Go_straight==1)&&(Flag.Start_Count==0))
				{
					Kinematic_Analysis(0,Yaw,200,200);
					Flag.Start_Count=1;
					Param.Timer_threshold_value=110;//��������Ϊ0.5s		
				}
			if(Flag.Is_Timer_Up==1)//��ʱʱ�䵽
			{
				Flag.Start_Count=0;//����������
				Flag.Is_Timer_Up=0;//��ʱʱ�䵽��־λ����
				Flag.Run_Step=11;//��ת��һ��
			}	
		break;
		
		case 11:
			Kinematic_Analysis(0.0,0.0,0.0,0.0);//ͣ��
		break;
	}
}
}
















