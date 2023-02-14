#include "pid.h"
#include "control.h"
PID_InitTypedef PID;

/**
 * ������:PID_Para_Init
 * ����:PID������ʼ��
 * ����:��
 * ���:��
 */
 void PID_Param_Init(void)
 {
	 /*λ��ʽPID-���*/
	 PID.Servo_Position_Kp=7.5;
	 PID.Servo_Position_Ki=0.0;
	 PID.Servo_Position_Kd=0.01;
	 /*λ��ʽPID-���1*/
	 PID.Motor1_Velocity_Kp=10.0; 
	 PID.Motor1_Velocity_Ki=6.0;
	 PID.Motor1_Velocity_Kd=0.0;
	 /*λ��ʽPID-���2*/
	 PID.Motor2_Velocity_Kp=10.0; 
	 PID.Motor2_Velocity_Ki=6.0;
	 PID.Motor2_Velocity_Kd=0.0;
 }
/**
 * ������:Position_PID_Servo
 * ����:���λ��ʽPID�㷨ʵ��
 * ����:Target_Val:Ŀ��Ƕ�
 * ���:PID.Location_Out:λ�û�PID���ֵ
 */
float Position_PID_Servo(float Target_Angle,float yaw)
{
	static float Error,Integral_Error,Error_Last,Actual_Angle,PWM_OUT;
	Actual_Angle=yaw;
	//2.����ƫ��
	Error=Target_Angle-Actual_Angle;
	//ƫ���˲�
//	if(Error>(-0.3)&&Error<0.3)
//	{
//		Error=Error_Last;
//	}
	//3.�ۼ�ƫ��
	Integral_Error+=Error;
	//4.PID�㷨
	PWM_OUT=PID.Servo_Position_Kp*Error+PID.Servo_Position_Ki*Integral_Error+PID.Servo_Position_Kd*(Error-Error_Last);
	//5.������һ�����
	Error_Last=Error;	
	//6.���ؿ��������ֵ
	return PWM_OUT;
}

void Position_PID_Servo_Realize(void)
{		
		float Position_PID_Angle_OUT=0;
		//���Ŀ��ֵ��ֵ
//		Param.Servo_Target_Angle=Target_Angle; 
		Position_PID_Angle_OUT=Position_PID_Servo(Param.Servo_Target_Angle,Param.Actual_Angle);
		Param.Position_PID_PWM_OUT=1850+Position_PID_Angle_OUT*6/5;			
}

/**
 * ������:VelocityRing_PID_MOTOR1_Realize
 * ����:���1�ٶȻ�PIDʵ��
 * ����:Velocity_Actual_Val:ʵ��ֵ
 * ���:PWM_OUT:�ٶȻ�PID���ֵ
 */
float VelocityRing_PID_MOTOR1_Realize(float Velocity_Actual_Val)
{
	static float  Error,Error_Last,Integral_Error,PWM_OUT;
    //1.����ƫ��
	Error=PID.Motor1_Velocity_Target_Val-Velocity_Actual_Val;
	
	//2.������е���
	if((Error<0.5f)&&(Error>-0.5f))//ʵ������1���ӣ��۲�Ŀ����ʵ��λ�ò�Ϊ1�����ӵ��ܳ� 
	{
	 Error=0.0f;
	}
	
	//3.�ۼ�ƫ��
	Integral_Error+=Error;
	
	//4.�����޷� (�޷�ֵ��ȷ��Ϊ��PID.Velocity_Ki*�޷�ֵ����PWM������ֵ)
	Integral_Error=(Integral_Error>690)?Integral_Error=690:((Integral_Error<-690)?Integral_Error=-690:Integral_Error);
	
	//5.PID�㷨ʵ��
	PWM_OUT=PID.Motor1_Velocity_Kp*Error+PID.Motor1_Velocity_Ki*Integral_Error+PID.Motor1_Velocity_Kd*(Error-Error_Last);

  //6.������һ�����
	Error_Last=Error;

	//7.����λ�û���ǰ���ֵ
	return PWM_OUT;
}

/**
 * ������:VelocityRing_PID_MOTOR2_Realize
 * ����:���2�ٶȻ�PIDʵ��
 * ����:Velocity_Actual_Val:ʵ��ֵ
 * ���:PWM_OUT:�ٶȻ�PID���ֵ
 */
float VelocityRing_PID_MOTOR2_Realize(float Velocity_Actual_Val)
{
	static float  Error,Error_Last,Integral_Error,PWM_OUT;
  //1.����ƫ��
	Error=PID.Motor2_Velocity_Target_Val-Velocity_Actual_Val;
	
	//2.������е���
	if((Error<0.5f)&&(Error>-0.5f))//ʵ������1���ӣ��۲�Ŀ����ʵ��λ�ò�Ϊ1�����ӵ��ܳ� 
	{
	 Error=0.0f;
	}
	
	//3.�ۼ�ƫ��
	Integral_Error+=Error;
	
	//4.�����޷� (�޷�ֵ��ȷ��Ϊ��PID.Velocity_Ki*�޷�ֵ����PWM������ֵ)
	Integral_Error=(Integral_Error>690)?Integral_Error=690:((Integral_Error<-690)?Integral_Error=-690:Integral_Error);
	
	//5.PID�㷨ʵ��
	PWM_OUT=PID.Motor2_Velocity_Kp*Error+PID.Motor2_Velocity_Ki*Integral_Error+PID.Motor2_Velocity_Kd*(Error-Error_Last);

  //6.������һ�����
	Error_Last=Error;

	//7.����λ�û���ǰ���ֵ
	return PWM_OUT;
}

/**
 * ������:VelocityRing_MOTOR1_Control
 * ����:�ٶȻ����1���
 * ����:��
 * ���:�������PWM
 */

float VelocityRing_MOTOR1_Control(void)
{
    
	float ExpectPWM = 0.0;//��ǰ����ֵ
//	  PID.Motor1_Velocity_Actual_Val=((float)Param.UnitTime_Motor1Pluse*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*VELOCITY_PID_PERIOD);/*      	
//    ʵ���ٶ�=����λʱ����������/�ֱܷ��ʣ�13*4��*���ٱȣ�30��*��ʱ�����ڣ�10ms�� ��*[��λ���� ]
	PID.Motor1_Velocity_Actual_Val=((float)Param.UnitTime_Motor1Pluse*60000.0)/15600;	
    ExpectPWM=VelocityRing_PID_MOTOR1_Realize(PID.Motor1_Velocity_Actual_Val);

	  return ExpectPWM;
}

/**
 * ������:VelocityRing_MOTOR2_Control
 * ����:�ٶȻ����2���
 * ����:��
 * ���:�������PWM
 */
float VelocityRing_MOTOR2_Control(void)
{
    float ExpectPWM = 0.0;//��ǰ����ֵ
	
	  PID.Motor2_Velocity_Actual_Val=((float)Param.UnitTime_Motor2Pluse*60000.0)/15600;
    ExpectPWM=VelocityRing_PID_MOTOR2_Realize(PID.Motor2_Velocity_Actual_Val);
	
	  return ExpectPWM;
}





