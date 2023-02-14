#include "pid.h"
#include "control.h"
PID_InitTypedef PID;

/**
 * 函数名:PID_Para_Init
 * 描述:PID参数初始化
 * 输入:无
 * 输出:无
 */
 void PID_Param_Init(void)
 {
	 /*位置式PID-舵机*/
	 PID.Servo_Position_Kp=7.5;
	 PID.Servo_Position_Ki=0.0;
	 PID.Servo_Position_Kd=0.01;
	 /*位置式PID-电机1*/
	 PID.Motor1_Velocity_Kp=10.0; 
	 PID.Motor1_Velocity_Ki=6.0;
	 PID.Motor1_Velocity_Kd=0.0;
	 /*位置式PID-电机2*/
	 PID.Motor2_Velocity_Kp=10.0; 
	 PID.Motor2_Velocity_Ki=6.0;
	 PID.Motor2_Velocity_Kd=0.0;
 }
/**
 * 函数名:Position_PID_Servo
 * 描述:舵机位置式PID算法实现
 * 输入:Target_Val:目标角度
 * 输出:PID.Location_Out:位置环PID输出值
 */
float Position_PID_Servo(float Target_Angle,float yaw)
{
	static float Error,Integral_Error,Error_Last,Actual_Angle,PWM_OUT;
	Actual_Angle=yaw;
	//2.计算偏差
	Error=Target_Angle-Actual_Angle;
	//偏差滤波
//	if(Error>(-0.3)&&Error<0.3)
//	{
//		Error=Error_Last;
//	}
	//3.累计偏差
	Integral_Error+=Error;
	//4.PID算法
	PWM_OUT=PID.Servo_Position_Kp*Error+PID.Servo_Position_Ki*Integral_Error+PID.Servo_Position_Kd*(Error-Error_Last);
	//5.更新上一次误差
	Error_Last=Error;	
	//6.返回控制量输出值
	return PWM_OUT;
}

void Position_PID_Servo_Realize(void)
{		
		float Position_PID_Angle_OUT=0;
		//舵机目标值赋值
//		Param.Servo_Target_Angle=Target_Angle; 
		Position_PID_Angle_OUT=Position_PID_Servo(Param.Servo_Target_Angle,Param.Actual_Angle);
		Param.Position_PID_PWM_OUT=1850+Position_PID_Angle_OUT*6/5;			
}

/**
 * 函数名:VelocityRing_PID_MOTOR1_Realize
 * 描述:电机1速度环PID实现
 * 输入:Velocity_Actual_Val:实际值
 * 输出:PWM_OUT:速度环PID输出值
 */
float VelocityRing_PID_MOTOR1_Realize(float Velocity_Actual_Val)
{
	static float  Error,Error_Last,Integral_Error,PWM_OUT;
    //1.计算偏差
	Error=PID.Motor1_Velocity_Target_Val-Velocity_Actual_Val;
	
	//2.消除机械误差
	if((Error<0.5f)&&(Error>-0.5f))//实际运行1分钟，观察目标与实际位置差为1个轮子的周长 
	{
	 Error=0.0f;
	}
	
	//3.累计偏差
	Integral_Error+=Error;
	
	//4.积分限幅 (限幅值的确定为：PID.Velocity_Ki*限幅值等于PWM输出最大值)
	Integral_Error=(Integral_Error>690)?Integral_Error=690:((Integral_Error<-690)?Integral_Error=-690:Integral_Error);
	
	//5.PID算法实现
	PWM_OUT=PID.Motor1_Velocity_Kp*Error+PID.Motor1_Velocity_Ki*Integral_Error+PID.Motor1_Velocity_Kd*(Error-Error_Last);

  //6.更新上一次误差
	Error_Last=Error;

	//7.返回位置环当前输出值
	return PWM_OUT;
}

/**
 * 函数名:VelocityRing_PID_MOTOR2_Realize
 * 描述:电机2速度环PID实现
 * 输入:Velocity_Actual_Val:实际值
 * 输出:PWM_OUT:速度环PID输出值
 */
float VelocityRing_PID_MOTOR2_Realize(float Velocity_Actual_Val)
{
	static float  Error,Error_Last,Integral_Error,PWM_OUT;
  //1.计算偏差
	Error=PID.Motor2_Velocity_Target_Val-Velocity_Actual_Val;
	
	//2.消除机械误差
	if((Error<0.5f)&&(Error>-0.5f))//实际运行1分钟，观察目标与实际位置差为1个轮子的周长 
	{
	 Error=0.0f;
	}
	
	//3.累计偏差
	Integral_Error+=Error;
	
	//4.积分限幅 (限幅值的确定为：PID.Velocity_Ki*限幅值等于PWM输出最大值)
	Integral_Error=(Integral_Error>690)?Integral_Error=690:((Integral_Error<-690)?Integral_Error=-690:Integral_Error);
	
	//5.PID算法实现
	PWM_OUT=PID.Motor2_Velocity_Kp*Error+PID.Motor2_Velocity_Ki*Integral_Error+PID.Motor2_Velocity_Kd*(Error-Error_Last);

  //6.更新上一次误差
	Error_Last=Error;

	//7.返回位置环当前输出值
	return PWM_OUT;
}

/**
 * 函数名:VelocityRing_MOTOR1_Control
 * 描述:速度环电机1输出
 * 输入:无
 * 输出:输出期望PWM
 */

float VelocityRing_MOTOR1_Control(void)
{
    
	float ExpectPWM = 0.0;//当前控制值
//	  PID.Motor1_Velocity_Actual_Val=((float)Param.UnitTime_Motor1Pluse*1000.0*60.0)/(ENCODER_TOTAL_RESOLUTION*REDUCTION_RATIO*VELOCITY_PID_PERIOD);/*      	
//    实际速度=（单位时间电机脉冲数/总分辨率（13*4）*减速比（30）*定时器周期（10ms） ）*[单位换算 ]
	PID.Motor1_Velocity_Actual_Val=((float)Param.UnitTime_Motor1Pluse*60000.0)/15600;	
    ExpectPWM=VelocityRing_PID_MOTOR1_Realize(PID.Motor1_Velocity_Actual_Val);

	  return ExpectPWM;
}

/**
 * 函数名:VelocityRing_MOTOR2_Control
 * 描述:速度环电机2输出
 * 输入:无
 * 输出:输出期望PWM
 */
float VelocityRing_MOTOR2_Control(void)
{
    float ExpectPWM = 0.0;//当前控制值
	
	  PID.Motor2_Velocity_Actual_Val=((float)Param.UnitTime_Motor2Pluse*60000.0)/15600;
    ExpectPWM=VelocityRing_PID_MOTOR2_Realize(PID.Motor2_Velocity_Actual_Val);
	
	  return ExpectPWM;
}





