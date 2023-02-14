#ifndef __PID_H 
#define	__PID_H

#include "stm32f10x.h"

typedef struct
{
	float Servo_Position_Kp;//���λ�û�Kp
	float Servo_Position_Ki;//���λ�û�Ki
	float Servo_Position_Kd;//���λ�û�Kd
	
	float Motor1_Velocity_Kp;//�ٶȻ�Kp
	float Motor1_Velocity_Ki;//�ٶȻ�Ki
	float Motor1_Velocity_Kd;//�ٶȻ�Kd
	float Motor1_Velocity_Target_Val;//����ֵ
	float Motor1_Velocity_Actual_Val;//ʵ��ֵ
	int Motor1_Velocity_Out;//�ٶȻ��������
	
	float Motor2_Velocity_Kp;//�ٶȻ�Kp
	float Motor2_Velocity_Ki;//�ٶȻ�Ki
	float Motor2_Velocity_Kd;//�ٶȻ�Kd
	float Motor2_Velocity_Target_Val;//����ֵ
	float Motor2_Velocity_Actual_Val;//ʵ��ֵ
	int Motor2_Velocity_Out;//�ٶȻ��������
}PID_InitTypedef;

extern PID_InitTypedef PID;
extern void   PID_Param_Init(void);
extern float Position_PID_Servo(float Target_Angle,float yaw);
extern float  VelocityRing_PID_MOTOR1_Realize(float Velocity_Actual_Val);
extern float  VelocityRing_PID_MOTOR2_Realize(float Velocity_Actual_Val);
extern float  VelocityRing_MOTOR1_Control(void);
extern float  VelocityRing_MOTOR2_Control(void);

extern void set_p_i_d(PID_InitTypedef *PID, float Val_Kp, float Val_Ki, float Val_Kd);
extern float get_pid_target(void);
extern void set_pid_target(PID_InitTypedef *PID,float val_temp);
extern void Position_PID_Servo_Realize(void);















#endif
