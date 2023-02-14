#ifndef  _CONTROL_H
#define  _CONTROL_H

#include "stm32f10x.h"
#include "sys.h" 

#define BACK_PACKING 			1
#define SIDE_PACKING 			2
#define BACK_SIDE_PACKING 		3

/* ����������ֱ��� */
#define ENCODER_RESOLUTION                     13
/* ������Ƶ֮����ֱܷ��� */
#define ENCODER_TOTAL_RESOLUTION             (ENCODER_RESOLUTION * 4)  /* 4��Ƶ����ֱܷ��� */
/* ���ٵ�����ٱ� */
#define REDUCTION_RATIO  30
/* ��ʱ������10ms */
#define VELOCITY_PID_PERIOD  10   

typedef struct{	
	float 		Servo_Target_Angle;//Ŀ��Ƕ�
	float       Actual_Angle;//ʵ�ʽǶ�
	float       Position_PID_PWM_OUT;//PWM���
	
	int   		Motor1_PWM_Out;//���1PWM���
	int  		Motor2_PWM_Out;//���2PWM���	
	uint8_t 	openMV_Data;//openMV����	
	uint16_t 	Timer_threshold_value;//��ʱ��ֵ
	uint8_t 	ModeChoose;//ģʽѡ��
	
	short UnitTime_Motor1Pluse;//��λʱ���ڵ��1������
	short UnitTime_Motor2Pluse;//��λʱ���ڵ��2������	
}Param_InitTypedef;


typedef struct{	
 uint8_t Is_Go_straight;//�Ƿ�ֱ��
 uint8_t Is_Stop_Car;//�Ƿ�ͣ��
 uint8_t Is_Turn_Car;//�Ƿ�ת��
 uint8_t Is_Timer_Up;//�Ƿ�ʱ��
 uint8_t Is_Start_Astern;//�Ƿ�ʼ��������
 uint8_t Run_Step;//���в���
 uint8_t Start_Count;//��ʼ��ʱ	
 uint8_t Is_Start_Turn_Straight;//ֱ�й����־
}Flag_InitTypedef;

extern Param_InitTypedef Param;
extern Flag_InitTypedef  Flag;
extern float Pitch,Roll,Yaw; 		//ŷ����

void Kinematic_Analysis(float Target_Angle,float Actual_Angle,float velocity1,float velocity2);
void Control_Proc(void);









#endif

