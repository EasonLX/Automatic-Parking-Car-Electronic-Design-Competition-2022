#include "motor.h"

/*�����ʼ������*/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//����ʱ��
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//��ʼ��GPIO--PB12��PB13��PB14��PB15Ϊ�������
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);	
}

/*�޷�����*/
void Limit(int *motoA,int *motoB )
{
	if(*motoA>=6500)*motoA=6500;
	if(*motoA<=-6500)*motoA=-6500;
	
	if(*motoB>=6500)*motoB=6500;
	if(*motoB<=-6500)*motoB=-6500;
	
	//�޷�
	Param.Position_PID_PWM_OUT=Param.Position_PID_PWM_OUT>=1900?1900:(Param.Position_PID_PWM_OUT<=1800?1800:Param.Position_PID_PWM_OUT);
}

/*����ֵ����*/
int abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}


/*��ֵ����*/
/*��ڲ�����PID������ɺ������PWMֵ*/
void Load(int motor1,int motor2,uint16_t Target_Position)//moto1=-200����ת200������
{
	if(motor1>0)   //�����ת
	{
		Ain12=1;  //�ߵ�ƽ
		Ain13=0;  //�͵�ƽ
	}
	else if(motor1<0)  //�����ת
	{
		Ain12=0;  //�͵�ƽ
		Ain13=1;  //�ߵ�ƽ
	}
	else              //���ֹͣ
	{
		Ain12=0;  //�͵�ƽ
		Ain13=0;  //�͵�ƽ
	}
	if(motor2>0)   //�����ת
	{
		Bin14=1;
		Bin15=0;
	}
	else if(motor2<0)  //�����ת
	{
		Bin14=0;
		Bin15=1;
	}
	else              //���ֹͣ
	{
		Bin14=0;
		Bin15=0;
	}
	
	//����PWM
	PWMA=abs(motor1);
	PWMB=abs(motor2);
	
//	//3.װ�ض��PWMֵ
//	TIM_SetCompare1(TIM3,Target_Position);
}

/**
 * ������:set_motor_enable
 * ����:ʹ�ܵ��
 * ����:��
 * ���:��
 */
void set_motor_enable(void)  
{
	TIM_Cmd(TIM1,ENABLE); //ʹ�ܶ�ʱ��1
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
}

/**
 * ������:set_motor_disable
 * ����:ʧ�ܵ��
 * ����:��
 * ���:��
 */
void set_motor_disable(void)
{
	TIM_Cmd(TIM1,DISABLE); //ʧ�ܶ�ʱ��1
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Disable);
}

/**************************************************************************
�������ܣ��������PWM����
��ڲ�������
����  ֵ����
**************************************************************************/
void Set_PWM(int motor1,int motor2)
{
	if(motor2>0)   //�����ת
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);   //�ߵ�ƽ
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);  //�͵�ƽ
	}
	else if(motor2<0)  //�����ת
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_13);   //�ߵ�ƽ
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);  //�͵�ƽ
	}
	else              //���ֹͣ
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);   //�͵�ƽ
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);  //�͵�ƽ
	}
	if(motor1>0)   //�����ת
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);   //�ߵ�ƽ
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);  //�͵�ƽ
	}
	else if(motor1<0)  //�����ת
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);   //�ߵ�ƽ
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);  //�͵�ƽ
	}
	else              //���ֹͣ
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);   //�͵�ƽ
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);  //�͵�ƽ
	}
	
	//װ�ض��PWMֵ
	TIM_SetCompare1(TIM3,Param.Position_PID_PWM_OUT);
	//����PWM
	PWMA=abs(motor1);
	PWMB=abs(motor2);
}








