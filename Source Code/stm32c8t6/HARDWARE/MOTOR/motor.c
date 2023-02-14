#include "motor.h"

/*电机初始化函数*/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//开启时钟
	
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;//初始化GPIO--PB12、PB13、PB14、PB15为推挽输出
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14 |GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);	
}

/*限幅函数*/
void Limit(int *motoA,int *motoB )
{
	if(*motoA>=6500)*motoA=6500;
	if(*motoA<=-6500)*motoA=-6500;
	
	if(*motoB>=6500)*motoB=6500;
	if(*motoB<=-6500)*motoB=-6500;
	
	//限幅
	Param.Position_PID_PWM_OUT=Param.Position_PID_PWM_OUT>=1900?1900:(Param.Position_PID_PWM_OUT<=1800?1800:Param.Position_PID_PWM_OUT);
}

/*绝对值函数*/
int abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}


/*赋值函数*/
/*入口参数：PID运算完成后的最终PWM值*/
void Load(int motor1,int motor2,uint16_t Target_Position)//moto1=-200：反转200个脉冲
{
	if(motor1>0)   //电机正转
	{
		Ain12=1;  //高电平
		Ain13=0;  //低电平
	}
	else if(motor1<0)  //电机反转
	{
		Ain12=0;  //低电平
		Ain13=1;  //高电平
	}
	else              //电机停止
	{
		Ain12=0;  //低电平
		Ain13=0;  //低电平
	}
	if(motor2>0)   //电机正转
	{
		Bin14=1;
		Bin15=0;
	}
	else if(motor2<0)  //电机反转
	{
		Bin14=0;
		Bin15=1;
	}
	else              //电机停止
	{
		Bin14=0;
		Bin15=0;
	}
	
	//设置PWM
	PWMA=abs(motor1);
	PWMB=abs(motor2);
	
//	//3.装载舵机PWM值
//	TIM_SetCompare1(TIM3,Target_Position);
}

/**
 * 函数名:set_motor_enable
 * 描述:使能电机
 * 输入:无
 * 输出:无
 */
void set_motor_enable(void)  
{
	TIM_Cmd(TIM1,ENABLE); //使能定时器1
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Enable);
}

/**
 * 函数名:set_motor_disable
 * 描述:失能电机
 * 输入:无
 * 输出:无
 */
void set_motor_disable(void)
{
	TIM_Cmd(TIM1,DISABLE); //失能定时器1
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_4, TIM_CCx_Disable);
}

/**************************************************************************
函数功能：最后设置PWM函数
入口参数：无
返回  值：无
**************************************************************************/
void Set_PWM(int motor1,int motor2)
{
	if(motor2>0)   //电机正转
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_12);   //高电平
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);  //低电平
	}
	else if(motor2<0)  //电机反转
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_13);   //高电平
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);  //低电平
	}
	else              //电机停止
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);   //低电平
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);  //低电平
	}
	if(motor1>0)   //电机正转
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_14);   //高电平
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);  //低电平
	}
	else if(motor1<0)  //电机反转
	{
		GPIO_SetBits(GPIOB,GPIO_Pin_15);   //高电平
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);  //低电平
	}
	else              //电机停止
	{
		GPIO_ResetBits(GPIOB,GPIO_Pin_14);   //低电平
		GPIO_ResetBits(GPIOB,GPIO_Pin_15);  //低电平
	}
	
	//装载舵机PWM值
	TIM_SetCompare1(TIM3,Param.Position_PID_PWM_OUT);
	//设置PWM
	PWMA=abs(motor1);
	PWMB=abs(motor2);
}








