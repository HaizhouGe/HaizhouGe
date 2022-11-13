/*
	上层电机的控制相关函数
*/
#include "BrushMotor.h"
#include "delay.h"

u8 CancelMotorControl=0;
u8 brake = 0;

/*电机PWM的PID控制函数，放在定时器（或任务）中进行*/
void MotorX_Control(_encoder *motor_encoder,_motorPID *motor_PID,TIM_TypeDef *EncoderTIMx,TIM_TypeDef *TIMx,u8 chx, GPIO_TypeDef *GPIOx_left,uint16_t GPIO_Pin_left,GPIO_TypeDef *GPIOx_right,uint16_t GPIO_Pin_right)
{
    /* 10ms内捕获的脉冲数计算 */
    motor_encoder->Capture_Count = TIM_GetCounter(EncoderTIMx) - 0x8000;
    /* 电机转轴转速 = 给定时间（10ms）内的编码器计数值/电机输出总分辨率/时间系数（时间系数用于转换单位为r/min）*/
    motor_PID->pid_speed.actual_val = (float)motor_encoder->Capture_Count/motor_encoder->PER_CYCLE_PULSES/0.01f*60.0f;//理论计算，对于所有的直流有刷电机，在本工程的配置下，10ms内不可能出现溢出的情况。
	/* 电机转轴位置 = motor_encoder->Capture_Count的不断累加*/
	motor_PID->pid_location.actual_val +=  motor_encoder->Capture_Count;
	/* 位置PID计算,将输出作为速度环目标值（注意与速度目标值区别）*/
	motor_PID->pid_speed.target_val = MotorLocation_pid_realize(motor_PID,motor_encoder);
	/* 速度PID计算，将输出作为PWM的值进行电机驱动 */
	motor_PID->pid_speed.output_val = MotorSpeed_pid_realize(motor_PID);
//	if(motor_PID->pid_speed.output_val>2000) motor_PID->pid_speed.output_val = 2000;
//	if(motor_PID->pid_speed.output_val<-2000) motor_PID->pid_speed.output_val = -2000;
//	motor_PID->pid_speed.output_val = motor_PID->pid_speed.target_val*motor_PID->MaxPWM/motor_PID->AimSpeed;//通用位置环输出值限定
	/* 电机方向及转速控制 */
//	if(!brake) //不强制刹车
//	{
	if(!CancelMotorControl)
	{
		if(motor_PID->pid_speed.output_val >= 0)
		{
			GPIO_ResetBits(GPIOx_left,GPIO_Pin_left);
			GPIO_SetBits(GPIOx_right,GPIO_Pin_right);
			switch(chx)
			{
				case 1:TIM_SetCompare1(TIMx,(int)motor_PID->pid_speed.output_val);break;//设置比较值
				case 2:TIM_SetCompare2(TIMx,(int)motor_PID->pid_speed.output_val);break;//设置比较值
				case 3:TIM_SetCompare3(TIMx,(int)motor_PID->pid_speed.output_val);break;//设置比较值
				case 4:TIM_SetCompare4(TIMx,(int)motor_PID->pid_speed.output_val);break;//设置比较值
			}
		}
		else if(motor_PID->pid_speed.output_val < 0)//速度输出小于0
		{
			GPIO_SetBits(GPIOx_left,GPIO_Pin_left);
			GPIO_ResetBits(GPIOx_right,GPIO_Pin_right);
			switch(chx)
			{
				case 1:TIM_SetCompare1(TIMx,-(int)motor_PID->pid_speed.output_val);break;//设置比较值
				case 2:TIM_SetCompare2(TIMx,-(int)motor_PID->pid_speed.output_val);break;//设置比较值
				case 3:TIM_SetCompare3(TIMx,-(int)motor_PID->pid_speed.output_val);break;//设置比较值
				case 4:TIM_SetCompare4(TIMx,-(int)motor_PID->pid_speed.output_val);break;//设置比较值
			}
		}
	}
	else//解除对电机的控制
	{
		switch(chx)
		{
			case 1:TIM_SetCompare1(TIMx,0);break;//设置比较值
			case 2:TIM_SetCompare2(TIMx,0);break;//设置比较值
			case 3:TIM_SetCompare3(TIMx,0);break;//设置比较值
			case 4:TIM_SetCompare4(TIMx,0);break;//设置比较值
		}
	}
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOx_left,GPIO_Pin_left);
//		GPIO_ResetBits(GPIOx_right,GPIO_Pin_right);
//	}
	/* 计数复位 */
	TIM_SetCounter(EncoderTIMx,0x8000);
}
//初始化逻辑GPIO输出
void BrushMotorLogicIOInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOA时钟
	//PA初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	//PB初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);GPIO_ResetBits(GPIOB,GPIO_Pin_1);GPIO_ResetBits(GPIOB,GPIO_Pin_2);
	//PC初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);GPIO_ResetBits(GPIOC,GPIO_Pin_1);GPIO_ResetBits(GPIOC,GPIO_Pin_4);GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);GPIO_ResetBits(GPIOC,GPIO_Pin_14);GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}
void SetMotorX_Speed_Position(_motorPID *motor_PID, float LocTarget,float SpdTarget)
{
	motor_PID->AimSpeed = SpdTarget;
	motor_PID->pid_location.target_val=LocTarget;
}
