#include "system_init.h"
/*定时器6和7功能少，主要用于计时。TIM6用于信号量控制(其中断写在stm_it文件中)，故PID控制使用TIM7(尽管可以与TIM6共用，但，既然有多余的那就用上呗)*/
void TIM6_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  ///使能TIM6时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);//初始化TIM6
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //允许定时器6更新中断
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	TIM_Cmd(TIM6,ENABLE); //使能定时器6
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn; //定时器6中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM7_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///使能TIM7时钟
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; //自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //允许定时器7更新中断
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_Cmd(TIM7,ENABLE); //使能定时器7
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //定时器7中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM7,TIM_IT_Update) == SET)
	{
		//控制5个编码器电机的PID，这个函数的配置决定了电机与定时器、IO口的确定关系，一定要对应好
		//电机号与编码器输入定时器号、PWM输出定时器号个位，一致（除了电机5）；同时通道均使用CH1
		MotorX_Control(&motor1_encoder,&Motor1PID,TIM1,TIM11,1,GPIOB,GPIO_Pin_0,GPIOB,GPIO_Pin_1);//编码器：PA8/9；PWM：PB9；方向：PB0/1。
		MotorX_Control(&motor2_encoder,&Motor2PID,TIM2,TIM12,1,GPIOC,GPIO_Pin_4,GPIOC,GPIO_Pin_5);//编码器：PA15/PB3；PWM：PB14；方向：PC4/5。
		MotorX_Control(&motor3_encoder,&Motor3PID,TIM3,TIM12,2,GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);//编码器：PB4/5；PWM：PB15；方向：PC13/PC14。（使用了M6接口控制）
		MotorX_Control(&motor4_encoder,&Motor4PID,TIM4,TIM14,1,GPIOC,GPIO_Pin_0,GPIOC,GPIO_Pin_1);//编码器：PB6/7；PWM：PA7；方向：PC0/1。
//		MotorX_Control(&motor5_encoder,&Motor5PID,TIM8,TIM10,1,GPIOA,GPIO_Pin_4,GPIOA,GPIO_Pin_5);//编码器：PC6/7；PWM：PB8；方向：PA4/5。
		//清中断标志位
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
		//注：TIM12的CH2用于控制无编码器直流有刷电机，可以利用PWM的占空比做到用24V驱动18V电机。
//		MotorX_Control(&motor6_encoder,&Motor6PID,TIM无,TIM13, 1, GPIOB,GPIO_Pin_2,GPIOC,GPIO_Pin_13);//编码器：无；PWM：PB15；方向：PB2/PC13。因绿深的PB2电压过低故实际很可能无法控制一些驱动模块正反转，只能单向转（但不排除其它对高电平要求较低模块可以）。
	}
	OSIntExit();
}
