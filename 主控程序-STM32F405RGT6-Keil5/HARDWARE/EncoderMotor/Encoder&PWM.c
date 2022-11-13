/*
	PWM与编码器的TIM配置
*/
#include "includes.h"
#include "Encoder&PWM.h" 
#include "led.h"
#include "BrushPID.h"
#include "timer.h"
#include "system_init.h"
//编码器参数初始化
_encoder motor1_encoder={0,3692};//4*13*71=3692
_encoder motor2_encoder={0,3692};
_encoder motor3_encoder={0,3692};

_encoder motor4_encoder={0,1560};//4*13*30=1560
_encoder motor5_encoder={0,1560};

/*定时器的编码器模式及中断配置*/
void TIM1_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);//使能TIM1时钟 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOB时钟

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM4时钟预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM1,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;//通道选择
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter = 0;//选择输入比较滤波器
	TIM_ICInit(TIM1, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM4

	TIM_SetCounter(TIM1,0x8000); //TIM1->CNT=0
	TIM_Cmd(TIM1, ENABLE); 
}
/*
TIM2(1个编码器)
CH1 PA15(JTDI)
CH2 PB3(JTDO)
*/
void TIM2_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;   

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//使能TIM2时钟  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOA时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM2时钟预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);//使用编码器模式2，上升下降都计数
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;//通道选择
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter = 0;//滤波系数，0表示不滤波
	TIM_ICInit(TIM2, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
    TIM_ICInit(TIM2, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM2

	TIM_SetCounter(TIM2,0x8000);
	TIM_Cmd(TIM2, ENABLE); 
}
/*
TIM3(1个编码器)
CH1 PB4(NJTRST)
CH2 PB5
*/
void TIM3_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;   

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//使能TIM3时钟  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM3时钟预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;//通道选择
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter = 0;//选择输入比较滤波器
	TIM_ICInit(TIM3, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM3

	TIM_SetCounter(TIM3,0x8000);
	TIM_Cmd(TIM3, ENABLE); 
}
/*
TIM4（1个编码器）
CH1 PB6
CH2 PB7
*/
void TIM4_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//使能TIM4时钟  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_TIM4);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;//设定计数器重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM4时钟预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//TIM向上计数
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM4,TIM_EncoderMode_TI12, TIM_ICPolarity_BothEdge, TIM_ICPolarity_BothEdge);//使用编码器模式3，上升下降都计数

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;//通道选择
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter = 0;//选择输入比较滤波器
	TIM_ICInit(TIM4, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM4

	TIM_SetCounter(TIM4,0x8000);//取中间值
	TIM_Cmd(TIM4, ENABLE);
}
/*
TIM8（1个编码器）
CHC PB6
CHC PB7
*/
void TIM8_Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;   
	//管脚配置
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);//使能TIM8时钟  
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//使能GPIOC时钟
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD;  //设定计数器重装值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //TIM8时钟预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision =TIM_CKD_DIV1 ;//设置时钟分割 T_dts = T_ck_int
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数 
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);//使用编码器模式2，上升下降都计数

	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1;//通道选择
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICFilter = 0;//选择输入比较滤波器
	TIM_ICInit(TIM8, &TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel     = TIM_Channel_2;
    TIM_ICInit(TIM8, &TIM_ICInitStructure);//将TIM_ICInitStructure中的指定参数初始化TIM8

	TIM_SetCounter(TIM8,0x8000);
	TIM_Cmd(TIM8, ENABLE);
}
/***********PWM控制配置**********/
void TIM10_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_TIM10);//GPIOB10复用为定时器12CH1
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_8;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC1Init(TIM10,&TIM_OCInitStruct);//通道1
	
	TIM_OC1PreloadConfig(TIM10,TIM_OCPreload_Enable);//使能预装载寄存器
	
  	TIM_Cmd(TIM10,ENABLE);  //使能TIM2
}

void TIM11_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_TIM11);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//110MHz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM11, &TIM_TimeBaseInitStruct);

	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC1Init(TIM11,&TIM_OCInitStruct);//通道1
	
	TIM_OC1PreloadConfig(TIM11,TIM_OCPreload_Enable);        //使能预装载寄存器
	
  	TIM_Cmd(TIM11,ENABLE);  //使能TIM2
}

void TIM12_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_TIM12);//GPIOB14复用为定时器12CH1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_TIM12);//GPIOB15复用为定时器12CH2
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//140MHz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_14;//ch1
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_15;//ch2
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC1Init(TIM12,&TIM_OCInitStruct);//通道1
	TIM_OC2Init(TIM12,&TIM_OCInitStruct);//通道2
	
	TIM_OC1PreloadConfig(TIM12,TIM_OCPreload_Enable);//使能通道1预装载寄存器
	TIM_OC2PreloadConfig(TIM12,TIM_OCPreload_Enable);//使能通道2预装载寄存器
  	TIM_Cmd(TIM12,ENABLE);//使能TIM2
}

void TIM13_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM13);//GPIOB13复用为定时器12CH1
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//130MHz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM13, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC1Init(TIM13,&TIM_OCInitStruct);//通道1
	
	TIM_OC1PreloadConfig(TIM13,TIM_OCPreload_Enable);        //使能预装载寄存器
	
  	TIM_Cmd(TIM13,ENABLE);
}
//PA7进行
void TIM14_PWM_Init(u16 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//使能GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM14);
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;//复用模式
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;//140MHz
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//分频因子
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStruct.TIM_Period=arr;//自动重装载值
	TIM_TimeBaseInitStruct.TIM_Prescaler=psc;//预分频系数
	TIM_TimeBaseInit(TIM14, &TIM_TimeBaseInitStruct);
	
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OC1Init(TIM14,&TIM_OCInitStruct);//通道1
	
	TIM_OC1PreloadConfig(TIM14,TIM_OCPreload_Enable);//使能预装载寄存器
	
  	TIM_Cmd(TIM14,ENABLE);
}
