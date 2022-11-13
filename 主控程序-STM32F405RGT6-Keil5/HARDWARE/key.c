#include "key.h"
#include "delay.h" 
#include "led.h"
#include "usart.h"	

//KEY1 PC9
//KEY2 PC8

//按键初始化函数
void KEY_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// 按键中断配置
void KEYEXTI_Init(void)
{
	KEY_Init(); //按键对应的IO口初始化
	//中断初始化
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	
    /* 配置EXTI_Line8 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;//LINE8
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //上沿出发
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE0
    EXTI_Init(&EXTI_InitStructure);//配置
	
	/* 配置EXTI_Line9 */
	EXTI_InitStructure.EXTI_Line =  EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//中断线使能
	EXTI_Init(&EXTI_InitStructure);//配置
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//外部中断8 9
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//子优先级2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//配置
}

u8 KEY_Scan(void)
{	 		  
	if(KEY1==0||KEY2==0)
	{
		delay_ms(100);//去抖动
		if(KEY1==0)return 1;
		else if(KEY2==0)return 2;
	}
 	return 0;// 无按键按下
}
