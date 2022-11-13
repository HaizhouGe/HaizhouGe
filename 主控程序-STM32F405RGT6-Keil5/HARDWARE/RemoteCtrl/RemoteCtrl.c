#include "RemoteCtrl.h"
#include "math.h"
#include "led.h"
#include "dma.h"
#include "SteeringGear.h"

#if SYSTEM_SUPPORT_OS
	#include "includes.h"
	#include "stm32f4xx_it.h"
#endif

u8 BM_Command = 0;
u8 SG_Command = 0;
u8 AutoPerform = 0;
u8 MoveSpeed=0;
u8 Remote_RX_BUF[Remote_REC_LEN];
//串口3 远程遥控接收串口初始化
void RemoteUSART_Init(int bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//DMA初始化配置(串口3 RX配置)
	MYDMA_Config(DMA1_Stream1,DMA_Channel_4,(uint32_t)(&(USART3->DR)),(uint32_t)Remote_RX_BUF,Remote_REC_LEN,1,0);	
    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART3, &USART_InitStructure); 		//初始化串口5
	//UART2 NVIC 配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//关闭串口接收中断（使用DMA+空闲中断方式不能开接收中断，接收DMA自动处理接收）				
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//开启串口5DMA使能（容易忘，不是说配置好DMA那边串口这边就自动配置了）
	USART_Cmd(USART3, ENABLE);                    //使能串口5 
}

//串口3中断
void USART3_IRQHandler(void)
{
	OSIntEnter();
	//立刻获取数据长度
	u16 rx_len = DMA_GetDataLenth(DMA1_Stream1,Remote_REC_LEN);
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //串口空闲中断(连续接收数据直到数据停流才会产生，适合长数据流分析)
	{
		//清除串口空闲中断标志位（注意读取顺序不能变）
		USART3->SR;USART3->DR;//必须读取SR DR数据以便清除“空闲中断”标志，只有这种硬件方式清理可用，USART_ClearITPendingBit(USART2,USART_IT_IDLE)这个函数不行。
		//进行数据处理
		Remote_Data_Process(rx_len);//中断中的数据处理去改变相应控制变量的值
		//重新启动DMA（1与DMA1_Stream1对应）
		MYDMA_Enable(DMA1_Stream1,Remote_REC_LEN,1);
	}
	USART_ClearFlag(USART3,USART_IT_IDLE);
	USART_ClearITPendingBit(USART3,USART_IT_IDLE);
	OSIntExit(); //ucos退出中断函数
}
//串口中断中的即时数据处理函数。
void Remote_Data_Process(u16 rx_len)
{
	if(rx_len>1)//动作控制
	{
//----------------------------------------帧尾判断--------------------------------------------------
		switch(Remote_RX_BUF[rx_len-1])
		{
			case '#':SG_Command =  Remote_RX_BUF[0];//舵机控制命令
			case '@':BM_Command =  Remote_RX_BUF[0];//有刷电机控制命令
			case '$':AutoPerform = Remote_RX_BUF[0];//自动全套命令
		}
	}
	else if (rx_len==1)//速度控制
	{
		MoveSpeed = Remote_RX_BUF[0];
	}
}
//DMA中断基本上没啥用，主要是错误中断（而基本上不会出错（只要串口数据传输不过快），所以这个中断基本是个摆设）
//经测试，1ms间隔的连续串口发送不会导致DMA传输出错，即不会导致串口空闲中断不被触发
void DMA1_Stream1_IRQHandler(void)
{
	OSIntEnter();
	DMA_Interrupt_Process(DMA1_Stream1,(uint32_t)Remote_RX_BUF,Remote_REC_LEN,1);//进行中断处理
	OSIntExit();
}
