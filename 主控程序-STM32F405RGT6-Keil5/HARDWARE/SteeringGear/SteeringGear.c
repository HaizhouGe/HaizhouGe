#include "SteeringGear.h"
#include "math.h"
#include "led.h"
#include "dma.h"

#if SYSTEM_SUPPORT_OS
	#include "includes.h"
	#include "stm32f4xx_it.h"
#endif

u8 SG_RX_BUF[SG_REC_LEN];
//串口4
void SGUSART_Init(int bound)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能UART5，GPIOA时钟
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//DMA初始化配置(串口4RX配置)
	MYDMA_Config(DMA1_Stream2,DMA_Channel_4,(uint32_t)(&(UART4->DR)),(uint32_t)SG_RX_BUF,SG_REC_LEN,1,0);
    //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); 		//初始化串口5
	//UART5 NVIC 配置
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//关闭串口接收中断（使用DMA+空闲中断方式不能开接收中断，接收DMA自动处理接收）				
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//开启串口空闲中断
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);//开启串口5DMA使能（容易忘，不是说配置好DMA那边串口这边就自动配置了）
	USART_Cmd(UART4, ENABLE);                    //使能串口5 
}

//舵机板串口中断(由于是dma从串口向存储器传输，因此总是先触发串口空闲中断，然后才能触发dma完成中断）
void UART4_IRQHandler(void)
{
	OSIntEnter();
	//立刻获取数据长度
	u16 rx_len = DMA_GetDataLenth(DMA1_Stream2,SG_REC_LEN);
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  //串口空闲中断(连续接收数据直到数据停流才会产生，适合长数据流分析)
	{
		//清除串口空闲中断标志位（注意读取顺序不能变）
		UART4->SR;UART4->DR;	//必须读取SR DR数据以便清除“空闲中断”标志，只有这种硬件方式清理可用，USART_ClearITPendingBit(UART5,USART_IT_IDLE)这个函数不行。
		//进行数据处理
		SG_Data_Process(rx_len);	//中断中的数据处理去改变相应控制变量的值
		//重新启动DMA（2与DMA1_Stream2对应）
		MYDMA_Enable(DMA1_Stream2,SG_REC_LEN,2);
	}
	OSIntExit();
}
//串口中断中的即时数据处理函数。
void SG_Data_Process(u16 rx_len)
{
//----------------------------------------帧尾判断--------------------------------------------------//
	if(rx_len>2 && SG_RX_BUF[rx_len-1]=='C' && SG_RX_BUF[rx_len-2]=='R')
	{
			rx_len-=2;//转换长度
			switch(SG_RX_BUF[0])
			{
//				case '1':LED1=!LED1;
			}
	}
}
//DMA中断基本上没啥用，主要是错误中断（而基本上不会出错（只要串口数据传输不过快），所以这个中断基本是个摆设）
//经测试，1ms间隔的连续串口发送不会导致DMA传输出错，即不会导致串口空闲中断不被触发。
void DMA1_Stream2_IRQHandler(void)
{
	OSIntEnter();
	DMA_Interrupt_Process(DMA1_Stream2,(uint32_t)SG_RX_BUF,SG_REC_LEN,2);//进行中断处理
	OSIntExit();
}
void SteeringGearSendCommand_180(u8 SG_ID, u16 time, u16 angle)
{
	angle = (int)(angle*2000/180 + 500);
	//帧头
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束	
	//数据长度
	USART_SendData(UART5, 0x08);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//指令
	USART_SendData(UART5, 0x03);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//舵机数
	USART_SendData(UART5, 0x01);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束	
	//时间（ms）
	USART_SendData(UART5, (uint8_t)time);//低八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束	
	USART_SendData(UART5, (uint8_t)(time>>8));//高八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//ID
	USART_SendData(UART5, SG_ID);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//角度（度）
	USART_SendData(UART5, (uint8_t)angle);//低八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(UART5, (uint8_t)(angle>>8));//高八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
}
void SteeringGearSendCommand_270(u8 SG_ID, u16 time, u16 angle)
{
	angle = (int)(angle*2000/270 + 500);
	//帧头
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束	
	//数据长度
	USART_SendData(UART5, 0x08);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//指令
	USART_SendData(UART5, 0x03);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//舵机数
	USART_SendData(UART5, 0x01);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束	
	//时间（ms）
	USART_SendData(UART5, (uint8_t)time);//低八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束	
	USART_SendData(UART5, (uint8_t)(time>>8));//高八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//ID
	USART_SendData(UART5, SG_ID);//向串口4发送数据
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	//角度（度）
	USART_SendData(UART5, (uint8_t)angle);//低八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
	USART_SendData(UART5, (uint8_t)(angle>>8));//高八位
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//等待发送结束
}
