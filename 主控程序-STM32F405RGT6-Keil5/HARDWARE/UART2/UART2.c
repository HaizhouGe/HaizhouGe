/*
	JY901S的通信用串口
*/
#include <stdio.h>
#include "sys.h"
#include "UART2.h"
#include "JY901.h"

//发送数据时用的缓存数组
static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
//
static unsigned char count=0; 
//CopeSerial2Data为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
extern void CopeSerial2Data(unsigned char ucData);
//PA 2/3
void Initial_UART2(unsigned long baudrate)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//复用推挽输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); 
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);    
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	
	USART_ClearFlag(USART2,USART_FLAG_TC);	
	USART_Cmd(USART2, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 8;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

//串口2中断，兼具接受JY901数据、向外发送数据功能。
void USART2_IRQHandler(void)
{
  //如果，，，则通过串口2发送数据
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {
    USART_SendData(USART2, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART2, USART_IT_TXE);//悬起
	//当发送次数等于count（接受次数）时，说明发送完全。
    if(TxCounter == count) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);//失能
  }
  
  //如果接受缓冲区非空，则进行数据处理。
  else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
	CopeSerial2Data((unsigned char)USART2->DR);//处理数据（将接受到的串口数据拷贝到对应的结构体中）
	JY901_Get_Angle();
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
  //
  USART_ClearITPendingBit(USART2,USART_IT_ORE);
}

//向外发送数据用的2个函数，这两个函数本身不发送数据，仅仅是将数据进行存储。count代表了数据的长度。
void UART2_Put_Char(unsigned char DataToSend)
{
  TxBuffer[count++] = DataToSend;  
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
}
void UART2_Put_String(unsigned char *Str)
{
	while(*Str)
	{
		if(*Str=='\r')UART2_Put_Char(0x0d);
		else if(*Str=='\n')UART2_Put_Char(0x0a);
		else UART2_Put_Char(*Str);
		Str++;
	}
}
