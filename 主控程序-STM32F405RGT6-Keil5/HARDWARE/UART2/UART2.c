/*
	JY901S��ͨ���ô���
*/
#include <stdio.h>
#include "sys.h"
#include "UART2.h"
#include "JY901.h"

//��������ʱ�õĻ�������
static unsigned char TxBuffer[256];
static unsigned char TxCounter=0;
//
static unsigned char count=0; 
//CopeSerial2DataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ�����������
extern void CopeSerial2Data(unsigned char ucData);
//PA 2/3
void Initial_UART2(unsigned long baudrate)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�����������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//��������
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

//����2�жϣ���߽���JY901���ݡ����ⷢ�����ݹ��ܡ�
void USART2_IRQHandler(void)
{
  //�����������ͨ������2��������
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {
    USART_SendData(USART2, TxBuffer[TxCounter++]); 
    USART_ClearITPendingBit(USART2, USART_IT_TXE);//����
	//�����ʹ�������count�����ܴ�����ʱ��˵��������ȫ��
    if(TxCounter == count) USART_ITConfig(USART2, USART_IT_TXE, DISABLE);//ʧ��
  }
  
  //������ܻ������ǿգ���������ݴ���
  else if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
	CopeSerial2Data((unsigned char)USART2->DR);//�������ݣ������ܵ��Ĵ������ݿ�������Ӧ�Ľṹ���У�
	JY901_Get_Angle();
	USART_ClearITPendingBit(USART2, USART_IT_RXNE);
  }
  //
  USART_ClearITPendingBit(USART2,USART_IT_ORE);
}

//���ⷢ�������õ�2�����������������������������ݣ������ǽ����ݽ��д洢��count���������ݵĳ��ȡ�
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
