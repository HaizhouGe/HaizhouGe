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
//����3 Զ��ң�ؽ��մ��ڳ�ʼ��
void RemoteUSART_Init(int bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�����������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//DMA��ʼ������(����3 RX����)
	MYDMA_Config(DMA1_Stream1,DMA_Channel_4,(uint32_t)(&(USART3->DR)),(uint32_t)Remote_RX_BUF,Remote_REC_LEN,1,0);	
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART3, &USART_InitStructure); 		//��ʼ������5
	//UART2 NVIC ����
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);//�رմ��ڽ����жϣ�ʹ��DMA+�����жϷ�ʽ���ܿ������жϣ�����DMA�Զ�������գ�				
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);//��������5DMAʹ�ܣ�������������˵���ú�DMA�Ǳߴ�����߾��Զ������ˣ�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���5 
}

//����3�ж�
void USART3_IRQHandler(void)
{
	OSIntEnter();
	//���̻�ȡ���ݳ���
	u16 rx_len = DMA_GetDataLenth(DMA1_Stream1,Remote_REC_LEN);
	if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)  //���ڿ����ж�(������������ֱ������ͣ���Ż�������ʺϳ�����������)
	{
		//������ڿ����жϱ�־λ��ע���ȡ˳���ܱ䣩
		USART3->SR;USART3->DR;//�����ȡSR DR�����Ա�����������жϡ���־��ֻ������Ӳ����ʽ������ã�USART_ClearITPendingBit(USART2,USART_IT_IDLE)����������С�
		//�������ݴ���
		Remote_Data_Process(rx_len);//�ж��е����ݴ���ȥ�ı���Ӧ���Ʊ�����ֵ
		//��������DMA��1��DMA1_Stream1��Ӧ��
		MYDMA_Enable(DMA1_Stream1,Remote_REC_LEN,1);
	}
	USART_ClearFlag(USART3,USART_IT_IDLE);
	USART_ClearITPendingBit(USART3,USART_IT_IDLE);
	OSIntExit(); //ucos�˳��жϺ���
}
//�����ж��еļ�ʱ���ݴ�������
void Remote_Data_Process(u16 rx_len)
{
	if(rx_len>1)//��������
	{
//----------------------------------------֡β�ж�--------------------------------------------------
		switch(Remote_RX_BUF[rx_len-1])
		{
			case '#':SG_Command =  Remote_RX_BUF[0];//�����������
			case '@':BM_Command =  Remote_RX_BUF[0];//��ˢ�����������
			case '$':AutoPerform = Remote_RX_BUF[0];//�Զ�ȫ������
		}
	}
	else if (rx_len==1)//�ٶȿ���
	{
		MoveSpeed = Remote_RX_BUF[0];
	}
}
//DMA�жϻ�����ûɶ�ã���Ҫ�Ǵ����жϣ��������ϲ������ֻҪ�������ݴ��䲻���죩����������жϻ����Ǹ����裩
//�����ԣ�1ms������������ڷ��Ͳ��ᵼ��DMA������������ᵼ�´��ڿ����жϲ�������
void DMA1_Stream1_IRQHandler(void)
{
	OSIntEnter();
	DMA_Interrupt_Process(DMA1_Stream1,(uint32_t)Remote_RX_BUF,Remote_REC_LEN,1);//�����жϴ���
	OSIntExit();
}
