#include "VisionUSART.h"
#include "math.h"
#include "led.h"
#include "dma.h"

#if SYSTEM_SUPPORT_OS
	#include "includes.h"
	#include "stm32f4xx_it.h"
#endif

u8 VISION_RX_BUF[VISION_REC_LEN];
//����5 �Ӿ����մ��ڳ�ʼ��
void VisionUSART_Init(int bound)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);//ʹ��UART5��GPIOAʱ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�����������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//DMA��ʼ������(����5RX����)
	MYDMA_Config(DMA1_Stream0,DMA_Channel_4,(uint32_t)(&(UART5->DR)),(uint32_t)VISION_RX_BUF,VISION_REC_LEN,1,0);	
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART5, &USART_InitStructure); 		//��ʼ������5
	//UART5 NVIC ����
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);//�رմ��ڽ����жϣ�ʹ��DMA+�����жϷ�ʽ���ܿ������жϣ�����DMA�Զ�������գ�				
	USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
	USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);//��������5DMAʹ�ܣ�������������˵���ú�DMA�Ǳߴ�����߾��Զ������ˣ�
	USART_Cmd(UART5, ENABLE);                    //ʹ�ܴ���5 
}

//�Ӿ��崮���ж�(������dma�Ӵ�����洢�����䣬��������ȴ������ڿ����жϣ�Ȼ����ܴ���dma����жϣ�
void UART5_IRQHandler(void)
{
	OSIntEnter();
	//���̻�ȡ���ݳ���
	u16 rx_len = DMA_GetDataLenth(DMA1_Stream0,VISION_REC_LEN);
	if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)  //���ڿ����ж�(������������ֱ������ͣ���Ż�������ʺϳ�����������)
	{
		//������ڿ����жϱ�־λ��ע���ȡ˳���ܱ䣩
		UART5->SR;UART5->DR;	//�����ȡSR DR�����Ա�����������жϡ���־��ֻ������Ӳ����ʽ������ã�USART_ClearITPendingBit(UART5,USART_IT_IDLE)����������С�
		//�������ݴ���
		Vision_Data_Process(rx_len);	//�ж��е����ݴ���ȥ�ı���Ӧ���Ʊ�����ֵ
		//��������DMA��0��DMA1_Stream0��Ӧ��
		MYDMA_Enable(DMA1_Stream0,VISION_REC_LEN,0);
	}
	OSIntExit();
}
//�����ж��еļ�ʱ���ݴ�������
void Vision_Data_Process(u16 rx_len)
{
//----------------------------------------֡β�ж�--------------------------------------------------
	if(rx_len>2 && VISION_RX_BUF[rx_len-1]=='C' && VISION_RX_BUF[rx_len-2]=='R')
	{
			rx_len-=2;//ת������
			switch(VISION_RX_BUF[0])
			{
//				case '1':LED1=!LED1;
			}
		}
}
//DMA�жϻ�����ûɶ�ã���Ҫ�Ǵ����жϣ��������ϲ������ֻҪ�������ݴ��䲻���죩����������жϻ����Ǹ����裩
//�����ԣ�1ms������������ڷ��Ͳ��ᵼ��DMA������������ᵼ�´��ڿ����жϲ���������
void DMA1_Stream0_IRQHandler(void)
{
	OSIntEnter();
	DMA_Interrupt_Process(DMA1_Stream0,(uint32_t)VISION_RX_BUF,VISION_REC_LEN,0);//�����жϴ���
	OSIntExit();
}
