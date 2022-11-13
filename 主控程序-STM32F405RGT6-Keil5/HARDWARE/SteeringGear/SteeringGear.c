#include "SteeringGear.h"
#include "math.h"
#include "led.h"
#include "dma.h"

#if SYSTEM_SUPPORT_OS
	#include "includes.h"
	#include "stm32f4xx_it.h"
#endif

u8 SG_RX_BUF[SG_REC_LEN];
//����4
void SGUSART_Init(int bound)
{
    //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��UART5��GPIOAʱ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_UART4); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //TX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	//�����������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;//RX
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//DMA��ʼ������(����4RX����)
	MYDMA_Config(DMA1_Stream2,DMA_Channel_4,(uint32_t)(&(UART4->DR)),(uint32_t)SG_RX_BUF,SG_REC_LEN,1,0);
    //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure); 		//��ʼ������5
	//UART5 NVIC ����
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
	USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);//�رմ��ڽ����жϣ�ʹ��DMA+�����жϷ�ʽ���ܿ������жϣ�����DMA�Զ�������գ�				
	USART_ITConfig(UART4, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
	USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);//��������5DMAʹ�ܣ�������������˵���ú�DMA�Ǳߴ�����߾��Զ������ˣ�
	USART_Cmd(UART4, ENABLE);                    //ʹ�ܴ���5 
}

//����崮���ж�(������dma�Ӵ�����洢�����䣬��������ȴ������ڿ����жϣ�Ȼ����ܴ���dma����жϣ�
void UART4_IRQHandler(void)
{
	OSIntEnter();
	//���̻�ȡ���ݳ���
	u16 rx_len = DMA_GetDataLenth(DMA1_Stream2,SG_REC_LEN);
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)  //���ڿ����ж�(������������ֱ������ͣ���Ż�������ʺϳ�����������)
	{
		//������ڿ����жϱ�־λ��ע���ȡ˳���ܱ䣩
		UART4->SR;UART4->DR;	//�����ȡSR DR�����Ա�����������жϡ���־��ֻ������Ӳ����ʽ������ã�USART_ClearITPendingBit(UART5,USART_IT_IDLE)����������С�
		//�������ݴ���
		SG_Data_Process(rx_len);	//�ж��е����ݴ���ȥ�ı���Ӧ���Ʊ�����ֵ
		//��������DMA��2��DMA1_Stream2��Ӧ��
		MYDMA_Enable(DMA1_Stream2,SG_REC_LEN,2);
	}
	OSIntExit();
}
//�����ж��еļ�ʱ���ݴ�������
void SG_Data_Process(u16 rx_len)
{
//----------------------------------------֡β�ж�--------------------------------------------------//
	if(rx_len>2 && SG_RX_BUF[rx_len-1]=='C' && SG_RX_BUF[rx_len-2]=='R')
	{
			rx_len-=2;//ת������
			switch(SG_RX_BUF[0])
			{
//				case '1':LED1=!LED1;
			}
	}
}
//DMA�жϻ�����ûɶ�ã���Ҫ�Ǵ����жϣ��������ϲ������ֻҪ�������ݴ��䲻���죩����������жϻ����Ǹ����裩
//�����ԣ�1ms������������ڷ��Ͳ��ᵼ��DMA������������ᵼ�´��ڿ����жϲ���������
void DMA1_Stream2_IRQHandler(void)
{
	OSIntEnter();
	DMA_Interrupt_Process(DMA1_Stream2,(uint32_t)SG_RX_BUF,SG_REC_LEN,2);//�����жϴ���
	OSIntExit();
}
void SteeringGearSendCommand_180(u8 SG_ID, u16 time, u16 angle)
{
	angle = (int)(angle*2000/180 + 500);
	//֡ͷ
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	//���ݳ���
	USART_SendData(UART5, 0x08);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//ָ��
	USART_SendData(UART5, 0x03);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//�����
	USART_SendData(UART5, 0x01);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	//ʱ�䣨ms��
	USART_SendData(UART5, (uint8_t)time);//�Ͱ�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	USART_SendData(UART5, (uint8_t)(time>>8));//�߰�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//ID
	USART_SendData(UART5, SG_ID);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//�Ƕȣ��ȣ�
	USART_SendData(UART5, (uint8_t)angle);//�Ͱ�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(UART5, (uint8_t)(angle>>8));//�߰�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
}
void SteeringGearSendCommand_270(u8 SG_ID, u16 time, u16 angle)
{
	angle = (int)(angle*2000/270 + 500);
	//֡ͷ
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(UART5, 0x55);while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	//���ݳ���
	USART_SendData(UART5, 0x08);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//ָ��
	USART_SendData(UART5, 0x03);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//�����
	USART_SendData(UART5, 0x01);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	//ʱ�䣨ms��
	USART_SendData(UART5, (uint8_t)time);//�Ͱ�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���	
	USART_SendData(UART5, (uint8_t)(time>>8));//�߰�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//ID
	USART_SendData(UART5, SG_ID);//�򴮿�4��������
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	//�Ƕȣ��ȣ�
	USART_SendData(UART5, (uint8_t)angle);//�Ͱ�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
	USART_SendData(UART5, (uint8_t)(angle>>8));//�߰�λ
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
}
