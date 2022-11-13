#include "dma.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:�����ַ
//mar:�洢����ַ
//ndtr:���ݴ�����
//dir:���䷽��.0,�洢�������裻1�����赽�洢��
//it_flag:0,�������жϣ�1,������������ж�(һ����1)(��������жϵ������ж�һ�㲻�ÿ���) 
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr,u8 dir,u8 it_flag)
{
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2ʱ��ʹ��
	}
	else 
	{
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1ʱ��ʹ�� 
	}
	DMA_InitTypeDef  DMA_InitStructure;
	DMA_DeInit(DMA_Streamx);
	/* ���� DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //ͨ��ѡ��
	DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA�����ַ
	DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA �洢��0��ַ
	if(dir == 0)DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//�洢��������ģʽ
	else DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//�洢��������ģʽ
	DMA_InitStructure.DMA_BufferSize = ndtr;//���ݴ����� 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�洢������ģʽ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//�洢�����ݳ���:8λ
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//���������ģʽ
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//�������ݳ���:8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// ʹ����ͨģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//�����ȼ�
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//�洢��ͻ�����δ���
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//����ͻ�����δ���
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//��ʼ��DMA Stream
	DMA_ITConfig(DMA_Streamx,DMA_IT_TE,ENABLE);//��������ж����һֱ�������������������ض��Ŀ�������
	//�ж��Ƿ��������ж�
	if(it_flag == 1)DMA_ITConfig(DMA_Streamx,DMA_IT_TC,ENABLE);
	else DMA_ITConfig(DMA_Streamx,DMA_IT_TC,DISABLE);
	DMA_Cmd(DMA_Streamx, ENABLE);
}
//����һ��DMA����(�����ڲ����̣��ȹ�DMA��Ȼ�������־λ��Ȼ��ȴ��غú���������DMA��������¿���DMA) 
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:���ݴ�����
//it_config:0-7,�� DMA_Streamx��x��Ӧ 
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr, u8 it_config)
{
	DMA_Cmd(DMA_Streamx, DISABLE);//�ر�DMA����
	switch(it_config)
	{
		case 0:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);break;
		case 1:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);break;
		case 2:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_FEIF2);break;
		case 3:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);break;
		case 4:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);break;
		case 5:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);break;
		case 6:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);break;
		case 7:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);break;
		default:break;
	}
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����
	DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA����
}
//DMA�жϴ�����Ҫ��������DMA���ʱ�ṩ������ 
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//mar:�洢����ַ
//ndtr:���ݴ�����
//it_config:0-7
void DMA_Interrupt_Process(DMA_Stream_TypeDef *DMA_Streamx,u32 mar,u16 ndtr,u8 it_config)
{
//	uint32_t DMA_FLAG;
//	uint32_t ERR_FLAG;
//	switch (it_config)
//	{
//		case 0:DMA_FLAG=DMA_FLAG_TCIF0;ERR_FLAG=DMA_FLAG_TEIF0;break;
//		case 1:DMA_FLAG=DMA_FLAG_TCIF1;ERR_FLAG=DMA_FLAG_TEIF1;break;
//		case 2:DMA_FLAG=DMA_FLAG_TCIF2;ERR_FLAG=DMA_FLAG_TEIF2;break;
//		case 3:DMA_FLAG=DMA_FLAG_TCIF3;ERR_FLAG=DMA_FLAG_TEIF3;break;
//		case 4:DMA_FLAG=DMA_FLAG_TCIF4;ERR_FLAG=DMA_FLAG_TEIF4;break;
//		case 5:DMA_FLAG=DMA_FLAG_TCIF5;ERR_FLAG=DMA_FLAG_TEIF5;break;
//		case 6:DMA_FLAG=DMA_FLAG_TCIF6;ERR_FLAG=DMA_FLAG_TEIF6;break;
//		case 7:DMA_FLAG=DMA_FLAG_TCIF7;ERR_FLAG=DMA_FLAG_TEIF7;break;
//	}
//	if(DMA_GetFlagStatus(DMA_Streamx,DMA_FLAG) != RESET)
//	{
//			DMA_Cmd(DMA_Streamx, DISABLE); 					//�ر�DMA,��ֹ�������������
//			switch(it_config)
//			{
//				case 0:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF0 | DMA_FLAG_TEIF0 | DMA_FLAG_HTIF0 | DMA_FLAG_DMEIF0 | DMA_FLAG_FEIF0);break;
//				case 1:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF1 | DMA_FLAG_TEIF1 | DMA_FLAG_HTIF1 | DMA_FLAG_DMEIF1 | DMA_FLAG_FEIF1);break;
//				case 2:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF2 | DMA_FLAG_TEIF2 | DMA_FLAG_HTIF2 | DMA_FLAG_DMEIF2 | DMA_FLAG_FEIF2);break;
//				case 3:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF3 | DMA_FLAG_TEIF3 | DMA_FLAG_HTIF3 | DMA_FLAG_DMEIF3 | DMA_FLAG_FEIF3);break;
//				case 4:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF4 | DMA_FLAG_TEIF4 | DMA_FLAG_HTIF4 | DMA_FLAG_DMEIF4 | DMA_FLAG_FEIF4);break;
//				case 5:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_FEIF5);break;
//				case 6:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6);break;
//				case 7:DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF7 | DMA_FLAG_TEIF7 | DMA_FLAG_HTIF7 | DMA_FLAG_DMEIF7 | DMA_FLAG_FEIF7);break;
//				default:break;
//			}
//			while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//ȷ��DMA���Ա�����
//			DMA_Streamx->M0AR = (uint32_t)mar;
//			DMA_SetCurrDataCounter(DMA_Streamx,ndtr);
//			DMA_Cmd(DMA_Streamx, ENABLE);    			 		//��DMA
//	}
		
        //DMA������ʾ��LED����ƿ��������������ٱ�ִ�У� 
		while(1)
		{
//			LED1 = 1;
//			delay_ms(250);
//			LED1 = 0;
//			delay_ms(250);
		}
}
//��ȡ����DMA���յ������ݳ���
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//ndtr:���ݴ�����
u16 DMA_GetDataLenth(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	u16 rx_len;
	rx_len = ndtr - DMA_GetCurrDataCounter(DMA_Streamx);
	return rx_len;
}
