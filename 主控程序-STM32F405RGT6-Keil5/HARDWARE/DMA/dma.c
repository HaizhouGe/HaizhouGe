#include "dma.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//par:外设地址
//mar:存储器地址
//ndtr:数据传输量
//dir:传输方向.0,存储器到外设；1，外设到存储器
//it_flag:0,不开启中断；1,开启传输错误中断(一般置1)(传输完成中断等其它中断一般不用开启) 
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr,u8 dir,u8 it_flag)
{
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);//DMA2时钟使能
	}
	else 
	{
	    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);//DMA1时钟使能 
	}
	DMA_InitTypeDef  DMA_InitStructure;
	DMA_DeInit(DMA_Streamx);
	/* 配置 DMA Stream */
	DMA_InitStructure.DMA_Channel = chx;  //通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = par;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = mar;//DMA 存储器0地址
	if(dir == 0)DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;//存储器到外设模式
	else DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//存储器到外设模式
	DMA_InitStructure.DMA_BufferSize = ndtr;//数据传输量 
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//存储器增量模式
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//存储器数据长度:8位
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设非增量模式
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;// 使用普通模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//高优先级
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_Init(DMA_Streamx, &DMA_InitStructure);//初始化DMA Stream
	DMA_ITConfig(DMA_Streamx,DMA_IT_TE,ENABLE);//传输错误中断最好一直开启，并在其中设置特定的卡死程序。
	//判断是否开启其它中断
	if(it_flag == 1)DMA_ITConfig(DMA_Streamx,DMA_IT_TC,ENABLE);
	else DMA_ITConfig(DMA_Streamx,DMA_IT_TC,DISABLE);
	DMA_Cmd(DMA_Streamx, ENABLE);
}
//开启一次DMA传输(函数内部流程：先关DMA，然后清除标志位，然后等待关好后重新设置DMA，最后重新开启DMA) 
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
//ndtr:数据传输量
//it_config:0-7,与 DMA_Streamx的x对应 
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr, u8 it_config)
{
	DMA_Cmd(DMA_Streamx, DISABLE);//关闭DMA传输
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
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}
//DMA中断处理（主要是用于在DMA溢出时提供报警） 
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//mar:存储器地址
//ndtr:数据传输量
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
//			DMA_Cmd(DMA_Streamx, DISABLE); 					//关闭DMA,防止处理其间有数据
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
//			while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置
//			DMA_Streamx->M0AR = (uint32_t)mar;
//			DMA_SetCurrDataCounter(DMA_Streamx,ndtr);
//			DMA_Cmd(DMA_Streamx, ENABLE);    			 		//打开DMA
//	}
		
        //DMA错误提示（LED陷入灯快闪，其它任务不再被执行） 
		while(1)
		{
//			LED1 = 1;
//			delay_ms(250);
//			LED1 = 0;
//			delay_ms(250);
		}
}
//获取本次DMA接收到的数据长度
//chx:DMA通道选择,@ref DMA_channel DMA_Channel_0~DMA_Channel_7
//ndtr:数据传输量
u16 DMA_GetDataLenth(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
	u16 rx_len;
	rx_len = ndtr - DMA_GetCurrDataCounter(DMA_Streamx);
	return rx_len;
}
