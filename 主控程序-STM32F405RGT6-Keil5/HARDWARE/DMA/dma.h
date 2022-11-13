#ifndef __DMA_H
#define	__DMA_H	   
#include "sys.h"
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr,u8 dir,u8 it_flag);//配置DMAx_CHx
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr,u8 it_config);	//使能一次DMA传输		   
void DMA_Interrupt_Process(DMA_Stream_TypeDef *DMA_Streamx,u32 mar,u16 ndtr,u8 it_config);
u16 DMA_GetDataLenth(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
#endif
