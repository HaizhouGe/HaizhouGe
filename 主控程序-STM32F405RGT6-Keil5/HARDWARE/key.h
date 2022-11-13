#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h" 

#define KEY1  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_9)
#define KEY2  GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_8)

#define KEY1_PRES		1
#define KEY2_PRES		2

void KEY_Init(void);	//IO初始化
void KEYEXTI_Init(void);	//外部中断初始化	
u8 KEY_Scan(void);  		//按键扫描函数	

#endif
