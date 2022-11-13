#ifndef __Mag_H
#define __Mag_H
#include "sys.h"

#define Mag1ON   GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define Mag1OFF  GPIO_ResetBits(GPIOB,GPIO_Pin_12)

#define Mag2ON   GPIO_SetBits(GPIOB,GPIO_Pin_13)
#define Mag2OFF  GPIO_ResetBits(GPIOB,GPIO_Pin_13)

void MagneticValve_Init(void);//≥ı ºªØ
#endif
