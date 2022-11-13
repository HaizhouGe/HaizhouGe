#ifndef _VISIONUSART_H
#define _VISIONUSART_H
#include "sys.h"

void VisionUSART_Init(int bound);
void Vision_Data_Process(u16 rx_len);
#define VISION_REC_LEN 50
extern u8 VISION_RX_BUF[VISION_REC_LEN];
extern u16 VISION_RX_STA;

#define MoveDelayTimes 15
#endif
