#ifndef _SG_H
#define _SG_H
#include "sys.h"

void SGUSART_Init(int bound);
void SG_Data_Process(u16 rx_len);
void SteeringGearSendCommand_180(u8 SG_ID, u16 time, u16 angle);
void SteeringGearSendCommand_270(u8 SG_ID, u16 time, u16 angle);
#define SG_REC_LEN 50
extern u8 SG_RX_BUF[SG_REC_LEN];
extern u16 SG_RX_STA;
#endif
