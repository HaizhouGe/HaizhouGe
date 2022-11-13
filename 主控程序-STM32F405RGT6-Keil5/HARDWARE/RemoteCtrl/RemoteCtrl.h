#ifndef _RemoteUSART_H
#define _RemoteUSART_H
#include "sys.h"

void RemoteUSART_Init(int bound);
void Remote_Data_Process(u16 rx_len);
#define Remote_REC_LEN 50
extern u8 Remote_RX_BUF[Remote_REC_LEN];
extern u16 Remote_RX_STA;

extern u8 BM_Command;
extern u8 SG_Command;
extern u8 AutoPerform;
#endif
