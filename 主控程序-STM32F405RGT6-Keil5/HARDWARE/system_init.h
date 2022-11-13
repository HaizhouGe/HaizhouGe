#ifndef  __SYSTEM_INIT_H__
#define  __SYSTEM_INIT_H__

#include "includes.h"

#include "sys.h"
#include "delay.h"
#include "usart.h"

#include "VisionUSART.h"
#include "MagneticValve.h"
#include "iwdg.h"
#include "RemoteCtrl.h"
#include "UART2.h"
#include "JY901.h"
#include "dma.h"

#include "led.h"
#include "timer.h"
#include "key.h"
#include "task.h"
#include "Encoder&PWM.h"
#include "BrushPID.h"
#include "BrushMotor.h"
#include "SteeringGear.h"

extern OS_EVENT 	*SteeringGearSem;
extern OS_EVENT 	*BrushMotorSem;
extern OS_EVENT 	*LegSem;
extern OS_EVENT 	*PostureSem;

void System_Init(void);


#endif

