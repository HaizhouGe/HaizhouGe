#ifndef __BM_H
#define __BM_H

#include "Encoder&PWM.h" 
#include "BrushPID.h"
extern u8 brake;
extern u8 CancelMotorControl;
void BrushMotorLogicIOInit(void);
void MotorX_Control(_encoder *motor_encoder,_motorPID *motor_PID,TIM_TypeDef *EncoderTIMx,TIM_TypeDef *TIMx,u8 chx, GPIO_TypeDef *GPIOx_left,uint16_t GPIO_Pin_left,GPIO_TypeDef *GPIOx_right,uint16_t GPIO_Pin_right);
void SetMotorX_Speed_Position(_motorPID *motor_PID, float LocTarget,float SpdTarget);
#endif

