#ifndef _EP_H_
#define _EP_H_
#include "sys.h"
/*编码器*/
#define ENCODER_TIM_PERIOD   65535 //编码器定时器周期（重装载值，这个值是最大值，就不要改了）
#define PWM_MAX_PERIOD_COUNT 8000 //比PWM定时器的重装载值（8399）略小，24V电机使用
#define HALF_PWM_MAX_PERIOD_COUNT 4000 //取一半，从而可以供12V电机使用 
#define Div8_PWM_PERIOD_COUNT 1000 //取八分之一，即为3V，如对于18V电机，可以用6*Div8_PWM_PERIOD_COUNT

//编码器相关参数
typedef struct
{
	int  Capture_Count;       //当前时刻总计数值，表示一圈内的多少角度，可正可负
	float PER_CYCLE_PULSES;//电机每转一圈的总脉冲数，等于4*减速比*编码器线数
}_encoder;

extern _encoder motor1_encoder;
extern _encoder motor2_encoder;
extern _encoder motor3_encoder;
extern _encoder motor4_encoder;
extern _encoder motor5_encoder;

void TIM10_PWM_Init(u16 arr,u16 psc);
void TIM11_PWM_Init(u16 arr,u16 psc);
void TIM13_PWM_Init(u16 arr,u16 psc);
void TIM14_PWM_Init(u16 arr,u16 psc);
void TIM12_PWM_Init(u16 arr,u16 psc);

void TIM8_Encoder_Init(void);
void TIM2_Encoder_Init(void);
void TIM3_Encoder_Init(void);
void TIM4_Encoder_Init(void);
void TIM1_Encoder_Init(void);

void MotorEncoder_param_Init(_encoder *motor_encoder);
#endif
