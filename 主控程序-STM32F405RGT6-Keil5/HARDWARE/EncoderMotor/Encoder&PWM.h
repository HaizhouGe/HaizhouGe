#ifndef _EP_H_
#define _EP_H_
#include "sys.h"
/*������*/
#define ENCODER_TIM_PERIOD   65535 //��������ʱ�����ڣ���װ��ֵ�����ֵ�����ֵ���Ͳ�Ҫ���ˣ�
#define PWM_MAX_PERIOD_COUNT 8000 //��PWM��ʱ������װ��ֵ��8399����С��24V���ʹ��
#define HALF_PWM_MAX_PERIOD_COUNT 4000 //ȡһ�룬�Ӷ����Թ�12V���ʹ�� 
#define Div8_PWM_PERIOD_COUNT 1000 //ȡ�˷�֮һ����Ϊ3V�������18V�����������6*Div8_PWM_PERIOD_COUNT

//��������ز���
typedef struct
{
	int  Capture_Count;       //��ǰʱ���ܼ���ֵ����ʾһȦ�ڵĶ��ٽǶȣ������ɸ�
	float PER_CYCLE_PULSES;//���ÿתһȦ����������������4*���ٱ�*����������
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
