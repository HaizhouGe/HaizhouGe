#ifndef __PID_H
#define __PID_H	 
#include "sys.h"
#include "timer.h"
#include "Encoder&PWM.h" 

typedef struct
{
    float target_val;               //Ŀ��ֵ������λ�û������Ծ����Ȧ�������ģ���λ��ʽpid���м���ʱ�ں����г�����PER_CYCLE_PULSES����ת���������ٶȻ�����
    float actual_val;        		//ʵ��ֵ�����������������ģ������PER_CYCLE_PULSES��õ�ǰȦ����
	float output_val;               //���ֵ������λ�û���Ŀ���ٶ�ֵ
    float err;             			//����ƫ��ֵ
    float err_last;          		//������һ��ƫ��ֵ
    float Kp,Ki,Kd;          		//������������֡�΢��ϵ��
    float integral;          		//�������ֵ
}_pid;

typedef struct
{
	_pid pid_location;
	_pid pid_speed;
	float AimSpeed;
	float MaxPWM;
	u16 LocIntegralLimit;
	u16 SpdIntegralLimit;
	u16 LocDead;
	u16 SpdDead;
}_motorPID;

extern _motorPID Motor1PID;
extern _motorPID Motor2PID;
extern _motorPID Motor3PID;
extern _motorPID Motor4PID;
extern _motorPID Motor5PID;

float MotorLocation_pid_realize(_motorPID *Motor_PID,_encoder *motor_encoder);
float MotorSpeed_pid_realize(_motorPID *Motor_PID);
void MotorSet_location_p_i_d(_motorPID *Motor_PID, float p, float i, float d);
void MotorSet_speed_p_i_d(_motorPID *Motor_PID, float p, float i, float d);
float MotorGet_pid_target(_motorPID *Motor_PID,u8 flag);
void MotorSet_pid_loc_target(_motorPID *Motor_PID, float LocTarget);
void Motorset_pid_target(_motorPID *Motor_PID, float LocTarget,float SpdTarget);
void MotorPID_param_init(_motorPID *Motor_PID,float LocTarget,float LocKp,float LocKi,float LocKd,float SpdTarget,float SpdKp,float SpdKi,float SpdKd,u32 maxpwm,u16 locintegrallimit,u16 spdintegrallimit,u16 locdead,u16 spddead);

#endif
