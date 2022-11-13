#ifndef __PID_H
#define __PID_H	 
#include "sys.h"
#include "timer.h"
#include "Encoder&PWM.h" 

typedef struct
{
    float target_val;               //目标值，对于位置环，是以具体的圈数计量的，在位置式pid进行计算时在函数中乘以了PER_CYCLE_PULSES进行转换；对于速度环，是
    float actual_val;        		//实际值，是以脉冲数计量的，需除以PER_CYCLE_PULSES获得当前圈数。
	float output_val;               //输出值，对于位置环是目标速度值
    float err;             			//定义偏差值
    float err_last;          		//定义上一个偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
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
