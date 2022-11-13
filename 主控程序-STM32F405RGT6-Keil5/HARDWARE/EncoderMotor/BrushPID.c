#include "BrushPID.h"
//定义编码器电机的PID参数
_motorPID Motor1PID;
_motorPID Motor2PID;
_motorPID Motor3PID;
_motorPID Motor4PID;
_motorPID Motor5PID;
/*PID参数初始化*/
void MotorPID_param_init(_motorPID *Motor_PID,float LocTarget,float LocKp,float LocKi,float LocKd,float SpdTarget,float SpdKp,float SpdKi,float SpdKd,u32 maxpwm,u16 locintegrallimit,u16 spdintegrallimit,u16 locdead,u16 spddead)
{
	/* 位置相关初始化参数 */
    Motor_PID->pid_location.target_val=LocTarget;//表示初始化转到的位置（单位：圈。注意，不是增量，而是绝对量）
    Motor_PID->pid_location.actual_val=0.0f;
    Motor_PID->pid_location.err=0.0f;
    Motor_PID->pid_location.err_last=0.0f;
    Motor_PID->pid_location.integral=0.0f;
  
	Motor_PID->pid_location.Kp = LocKp;//位置环只设定比例项，因为位置必然是速度为0时才确定->kp=max_speed/(r_remain*ppr);一圈1560，对于速度为100rpm的情况，kp应该在0.064，i可以略给一点，给个0.0001
	Motor_PID->pid_location.Ki = LocKi;//一个非常小的数
	Motor_PID->pid_location.Kd = LocKd;//0即可

  	/* 速度相关初始化参数 */
    Motor_PID->pid_speed.target_val=0.0f;//实际上速度的目标值由位置环给定，SpdTarget用做后面的速度限制。
    Motor_PID->pid_speed.actual_val=0.0f;
    Motor_PID->pid_speed.err=0.0f;
    Motor_PID->pid_speed.err_last=0.0f;
    Motor_PID->pid_speed.integral=0.0f;
  
	Motor_PID->pid_speed.Kp = SpdKp;//将输入量放大Kp倍作为输出PWM。当位置环传给速度环的target是150，当设定转速为100，pwm上限为
	Motor_PID->pid_speed.Ki = SpdKi;
	Motor_PID->pid_speed.Kd = SpdKd;
	
	Motor_PID->AimSpeed = SpdTarget;
	/*一些限制性参数*/
	Motor_PID->MaxPWM = maxpwm;
	Motor_PID->LocIntegralLimit = locintegrallimit;
	Motor_PID->SpdIntegralLimit = spdintegrallimit;
	Motor_PID->LocDead = locdead;
	Motor_PID->SpdDead = spddead;
}

void Motorset_pid_target(_motorPID *Motor_PID, float LocTarget,float SpdTarget)
{
	Motor_PID->pid_location.target_val = LocTarget; //设置位置环目标值
	Motor_PID->pid_speed.target_val = SpdTarget; //设置速度环目标值
}

void MotorSet_pid_loc_target(_motorPID *Motor_PID, float LocTarget)
{
	Motor_PID->pid_location.target_val = LocTarget; //设置位置环目标值
}

void MotorSet_Speed_target(_motorPID *Motor_PID, float SpeedTarget)
{
	Motor_PID->AimSpeed = SpeedTarget; //设置位置环目标值
}

float MotorGet_pid_target(_motorPID *Motor_PID,u8 flag)
{
  if(flag==0) return Motor_PID->pid_location.target_val;    //获取当前的目标值
  else return Motor_PID->AimSpeed;
}
/*设置比例、积分、微分系数*/
void MotorSet_speed_p_i_d(_motorPID *Motor_PID, float p, float i, float d)
{
	Motor_PID->pid_speed.Kp = p;    // 设置比例系数 P
	Motor_PID->pid_speed.Ki = i;    // 设置积分系数 I
	Motor_PID->pid_speed.Kd = d;    // 设置微分系数 D
}
void MotorSet_location_p_i_d(_motorPID *Motor_PID, float p, float i, float d)
{
	Motor_PID->pid_location.Kp = p;    // 设置比例系数 P
	Motor_PID->pid_location.Ki = i;    // 设置积分系数 I
	Motor_PID->pid_location.Kd = d;    // 设置微分系数 D
}
/*位置PID算法*/
float MotorLocation_pid_realize(_motorPID *Motor_PID,_encoder *motor_encoder)
{
	/*偏差计算*/
    Motor_PID->pid_location.err=Motor_PID->pid_location.target_val*motor_encoder->PER_CYCLE_PULSES - Motor_PID->pid_location.actual_val;
    /*位置闭环死区*/
    if((Motor_PID->pid_location.err >= -Motor_PID->LocDead) && (Motor_PID->pid_location.err <= Motor_PID->LocDead))//位置死区，非常重要。
    {
      Motor_PID->pid_location.err = 0;
      Motor_PID->pid_location.integral = 0;
    }
    /*误差累积与积分限幅*/
    Motor_PID->pid_location.integral += Motor_PID->pid_location.err;
	if(Motor_PID->pid_location.integral > Motor_PID->LocIntegralLimit) Motor_PID->pid_location.integral  = Motor_PID->LocIntegralLimit;	//积分限幅 防止到达目标位置后过冲
	if(Motor_PID->pid_location.integral < -Motor_PID->LocIntegralLimit) Motor_PID->pid_location.integral =-Motor_PID->LocIntegralLimit;	//积分限幅 防止到达目标位置后过冲
	/*PID算法实现*/
    Motor_PID->pid_location.output_val = Motor_PID->pid_location.Kp*Motor_PID->pid_location.err
										 +Motor_PID->pid_location.Ki*Motor_PID->pid_location.integral
										 +Motor_PID->pid_location.Kd*(Motor_PID->pid_location.err-Motor_PID->pid_location.err_last);
	/*输出值根据目标速度值进行限幅*/
	if(Motor_PID->pid_location.output_val > Motor_PID->AimSpeed) Motor_PID->pid_location.output_val = Motor_PID->AimSpeed;
	else if(Motor_PID->pid_location.output_val < -Motor_PID->AimSpeed) Motor_PID->pid_location.output_val = -Motor_PID->AimSpeed;
	/*误差传递（必须在PID计算之后，因为微分要用到上一次的误差，如果在之前则微分失效）*/
    Motor_PID->pid_location.err_last=Motor_PID->pid_location.err;	
	/*返回当前实际值*/
    return Motor_PID->pid_location.output_val;
}
/*速度PID算法*/
float MotorSpeed_pid_realize(_motorPID *Motor_PID)
{
	/*计算目标值与实际值的误差*/
    Motor_PID->pid_speed.err = Motor_PID->pid_speed.target_val - Motor_PID->pid_speed.actual_val;//比如目标是100rpm，当前是0rpm，则相差为100。
//	/*速度死区，没啥大用*/
//    if((Motor_PID->pid_speed.err < Motor_PID->SpdDead) && (Motor_PID->pid_speed.err > -Motor_PID->SpdDead)) Motor_PID->pid_speed.err = 0.0f;
	/*误差累积与积分限幅*/
    Motor_PID->pid_speed.integral += Motor_PID->pid_speed.err;
	/*PID算法实现*/
    Motor_PID->pid_speed.output_val =  Motor_PID->pid_speed.Kp * Motor_PID->pid_speed.err
									  +Motor_PID->pid_speed.Ki * Motor_PID->pid_speed.integral
									  +Motor_PID->pid_speed.Kd * (Motor_PID->pid_speed.err - Motor_PID->pid_speed.err_last);
	
//	Motor_PID->pid_speed.output_val = Motor_PID->pid_speed.output_val;//防止开环的正负号转换。
	/*速度输出死区，即当PID计算得到的速度输出值很小时，认为已经非常接近目标位置了，可以取消PWM控制以“强制”使停稳，适用于对定位精度不高但对稳定性要求高的场合*/
//	if(Motor_PID->pid_speed.output_val <= 50 && Motor_PID->pid_speed.output_val >= -50) Motor_PID->pid_speed.output_val=0;//死区控制
	/*PWM限幅*/
//	if(Motor_PID->pid_speed.output_val > 4000) Motor_PID->pid_speed.output_val = 4000;
//	else if(Motor_PID->pid_speed.output_val < -4000) Motor_PID->pid_speed.output_val = -4000;
	
	if(Motor_PID->pid_speed.output_val > Motor_PID->MaxPWM) Motor_PID->pid_speed.output_val = Motor_PID->MaxPWM;
	else if(Motor_PID->pid_speed.output_val < -Motor_PID->MaxPWM) Motor_PID->pid_speed.output_val = -Motor_PID->MaxPWM;
	
//	Motor_PID->pid_speed.output_val = (Motor_PID->pid_speed.output_val >  Motor_PID->MaxPWM) ?  Motor_PID->MaxPWM : Motor_PID->pid_speed.output_val;//PWM上限处理
//	Motor_PID->pid_speed.output_val = (Motor_PID->pid_speed.output_val < -Motor_PID->MaxPWM) ? -Motor_PID->MaxPWM : Motor_PID->pid_speed.output_val;//PWM上限处理
	/*误差传递*/
    Motor_PID->pid_speed.err_last = Motor_PID->pid_speed.err;
//	Motor_PID->pid_speed.output_val = 8000;
	return Motor_PID->pid_speed.output_val;
}
