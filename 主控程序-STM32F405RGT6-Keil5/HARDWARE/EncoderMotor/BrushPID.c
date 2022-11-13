#include "BrushPID.h"
//��������������PID����
_motorPID Motor1PID;
_motorPID Motor2PID;
_motorPID Motor3PID;
_motorPID Motor4PID;
_motorPID Motor5PID;
/*PID������ʼ��*/
void MotorPID_param_init(_motorPID *Motor_PID,float LocTarget,float LocKp,float LocKi,float LocKd,float SpdTarget,float SpdKp,float SpdKi,float SpdKd,u32 maxpwm,u16 locintegrallimit,u16 spdintegrallimit,u16 locdead,u16 spddead)
{
	/* λ����س�ʼ������ */
    Motor_PID->pid_location.target_val=LocTarget;//��ʾ��ʼ��ת����λ�ã���λ��Ȧ��ע�⣬�������������Ǿ�������
    Motor_PID->pid_location.actual_val=0.0f;
    Motor_PID->pid_location.err=0.0f;
    Motor_PID->pid_location.err_last=0.0f;
    Motor_PID->pid_location.integral=0.0f;
  
	Motor_PID->pid_location.Kp = LocKp;//λ�û�ֻ�趨�������Ϊλ�ñ�Ȼ���ٶ�Ϊ0ʱ��ȷ��->kp=max_speed/(r_remain*ppr);һȦ1560�������ٶ�Ϊ100rpm�������kpӦ����0.064��i�����Ը�һ�㣬����0.0001
	Motor_PID->pid_location.Ki = LocKi;//һ���ǳ�С����
	Motor_PID->pid_location.Kd = LocKd;//0����

  	/* �ٶ���س�ʼ������ */
    Motor_PID->pid_speed.target_val=0.0f;//ʵ�����ٶȵ�Ŀ��ֵ��λ�û�������SpdTarget����������ٶ����ơ�
    Motor_PID->pid_speed.actual_val=0.0f;
    Motor_PID->pid_speed.err=0.0f;
    Motor_PID->pid_speed.err_last=0.0f;
    Motor_PID->pid_speed.integral=0.0f;
  
	Motor_PID->pid_speed.Kp = SpdKp;//���������Ŵ�Kp����Ϊ���PWM����λ�û������ٶȻ���target��150�����趨ת��Ϊ100��pwm����Ϊ
	Motor_PID->pid_speed.Ki = SpdKi;
	Motor_PID->pid_speed.Kd = SpdKd;
	
	Motor_PID->AimSpeed = SpdTarget;
	/*һЩ�����Բ���*/
	Motor_PID->MaxPWM = maxpwm;
	Motor_PID->LocIntegralLimit = locintegrallimit;
	Motor_PID->SpdIntegralLimit = spdintegrallimit;
	Motor_PID->LocDead = locdead;
	Motor_PID->SpdDead = spddead;
}

void Motorset_pid_target(_motorPID *Motor_PID, float LocTarget,float SpdTarget)
{
	Motor_PID->pid_location.target_val = LocTarget; //����λ�û�Ŀ��ֵ
	Motor_PID->pid_speed.target_val = SpdTarget; //�����ٶȻ�Ŀ��ֵ
}

void MotorSet_pid_loc_target(_motorPID *Motor_PID, float LocTarget)
{
	Motor_PID->pid_location.target_val = LocTarget; //����λ�û�Ŀ��ֵ
}

void MotorSet_Speed_target(_motorPID *Motor_PID, float SpeedTarget)
{
	Motor_PID->AimSpeed = SpeedTarget; //����λ�û�Ŀ��ֵ
}

float MotorGet_pid_target(_motorPID *Motor_PID,u8 flag)
{
  if(flag==0) return Motor_PID->pid_location.target_val;    //��ȡ��ǰ��Ŀ��ֵ
  else return Motor_PID->AimSpeed;
}
/*���ñ��������֡�΢��ϵ��*/
void MotorSet_speed_p_i_d(_motorPID *Motor_PID, float p, float i, float d)
{
	Motor_PID->pid_speed.Kp = p;    // ���ñ���ϵ�� P
	Motor_PID->pid_speed.Ki = i;    // ���û���ϵ�� I
	Motor_PID->pid_speed.Kd = d;    // ����΢��ϵ�� D
}
void MotorSet_location_p_i_d(_motorPID *Motor_PID, float p, float i, float d)
{
	Motor_PID->pid_location.Kp = p;    // ���ñ���ϵ�� P
	Motor_PID->pid_location.Ki = i;    // ���û���ϵ�� I
	Motor_PID->pid_location.Kd = d;    // ����΢��ϵ�� D
}
/*λ��PID�㷨*/
float MotorLocation_pid_realize(_motorPID *Motor_PID,_encoder *motor_encoder)
{
	/*ƫ�����*/
    Motor_PID->pid_location.err=Motor_PID->pid_location.target_val*motor_encoder->PER_CYCLE_PULSES - Motor_PID->pid_location.actual_val;
    /*λ�ñջ�����*/
    if((Motor_PID->pid_location.err >= -Motor_PID->LocDead) && (Motor_PID->pid_location.err <= Motor_PID->LocDead))//λ���������ǳ���Ҫ��
    {
      Motor_PID->pid_location.err = 0;
      Motor_PID->pid_location.integral = 0;
    }
    /*����ۻ�������޷�*/
    Motor_PID->pid_location.integral += Motor_PID->pid_location.err;
	if(Motor_PID->pid_location.integral > Motor_PID->LocIntegralLimit) Motor_PID->pid_location.integral  = Motor_PID->LocIntegralLimit;	//�����޷� ��ֹ����Ŀ��λ�ú����
	if(Motor_PID->pid_location.integral < -Motor_PID->LocIntegralLimit) Motor_PID->pid_location.integral =-Motor_PID->LocIntegralLimit;	//�����޷� ��ֹ����Ŀ��λ�ú����
	/*PID�㷨ʵ��*/
    Motor_PID->pid_location.output_val = Motor_PID->pid_location.Kp*Motor_PID->pid_location.err
										 +Motor_PID->pid_location.Ki*Motor_PID->pid_location.integral
										 +Motor_PID->pid_location.Kd*(Motor_PID->pid_location.err-Motor_PID->pid_location.err_last);
	/*���ֵ����Ŀ���ٶ�ֵ�����޷�*/
	if(Motor_PID->pid_location.output_val > Motor_PID->AimSpeed) Motor_PID->pid_location.output_val = Motor_PID->AimSpeed;
	else if(Motor_PID->pid_location.output_val < -Motor_PID->AimSpeed) Motor_PID->pid_location.output_val = -Motor_PID->AimSpeed;
	/*���ݣ�������PID����֮����Ϊ΢��Ҫ�õ���һ�ε��������֮ǰ��΢��ʧЧ��*/
    Motor_PID->pid_location.err_last=Motor_PID->pid_location.err;	
	/*���ص�ǰʵ��ֵ*/
    return Motor_PID->pid_location.output_val;
}
/*�ٶ�PID�㷨*/
float MotorSpeed_pid_realize(_motorPID *Motor_PID)
{
	/*����Ŀ��ֵ��ʵ��ֵ�����*/
    Motor_PID->pid_speed.err = Motor_PID->pid_speed.target_val - Motor_PID->pid_speed.actual_val;//����Ŀ����100rpm����ǰ��0rpm�������Ϊ100��
//	/*�ٶ�������ûɶ����*/
//    if((Motor_PID->pid_speed.err < Motor_PID->SpdDead) && (Motor_PID->pid_speed.err > -Motor_PID->SpdDead)) Motor_PID->pid_speed.err = 0.0f;
	/*����ۻ�������޷�*/
    Motor_PID->pid_speed.integral += Motor_PID->pid_speed.err;
	/*PID�㷨ʵ��*/
    Motor_PID->pid_speed.output_val =  Motor_PID->pid_speed.Kp * Motor_PID->pid_speed.err
									  +Motor_PID->pid_speed.Ki * Motor_PID->pid_speed.integral
									  +Motor_PID->pid_speed.Kd * (Motor_PID->pid_speed.err - Motor_PID->pid_speed.err_last);
	
//	Motor_PID->pid_speed.output_val = Motor_PID->pid_speed.output_val;//��ֹ������������ת����
	/*�ٶ��������������PID����õ����ٶ����ֵ��Сʱ����Ϊ�Ѿ��ǳ��ӽ�Ŀ��λ���ˣ�����ȡ��PWM�����ԡ�ǿ�ơ�ʹͣ�ȣ������ڶԶ�λ���Ȳ��ߵ����ȶ���Ҫ��ߵĳ���*/
//	if(Motor_PID->pid_speed.output_val <= 50 && Motor_PID->pid_speed.output_val >= -50) Motor_PID->pid_speed.output_val=0;//��������
	/*PWM�޷�*/
//	if(Motor_PID->pid_speed.output_val > 4000) Motor_PID->pid_speed.output_val = 4000;
//	else if(Motor_PID->pid_speed.output_val < -4000) Motor_PID->pid_speed.output_val = -4000;
	
	if(Motor_PID->pid_speed.output_val > Motor_PID->MaxPWM) Motor_PID->pid_speed.output_val = Motor_PID->MaxPWM;
	else if(Motor_PID->pid_speed.output_val < -Motor_PID->MaxPWM) Motor_PID->pid_speed.output_val = -Motor_PID->MaxPWM;
	
//	Motor_PID->pid_speed.output_val = (Motor_PID->pid_speed.output_val >  Motor_PID->MaxPWM) ?  Motor_PID->MaxPWM : Motor_PID->pid_speed.output_val;//PWM���޴���
//	Motor_PID->pid_speed.output_val = (Motor_PID->pid_speed.output_val < -Motor_PID->MaxPWM) ? -Motor_PID->MaxPWM : Motor_PID->pid_speed.output_val;//PWM���޴���
	/*����*/
    Motor_PID->pid_speed.err_last = Motor_PID->pid_speed.err;
//	Motor_PID->pid_speed.output_val = 8000;
	return Motor_PID->pid_speed.output_val;
}
