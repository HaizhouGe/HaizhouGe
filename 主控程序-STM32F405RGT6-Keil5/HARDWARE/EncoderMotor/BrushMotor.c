/*
	�ϲ����Ŀ�����غ���
*/
#include "BrushMotor.h"
#include "delay.h"

u8 CancelMotorControl=0;
u8 brake = 0;

/*���PWM��PID���ƺ��������ڶ�ʱ�����������н���*/
void MotorX_Control(_encoder *motor_encoder,_motorPID *motor_PID,TIM_TypeDef *EncoderTIMx,TIM_TypeDef *TIMx,u8 chx, GPIO_TypeDef *GPIOx_left,uint16_t GPIO_Pin_left,GPIO_TypeDef *GPIOx_right,uint16_t GPIO_Pin_right)
{
    /* 10ms�ڲ�������������� */
    motor_encoder->Capture_Count = TIM_GetCounter(EncoderTIMx) - 0x8000;
    /* ���ת��ת�� = ����ʱ�䣨10ms���ڵı���������ֵ/�������ֱܷ���/ʱ��ϵ����ʱ��ϵ������ת����λΪr/min��*/
    motor_PID->pid_speed.actual_val = (float)motor_encoder->Capture_Count/motor_encoder->PER_CYCLE_PULSES/0.01f*60.0f;//���ۼ��㣬�������е�ֱ����ˢ������ڱ����̵������£�10ms�ڲ����ܳ�������������
	/* ���ת��λ�� = motor_encoder->Capture_Count�Ĳ����ۼ�*/
	motor_PID->pid_location.actual_val +=  motor_encoder->Capture_Count;
	/* λ��PID����,�������Ϊ�ٶȻ�Ŀ��ֵ��ע�����ٶ�Ŀ��ֵ����*/
	motor_PID->pid_speed.target_val = MotorLocation_pid_realize(motor_PID,motor_encoder);
	/* �ٶ�PID���㣬�������ΪPWM��ֵ���е������ */
	motor_PID->pid_speed.output_val = MotorSpeed_pid_realize(motor_PID);
//	if(motor_PID->pid_speed.output_val>2000) motor_PID->pid_speed.output_val = 2000;
//	if(motor_PID->pid_speed.output_val<-2000) motor_PID->pid_speed.output_val = -2000;
//	motor_PID->pid_speed.output_val = motor_PID->pid_speed.target_val*motor_PID->MaxPWM/motor_PID->AimSpeed;//ͨ��λ�û����ֵ�޶�
	/* �������ת�ٿ��� */
//	if(!brake) //��ǿ��ɲ��
//	{
	if(!CancelMotorControl)
	{
		if(motor_PID->pid_speed.output_val >= 0)
		{
			GPIO_ResetBits(GPIOx_left,GPIO_Pin_left);
			GPIO_SetBits(GPIOx_right,GPIO_Pin_right);
			switch(chx)
			{
				case 1:TIM_SetCompare1(TIMx,(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
				case 2:TIM_SetCompare2(TIMx,(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
				case 3:TIM_SetCompare3(TIMx,(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
				case 4:TIM_SetCompare4(TIMx,(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
			}
		}
		else if(motor_PID->pid_speed.output_val < 0)//�ٶ����С��0
		{
			GPIO_SetBits(GPIOx_left,GPIO_Pin_left);
			GPIO_ResetBits(GPIOx_right,GPIO_Pin_right);
			switch(chx)
			{
				case 1:TIM_SetCompare1(TIMx,-(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
				case 2:TIM_SetCompare2(TIMx,-(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
				case 3:TIM_SetCompare3(TIMx,-(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
				case 4:TIM_SetCompare4(TIMx,-(int)motor_PID->pid_speed.output_val);break;//���ñȽ�ֵ
			}
		}
	}
	else//����Ե���Ŀ���
	{
		switch(chx)
		{
			case 1:TIM_SetCompare1(TIMx,0);break;//���ñȽ�ֵ
			case 2:TIM_SetCompare2(TIMx,0);break;//���ñȽ�ֵ
			case 3:TIM_SetCompare3(TIMx,0);break;//���ñȽ�ֵ
			case 4:TIM_SetCompare4(TIMx,0);break;//���ñȽ�ֵ
		}
	}
//	}
//	else
//	{
//		GPIO_ResetBits(GPIOx_left,GPIO_Pin_left);
//		GPIO_ResetBits(GPIOx_right,GPIO_Pin_right);
//	}
	/* ������λ */
	TIM_SetCounter(EncoderTIMx,0x8000);
}
//��ʼ���߼�GPIO���
void BrushMotorLogicIOInit(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);//ʹ��GPIOAʱ��
	//PA��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	//PB��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);GPIO_ResetBits(GPIOB,GPIO_Pin_1);GPIO_ResetBits(GPIOB,GPIO_Pin_2);
	//PC��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC,GPIO_Pin_0);GPIO_ResetBits(GPIOC,GPIO_Pin_1);GPIO_ResetBits(GPIOC,GPIO_Pin_4);GPIO_ResetBits(GPIOC,GPIO_Pin_5);
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);GPIO_ResetBits(GPIOC,GPIO_Pin_14);GPIO_ResetBits(GPIOC,GPIO_Pin_15);
}
void SetMotorX_Speed_Position(_motorPID *motor_PID, float LocTarget,float SpdTarget)
{
	motor_PID->AimSpeed = SpdTarget;
	motor_PID->pid_location.target_val=LocTarget;
}
