#include "system_init.h"

OS_EVENT 		*SteeringGearSem;
OS_EVENT 		*BrushMotorSem;
OS_EVENT 		*LegSem;
OS_EVENT 		*PostureSem;

void System_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(168);//��ʼ����ʱ����
	//LED��KEY��ʼ��
	LED_Init();
	KEYEXTI_Init();
	//5�����ڳ�ʼ��
	uart1_init(115200);//����1��PC�����ã�ֻ����PA10���ն����ܷ���
	Initial_UART2(115200);//����2-JY-901ģ��
	RemoteUSART_Init(115200);//����3�������߿��������
	SGUSART_Init(9600);//����4�������ư�ͨ��
	VisionUSART_Init(9600);//����5�Ӿ�����գ�Ԥ�Ʋ������Ӿ��壬�������ű��ã�
	MagneticValve_Init();
	/**************������*************/
	//5+1������߼�IO��ʼ��
	BrushMotorLogicIOInit();
	/*���������PID���Ƴ�ʼ��*/
	//3����Ť�������ʼ����ת�����87��
	MotorPID_param_init(&Motor1PID,/*Ŀ��Ȧ��*/0,/*λ�û�PID*/0.4,0,0.2,/*Ŀ���ٶ�rpm*/85,/*�ٶȻ�PID*/35,1.2,10,/*PWM�޷�*/PWM_MAX_PERIOD_COUNT,/*���������޷�*/40,8000,/*�������*/5,1);
	MotorPID_param_init(&Motor2PID,/*Ŀ��Ȧ��*/0,/*λ�û�PID*/0.4,0,0.2,/*Ŀ���ٶ�rpm*/85,/*�ٶȻ�PID*/35,1.2,10,/*PWM�޷�*/PWM_MAX_PERIOD_COUNT,/*���������޷�*/40,8000,/*�������*/5,1);
	MotorPID_param_init(&Motor3PID,/*Ŀ��Ȧ��*/0,/*λ�û�PID*/0.4,0,0.2,/*Ŀ���ٶ�rpm*/40,/*�ٶȻ�PID*/35,2.1,10,/*PWM�޷�*/PWM_MAX_PERIOD_COUNT,/*���������޷�*/40,8000,/*�������*/5,1);
	//2��СŤ�������ʼ����ת�����333��
	MotorPID_param_init(&Motor4PID,/*Ŀ��Ȧ��*/0,/*λ�û�PID*/0.98,0.1,6,/*Ŀ���ٶ�rpm*/130,/*�ٶȻ�PID*/150,10,20,/*PWM�޷�*/PWM_MAX_PERIOD_COUNT,/*���������޷�*/40,8000,/*�������*/5,1);
//	MotorPID_param_init(&Motor5PID,/*Ŀ��Ȧ��*/2,/*λ�û�PID*/0.98,0.1,6,/*Ŀ���ٶ�rpm*/130,/*�ٶȻ�PID*/65,2,20,/*PWM�޷�*/PWM_MAX_PERIOD_COUNT,/*���������޷�*/40,4000,/*�������*/5,1);
	//5�������PWM��ʼ��
	TIM11_PWM_Init(8399,0);//PWM����Ϊ0.1ms����10kHz����׼Ƶ�ʣ���
	TIM12_PWM_Init(8399,0);//TIM12��������·PWM�����һ�����Ʊ��������2����һ�����Ʊ��������3����Ϊ3������PWM��Ϊ��PB2�����޷����������
//	TIM13_PWM_Init(8399,0);
	TIM14_PWM_Init(8399,0);
	TIM10_PWM_Init(8399,0);
//	TIM_SetCompare1(TIM13,0);//���ñȽ�ֵΪ0����PWM���Ϊ0����Ҫת����������д��TIM_SetCompare1(TIM13,6*Div8_PWM_PERIOD_COUNT);//һ����Կ���С��6������3��
	//5�����������encoder����ģʽ��ʼ��
	TIM1_Encoder_Init();
	TIM2_Encoder_Init();
	TIM3_Encoder_Init();//����TIM3��������ӦTIM12CH2��pwm�ˣ�����ӦM6�Ŀ��ƶ˿ڡ�
	TIM4_Encoder_Init();
	TIM8_Encoder_Init();
	//��ʱ����ʼ��
	TIM6_Int_Init(5000-1,84-1);//5msһ��,���ڿ��������ź���
	TIM7_Int_Init(8399,99);//10msһ�Σ����ڶ�������ֵ������PID����
	//���Ź���ʼ��
	IWDG_Init(6,400); //��ʼ�����Ź���ÿ500��4s����������0.4s������
}
/*
PID˵����
	�ٶȻ���λ�û������Ĳ���Ҫ����PWM���޵ĸı��Լ������ÿת��������ͬ���ı䣬�������£�
		��PWM�������ޱ仯ΪPWM_MAX_PERIOD_COUNT��1/2ʱ���ٶȻ������в�����Ҫ����2��
		�������ÿת�������仯Ϊԭ����2��ʱ��λ�û������в�����Ҫ�ɳ���2��
*/
