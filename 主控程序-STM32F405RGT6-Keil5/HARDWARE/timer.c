#include "system_init.h"
/*��ʱ��6��7�����٣���Ҫ���ڼ�ʱ��TIM6�����ź�������(���ж�д��stm_it�ļ���)����PID����ʹ��TIM7(���ܿ�����TIM6���ã�������Ȼ�ж�����Ǿ�������)*/
void TIM6_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);  ///ʹ��TIM6ʱ��
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);//��ʼ��TIM6
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE); //����ʱ��6�����ж�
	TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	TIM_Cmd(TIM6,ENABLE); //ʹ�ܶ�ʱ��6
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn; //��ʱ��6�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
void TIM7_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);  ///ʹ��TIM7ʱ��
	
    TIM_TimeBaseInitStructure.TIM_Period = arr; //�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;//��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//��ʼ��TIM7
	
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE); //����ʱ��7�����ж�
	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_Cmd(TIM7,ENABLE); //ʹ�ܶ�ʱ��7
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn; //��ʱ��7�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x00; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM7_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM7,TIM_IT_Update) == SET)
	{
		//����5�������������PID��������������þ����˵���붨ʱ����IO�ڵ�ȷ����ϵ��һ��Ҫ��Ӧ��
		//���������������붨ʱ���š�PWM�����ʱ���Ÿ�λ��һ�£����˵��5����ͬʱͨ����ʹ��CH1
		MotorX_Control(&motor1_encoder,&Motor1PID,TIM1,TIM11,1,GPIOB,GPIO_Pin_0,GPIOB,GPIO_Pin_1);//��������PA8/9��PWM��PB9������PB0/1��
		MotorX_Control(&motor2_encoder,&Motor2PID,TIM2,TIM12,1,GPIOC,GPIO_Pin_4,GPIOC,GPIO_Pin_5);//��������PA15/PB3��PWM��PB14������PC4/5��
		MotorX_Control(&motor3_encoder,&Motor3PID,TIM3,TIM12,2,GPIOC,GPIO_Pin_13,GPIOC,GPIO_Pin_14);//��������PB4/5��PWM��PB15������PC13/PC14����ʹ����M6�ӿڿ��ƣ�
		MotorX_Control(&motor4_encoder,&Motor4PID,TIM4,TIM14,1,GPIOC,GPIO_Pin_0,GPIOC,GPIO_Pin_1);//��������PB6/7��PWM��PA7������PC0/1��
//		MotorX_Control(&motor5_encoder,&Motor5PID,TIM8,TIM10,1,GPIOA,GPIO_Pin_4,GPIOA,GPIO_Pin_5);//��������PC6/7��PWM��PB8������PA4/5��
		//���жϱ�־λ
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
		//ע��TIM12��CH2���ڿ����ޱ�����ֱ����ˢ�������������PWM��ռ�ձ�������24V����18V�����
//		MotorX_Control(&motor6_encoder,&Motor6PID,TIM��,TIM13, 1, GPIOB,GPIO_Pin_2,GPIOC,GPIO_Pin_13);//���������ޣ�PWM��PB15������PB2/PC13���������PB2��ѹ���͹�ʵ�ʺܿ����޷�����һЩ����ģ������ת��ֻ�ܵ���ת�������ų������Ըߵ�ƽҪ��ϵ�ģ����ԣ���
	}
	OSIntExit();
}
