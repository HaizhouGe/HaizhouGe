#include "key.h"
#include "delay.h" 
#include "led.h"
#include "usart.h"	

//KEY1 PC9
//KEY2 PC8

//������ʼ������
void KEY_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//��ͨ����ģʽ
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

// �����ж�����
void KEYEXTI_Init(void)
{
	KEY_Init(); //������Ӧ��IO�ڳ�ʼ��
	//�жϳ�ʼ��
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource8);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource9);
	
    /* ����EXTI_Line8 */
    EXTI_InitStructure.EXTI_Line = EXTI_Line8;//LINE8
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //���س���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
    EXTI_Init(&EXTI_InitStructure);//����
	
	/* ����EXTI_Line9 */
	EXTI_InitStructure.EXTI_Line =  EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;//�ж���ʹ��
	EXTI_Init(&EXTI_InitStructure);//����
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;//�ⲿ�ж�8 9
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;//�����ȼ�2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//����
}

u8 KEY_Scan(void)
{	 		  
	if(KEY1==0||KEY2==0)
	{
		delay_ms(100);//ȥ����
		if(KEY1==0)return 1;
		else if(KEY2==0)return 2;
	}
 	return 0;// �ް�������
}
