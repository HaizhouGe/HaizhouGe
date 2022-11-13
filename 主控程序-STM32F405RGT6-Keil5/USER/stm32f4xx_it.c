#include "stm32f4xx_it.h"
#include "system_init.h" 
//����ϵͳʱ�����ã���λ��ms��
/*
����LED����û�з����ź���������ʹ�õ�delay������LED�ƿ��԰����趨����ʱ������˸��֤���������õ���ʱû�����⡣������ܾ������⡣
ʱ������ԭ��
*/
#define SG_COUNTER       (5)
#define WALK_COUNTER     (5)
#define POSTURE_COUNTER  (10)

extern float angleWant1;

void TIM6_DAC_IRQHandler(void)
{
	static uint8_t steeringgearcounter = SG_COUNTER;
	static uint8_t brushmotorcounter  = WALK_COUNTER;
	static uint8_t posturecounter  = POSTURE_COUNTER;

	OSIntEnter();
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) == SET)
	{
		
		steeringgearcounter--;
		brushmotorcounter--;
		posturecounter --;
		
		if(steeringgearcounter==0){
		OSSemPost(SteeringGearSem);
		steeringgearcounter = SG_COUNTER;
		}
		
		if(brushmotorcounter==0) {
		OSSemPost(BrushMotorSem);
		brushmotorcounter = WALK_COUNTER; 
		}
		if(posturecounter==0) {
		OSSemPost(PostureSem);
		posturecounter = POSTURE_COUNTER; 
		}
		
    TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	}
	OSIntExit(); 
}

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
