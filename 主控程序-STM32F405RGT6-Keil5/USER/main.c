#define SYSTEM_SUPPORT0_OS 1
#include "system_init.h"

int main(void)
{
#if SYSTEM_SUPPORT0_OS 
	System_Init();//ȫ����ʼ��
	OSInit();
 	OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();
#else
#endif	
}
