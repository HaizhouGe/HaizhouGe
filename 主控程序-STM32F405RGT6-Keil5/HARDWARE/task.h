#ifndef  __TASK_H__
#define  __TASK_H__
#include "os_cpu.h "

#define START_TASK_PRIO      		10 //一定要优先级最低
#define LED0_TASK_PRIO       		8
#define BrushMotor_TASK_PRIO       	6
#define SteeringGear_task_PRIO      5
#define POSTURE_TASK_PRIO       	4

#define START_STK_SIZE  				256u
#define LED0_STK_SIZE  				    256u
#define BrushMotor_STK_SIZE  			256u
#define SteeringGear_STK_SIZE      		256u
#define POSTURE_STK_SIZE                256u

extern __align(8) OS_STK START_TASK_STK[START_STK_SIZE];
extern __align(8) OS_STK LED0_TASK_STK[LED0_STK_SIZE];
extern __align(8) OS_STK BrushMotor_TASK_STK[BrushMotor_STK_SIZE];
extern __align(8) OS_STK SteeringGear_task_STK[SteeringGear_STK_SIZE];
extern __align(8) OS_STK POSTURE_TASK_STK[POSTURE_STK_SIZE];

void start_task(void *pdata);	
void led_task(void *pdata);	
void BrushMotor_task(void *pdata);
void SteeringGear_task(void *pdata);
void posture_task(void *pdata);
void tmr1_callback(void *p_tmr, void *p_arg); 	//定时器1回调函数

extern float movespeed_left;
extern float movespeed_right;

#endif
