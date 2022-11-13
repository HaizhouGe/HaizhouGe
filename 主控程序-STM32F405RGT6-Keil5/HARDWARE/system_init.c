#include "system_init.h"

OS_EVENT 		*SteeringGearSem;
OS_EVENT 		*BrushMotorSem;
OS_EVENT 		*LegSem;
OS_EVENT 		*PostureSem;

void System_Init(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init(168);//初始化延时函数
	//LED与KEY初始化
	LED_Init();
	KEYEXTI_Init();
	//5个串口初始化
	uart1_init(115200);//串口1与PC调试用，只能用PA10接收而不能发送
	Initial_UART2(115200);//串口2-JY-901模块
	RemoteUSART_Init(115200);//串口3进行无线控制与调试
	SGUSART_Init(9600);//串口4与舵机控制板通信
	VisionUSART_Init(9600);//串口5视觉板接收（预计不会有视觉板，所以留着备用）
	MagneticValve_Init();
	/**************电机相关*************/
	//5+1个电机逻辑IO初始化
	BrushMotorLogicIOInit();
	/*编码器电机PID控制初始化*/
	//3个大扭力电机初始化（转速最大87）
	MotorPID_param_init(&Motor1PID,/*目标圈数*/0,/*位置环PID*/0.4,0,0.2,/*目标速度rpm*/85,/*速度环PID*/35,1.2,10,/*PWM限幅*/PWM_MAX_PERIOD_COUNT,/*两环积分限幅*/40,8000,/*误差死区*/5,1);
	MotorPID_param_init(&Motor2PID,/*目标圈数*/0,/*位置环PID*/0.4,0,0.2,/*目标速度rpm*/85,/*速度环PID*/35,1.2,10,/*PWM限幅*/PWM_MAX_PERIOD_COUNT,/*两环积分限幅*/40,8000,/*误差死区*/5,1);
	MotorPID_param_init(&Motor3PID,/*目标圈数*/0,/*位置环PID*/0.4,0,0.2,/*目标速度rpm*/40,/*速度环PID*/35,2.1,10,/*PWM限幅*/PWM_MAX_PERIOD_COUNT,/*两环积分限幅*/40,8000,/*误差死区*/5,1);
	//2个小扭力电机初始化（转速最大333）
	MotorPID_param_init(&Motor4PID,/*目标圈数*/0,/*位置环PID*/0.98,0.1,6,/*目标速度rpm*/130,/*速度环PID*/150,10,20,/*PWM限幅*/PWM_MAX_PERIOD_COUNT,/*两环积分限幅*/40,8000,/*误差死区*/5,1);
//	MotorPID_param_init(&Motor5PID,/*目标圈数*/2,/*位置环PID*/0.98,0.1,6,/*目标速度rpm*/130,/*速度环PID*/65,2,20,/*PWM限幅*/PWM_MAX_PERIOD_COUNT,/*两环积分限幅*/40,4000,/*误差死区*/5,1);
	//5个电机的PWM初始化
	TIM11_PWM_Init(8399,0);//PWM周期为0.1ms，即10kHz（标准频率）。
	TIM12_PWM_Init(8399,0);//TIM12配置了两路PWM输出，一个控制编码器电机2，另一个控制编码器电机3（因为3本来的PWM因为是PB2所以无法驱动电机）
//	TIM13_PWM_Init(8399,0);
	TIM14_PWM_Init(8399,0);
	TIM10_PWM_Init(8399,0);
//	TIM_SetCompare1(TIM13,0);//设置比较值为0，即PWM输出为0。若要转动，则这样写：TIM_SetCompare1(TIM13,6*Div8_PWM_PERIOD_COUNT);//一般调试可以小于6，比如3。
	//5个编码器电机encoder输入模式初始化
	TIM1_Encoder_Init();
	TIM2_Encoder_Init();
	TIM3_Encoder_Init();//现在TIM3编码器对应TIM12CH2的pwm了，即对应M6的控制端口。
	TIM4_Encoder_Init();
	TIM8_Encoder_Init();
	//定时器初始化
	TIM6_Int_Init(5000-1,84-1);//5ms一次,用于控制任务信号量
	TIM7_Int_Init(8399,99);//10ms一次，用于读编码器值并进行PID控制
	//看门狗初始化
	IWDG_Init(6,400); //初始化看门狗。每500是4s。这里配置0.4s重启。
}
/*
PID说明：
	速度环和位置环两环的参数要随着PWM上限的改变以及电机的每转脉冲数不同而改变，具体如下：
		当PWM上限上限变化为PWM_MAX_PERIOD_COUNT的1/2时，速度环的所有参数均要除以2。
		当电机的每转脉冲数变化为原来的2倍时，位置环的所有参数均要成除以2。
*/
