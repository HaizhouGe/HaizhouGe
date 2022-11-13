#include "system_init.h"
#include "task.h"
#include "JY901.h"
#include "BrushMotor.h"
#include "iwdg.h"

float angleWant1 = 0;
float movespeed_left = 85;
float movespeed_right = 85;

//定义进行数据解析用的结构体
struct STime		stcTime;
struct SAcc 		stcAcc;
struct SGyro 		stcGyro;
struct SAngle 	    stcAngle;
struct SMag 		stcMag;
struct SDStatus     stcDStatus;
struct SPress 	    stcPress;
struct SLonLat 	    stcLonLat;
struct SGPSV 		stcGPSV;
struct SQ           stcQ;

__align(8) OS_STK START_TASK_STK[START_STK_SIZE];
__align(8) OS_STK LED0_TASK_STK[LED0_STK_SIZE];
__align(8) OS_STK BrushMotor_TASK_STK[BrushMotor_STK_SIZE];
__align(8) OS_STK SteeringGear_task_STK[SteeringGear_STK_SIZE];
__align(8) OS_STK POSTURE_TASK_STK[POSTURE_STK_SIZE];

OS_TMR  *tmr1;//定时器1
//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
	INT8U err = 0;
	
	SteeringGearSem=OSSemCreate(0);
	BrushMotorSem=OSSemCreate(0);
	LegSem=OSSemCreate(0);
	PostureSem=OSSemCreate(0);
	
	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断)   
	//创建一个定时器
	tmr1=OSTmrCreate ((INT32U)           0,
					 (INT32U)            1, //1*5=5ms
					 (INT8U)             OS_TMR_OPT_PERIODIC,//周期模式
					 (OS_TMR_CALLBACK)   tmr1_callback,//定时器1回调函数
					 (void        * )    0,//参数为0
					 (INT8U       * )    "tmr1",//定时器名字
					 (INT8U       * )    &err);//返回的错误码
	//其它任务创建
 	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);		
 	OSTaskCreate(SteeringGear_task,(void *)0,(OS_STK*)&SteeringGear_task_STK[SteeringGear_STK_SIZE-1],SteeringGear_task_PRIO);					 
    OSTaskCreate(BrushMotor_task,(void *)0,(OS_STK*)&BrushMotor_TASK_STK[BrushMotor_STK_SIZE-1],BrushMotor_TASK_PRIO);	
	OSTaskCreate(posture_task,(void *)0,(OS_STK*)&POSTURE_TASK_STK[POSTURE_STK_SIZE-1],POSTURE_TASK_PRIO);
	//相关处理
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OSTmrStart(tmr1,&err);			//开启定时器1
	OS_EXIT_CRITICAL();				//退出临界区(可以被中断打断)
}
//LED任务(兼调试任务）
void led_task(void *pdata)
{
	pdata = pdata;
	while(1)
	{
		IWDG_Feed();
		LED = !LED;
//		printf("Motor4当前捕获:%d\r\n",motor4_encoder.Capture_Count);
//		printf("Motor4PID.pid_location.output_val:%f\r\n",Motor4PID.pid_location.output_val);
//		printf("Motor4PID.pid_location.target_val:%f\r\n",Motor4PID.pid_location.target_val);
//		printf("Motor4PID.pid_location.actual_val:%d\r\n",Motor4PID.pid_location.actual_val);
//		
//		printf("Motor4PID.pid_speed.target_val:%f\r\n",Motor4PID.pid_speed.target_val);
//		printf("Motor4PID.pid_speed.actual_val:%d\r\n",Motor4PID.pid_speed.actual_val);
//		printf("Motor4PID.pid_speed.output_val:%f\r\n",Motor4PID.pid_speed.output_val);
//		printf("************************************************\r\n");
//		u8 b0 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0);
//		u8 b1 = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1);
		delay_ms(500);
	}
}
//舵机控制任务
void SteeringGear_task(void *pdata)
{
	u8 err;
	pdata = pdata;
	delay_ms(400);
	//舵机位置初始化
	SteeringGearSendCommand_180(0,500,65);//启动命令
	
	SteeringGearSendCommand_270(1,100,47);//左推板，横起来
	SteeringGearSendCommand_180(2,100,20);//右推板，横起来
	
	SteeringGearSendCommand_180(3,250,165);//伸缩机构，伸
	SteeringGearSendCommand_180(15,1500,115);//种树机构，平放
	
//	SteeringGearSendCommand_270(1,250,137);//左推板，竖起来
//	SteeringGearSendCommand_180(2,500,110);//右推板，竖起来
	
//	SteeringGearSendCommand_270(1,100,157);//左推板，斜起来
//	SteeringGearSendCommand_180(2,500,130);//右推板，斜起来
	
//	SteeringGearSendCommand_180(3,250,165);//伸缩机构，伸
//	SteeringGearSendCommand_180(3,250,120);//伸缩机构，缩
	
//	SteeringGearSendCommand_270(15,50,80);//种树机构，平放
//	delay_ms(500);
//	SteeringGearSendCommand_180(15,500,15);//种树机构，平放
//	SteeringGearSendCommand_180(15,1500,115);//种树机构，平放
	delay_ms(1500);
	while(1)
	{
		OSSemPend(SteeringGearSem,0,&err);
		IWDG_Feed();
		if(SG_Command!=0)
		{
			switch(SG_Command)//各个舵机的角度和执行时间
			{
				//填土
				case 'T':
						//挡板，竖起来
						SteeringGearSendCommand_270(1,250,137);//左推板，竖起来
						SteeringGearSendCommand_180(2,250,110);//右推板，竖起来
						delay_ms(250);
						//伸缩机构，收缩
						SteeringGearSendCommand_180(3,250,65);//伸缩机构，收缩 115
						delay_ms(500);
						//挡板，斜起来，竖起来，再斜起来
						SteeringGearSendCommand_270(1,100,160);//左推板，斜起来
						SteeringGearSendCommand_180(2,100,133);//右推板，斜起来
						delay_ms(300);
						SteeringGearSendCommand_270(1,250,137);//左推板，竖起来
						SteeringGearSendCommand_180(2,250,110);//右推板，竖起来
						delay_ms(300);
						SteeringGearSendCommand_270(1,100,160);//左推板，斜起来
						SteeringGearSendCommand_180(2,500,133);//右推板，斜起来
						delay_ms(450);
						//全部复位
						SteeringGearSendCommand_180(3,250,165);//伸缩机构，伸展
						SteeringGearSendCommand_270(1,150,47);//左推板，横起来
						SteeringGearSendCommand_180(2,150,20);//右推板，横起来
						delay_ms(800);
						
				break;
				//种树(让苗子掉到坑里去)
				case 'G':
						SteeringGearSendCommand_180(15,500,15);
						delay_ms(700);
						SteeringGearSendCommand_180(15,500,115);//复位
				break;
				
				default:
				break;
			}
			SG_Command = 0;
		}
	}
}
//直流有刷电机控制任务
void BrushMotor_task(void *pdata)
{
	u8 err;
	pdata = pdata;
//	u8 BM_Command_Last = '!';
	while(1)
	{
		OSSemPend(BrushMotorSem,0,&err);
//		if((BM_Command!=BM_Command_Last) && (BM_Command != '?'))
//		{
//			//状态转换时最好进行短暂刹车，以保护驱动模块
//			brake = 1;
//			delay_ms(200);//刹车0.2s
//			brake = 0;
//			BM_Command_Last = BM_Command;
//		}
		IWDG_Feed();
		switch(BM_Command)//用来设置各个电机的目标位置和速度
		{
			case 'W':
					IMU_Direction_Ctrl(0,0);//IMU_Direction_Ctrl(u8 direction,float yawtarget)
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES-1.0f,movespeed_left);//Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES算出当前圈数
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES+1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//取消送苗电机PID控制，避免硬件波动造成的乱转
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//取消钻头升降电机PID控制，避免硬件波动造成的乱转
			break;
			case 'A':
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES-1.0f,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES-1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//取消送苗电机PID控制，避免硬件波动造成的乱转
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//取消钻头升降电机PID控制，避免硬件波动造成的乱转			
			break;
			case 'S':
					IMU_Direction_Ctrl(1,0);
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES+1.0f,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES-1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//取消送苗电机PID控制，避免硬件波动造成的乱转
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//取消钻头升降电机PID控制，避免硬件波动造成的乱转
			break;
			case 'D':
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES+1.0f,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES+1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//取消送苗电机PID控制，避免硬件波动造成的乱转
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//取消钻头升降电机PID控制，避免硬件波动造成的乱转			
			break;
			case 'X'://停止
					IMU_Direction_Ctrl(0,0);
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);
					BM_Command = '!';
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,30);//取消送苗电机PID控制，避免硬件波动造成的乱转
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,40);//取消钻头升降电机PID控制，避免硬件波动造成的乱转	
			break;
//			case 'Q'://前进固定距离
//					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES - 5.0f,movespeed_left);
//					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES + 5.0f,movespeed_right);
//					BM_Command = '!';
//			break;

//			case 'E'://掉头
//					if(yaw<0 && yaw>-178)
//					{
//						SetMotorX_Speed_Position(&Motor1PID,--Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
//						SetMotorX_Speed_Position(&Motor2PID,++Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);
//					}
//					else if (yaw>0 && yaw<178)
//					{
//						SetMotorX_Speed_Position(&Motor1PID,++Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
//						SetMotorX_Speed_Position(&Motor2PID,--Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);						
//					}
//			break;
//			
//			case 'Z'://左九十
//					if(!(yaw<91 && yaw > 89))
//					{
//						SetMotorX_Speed_Position(&Motor1PID,--Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
//						SetMotorX_Speed_Position(&Motor2PID,++Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);						
//					}
//			break;
//			
//			case 'C'://右九十
//					if(!(yaw>-91 && yaw < -89))
//					{
//						SetMotorX_Speed_Position(&Motor1PID,++Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
//						SetMotorX_Speed_Position(&Motor2PID,--Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);						
//					}
//			break;

			//送苗
			case 'F':
				SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES+0.11f,30);
				delay_ms(1000);
				SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES-0.01f,30);
				delay_ms(500);
				BM_Command = '!';
			break;

			//钻孔+注水
			case 'K':
					//启动电钻
					Mag2ON;
					//下放
					SetMotorX_Speed_Position(&Motor3PID,-7.15f,60);
					Mag1ON;//启动注水
					delay_ms(3000);
					Mag1OFF;//关闭注水
					SetMotorX_Speed_Position(&Motor3PID,-7.15f,15);
					delay_ms(21000);
					//上拉
					SetMotorX_Speed_Position(&Motor3PID,0.0f,40);//先快速上拉
					delay_ms(7900);
					SetMotorX_Speed_Position(&Motor3PID,-1.0f,15);//再慢速上拉
					delay_ms(3000);
					//关闭电钻
					Mag2OFF;
					//退出命令
					BM_Command = '!';
			break;

			//无极调节钻头下降
			case 'T':
				SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES-1.0f,25);
				BM_Command = '?';
			break;
			//无极调节钻头上升
			case 'N':
				if(Motor3PID.pid_location.actual_val>50) SetMotorX_Speed_Position(&Motor3PID,0,25);
				else SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES+1.0f,25);
				BM_Command = '?';
			break;
			case '?'://配套无极
				SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,25);
				BM_Command = '!';
			break;
			//钻头起转
			case 'O':
				Mag2ON;
				BM_Command = '!';
			break;
			//钻头停转
			case 'I':
				Mag2OFF;
				BM_Command = '!';
			break;
			default:
			break;
		}
	}
}
//惯导角度信息
void posture_task(void *pdata)
{
	u8 err;
	u8 RestartFlag=0;
	pdata = pdata;
	//等一等JY901初始化完成
	delay_ms(2000);
	//加速度校准（即确定模块初始态）
	AccCalibrate();
	while(1)
	{
		OSSemPend(PostureSem,0,&err);
		IWDG_Feed();
		//三大角度信息打印
//		printf("roll=%hd\r\n",roll);
//		printf("pitch=%hd\r\n",pitch);
//		printf("yaw=%hd\r\n",yaw);
//		printf("*****************\r\n");
		//借用这个用不着的惯导任务进行自动任务执行
		if(AutoPerform == 1)
		{
			/****先前进固定距离****/
				SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES - 5.0f,movespeed_left);
				SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES + 5.0f,movespeed_right);
				delay_ms(6000);
			/********钻孔*******/
				Mag1ON;
				delay_ms(300);//略等转稳
				//下放
				SetMotorX_Speed_Position(&Motor3PID,-1,40);
				delay_ms(5000);
				//上拉
				SetMotorX_Speed_Position(&Motor3PID,0,40);
				delay_ms(5000);
				//关闭电钻
				Mag1OFF;
				delay_ms(300);//略等停稳
			/******送苗******/
				SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES+1.0f,130);
				delay_ms(600);
			/******种树******/
				SteeringGearSendCommand_180(15,500,15);
				delay_ms(700);
				SteeringGearSendCommand_180(15,500,115);//复位	
				delay_ms(400);
			/******填土******/
				//挡板，竖起来
				SteeringGearSendCommand_270(1,250,137);//左推板，竖起来
				SteeringGearSendCommand_180(2,250,110);//右推板，竖起来
				delay_ms(250);
				//伸缩机构，收缩
				SteeringGearSendCommand_180(3,250,115);//伸缩机构，收缩
				delay_ms(500);
				//挡板，斜起来，竖起来，再斜起来
				SteeringGearSendCommand_270(1,100,160);//左推板，斜起来
				SteeringGearSendCommand_180(2,100,133);//右推板，斜起来
				delay_ms(500);
				SteeringGearSendCommand_270(1,250,137);//左推板，竖起来
				SteeringGearSendCommand_180(2,250,110);//右推板，竖起来
				delay_ms(500);
				SteeringGearSendCommand_270(1,100,160);//左推板，斜起来
				SteeringGearSendCommand_180(2,500,133);//右推板，斜起来
				delay_ms(500);
				//全部复位
				SteeringGearSendCommand_180(3,250,165);//伸缩机构，伸展
				SteeringGearSendCommand_270(1,150,47);//左推板，横起来
				SteeringGearSendCommand_180(2,150,20);//右推板，横起来
				delay_ms(200);
			/*******再前进固定距离*******/
				SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES - 2.0f,movespeed_left);
				SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES + 2.0f,movespeed_right);
				delay_ms(3000);
			AutoPerform = 0;
		}
		//结束，即回到初始态
		else if(AutoPerform == 2)
		{
//			//停车
//			brake = 1;//状态转换时最好进行短暂刹车，以保护驱动模块
//			delay_ms(200);//刹车0.2s
//			brake = 0;
			IMU_Direction_Ctrl(0,0);
			SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
			SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);			
			//继电器关闭
			Mag1OFF;Mag2OFF;
			//4个舵机回到初始态
			SteeringGearSendCommand_270(1,100,47);//左推板，横起来
			SteeringGearSendCommand_180(2,100,20);//右推板，横起来
			SteeringGearSendCommand_180(3,250,165);//伸缩机构，伸
			SteeringGearSendCommand_180(15,1500,115);//种树机构，平放
			//有刷电机回到初始态（其它不必用）
			SetMotorX_Speed_Position(&Motor3PID,0,60);
			AutoPerform = 0;
		}
		delay_ms(350);
		switch(BM_Command)
		{
			//注水
			case 'P':
			{
				static u8 i = 0;
				if(i==0){Mag1ON;i++;}
				else {Mag1OFF;i=0;}
				//退出命令
				BM_Command = '!';
			}
			break;

			//重启
			case 'R':
			{
				RestartFlag=1;
				SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,25);
				BM_Command = '!';
			}
			break;
			
			default:
				
			break;
		}
		if(RestartFlag)
		{
			CancelMotorControl = 1;//放弃对电机的控制，便于重新复位
			while(1);//无法喂狗，等着重启吧
		}
	}
}
//操作系统的定时任务
void tmr1_callback(void *p_tmr, void *p_arg)
{
	
}
