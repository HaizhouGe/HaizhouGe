#include "system_init.h"
#include "task.h"
#include "JY901.h"
#include "BrushMotor.h"
#include "iwdg.h"

float angleWant1 = 0;
float movespeed_left = 85;
float movespeed_right = 85;

//����������ݽ����õĽṹ��
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

OS_TMR  *tmr1;//��ʱ��1
//��ʼ����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata = pdata; 
	INT8U err = 0;
	
	SteeringGearSem=OSSemCreate(0);
	BrushMotorSem=OSSemCreate(0);
	LegSem=OSSemCreate(0);
	PostureSem=OSSemCreate(0);
	
	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��)   
	//����һ����ʱ��
	tmr1=OSTmrCreate ((INT32U)           0,
					 (INT32U)            1, //1*5=5ms
					 (INT8U)             OS_TMR_OPT_PERIODIC,//����ģʽ
					 (OS_TMR_CALLBACK)   tmr1_callback,//��ʱ��1�ص�����
					 (void        * )    0,//����Ϊ0
					 (INT8U       * )    "tmr1",//��ʱ������
					 (INT8U       * )    &err);//���صĴ�����
	//�������񴴽�
 	OSTaskCreate(led_task,(void *)0,(OS_STK*)&LED0_TASK_STK[LED0_STK_SIZE-1],LED0_TASK_PRIO);		
 	OSTaskCreate(SteeringGear_task,(void *)0,(OS_STK*)&SteeringGear_task_STK[SteeringGear_STK_SIZE-1],SteeringGear_task_PRIO);					 
    OSTaskCreate(BrushMotor_task,(void *)0,(OS_STK*)&BrushMotor_TASK_STK[BrushMotor_STK_SIZE-1],BrushMotor_TASK_PRIO);	
	OSTaskCreate(posture_task,(void *)0,(OS_STK*)&POSTURE_TASK_STK[POSTURE_STK_SIZE-1],POSTURE_TASK_PRIO);
	//��ش���
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OSTmrStart(tmr1,&err);			//������ʱ��1
	OS_EXIT_CRITICAL();				//�˳��ٽ���(���Ա��жϴ��)
}
//LED����(���������
void led_task(void *pdata)
{
	pdata = pdata;
	while(1)
	{
		IWDG_Feed();
		LED = !LED;
//		printf("Motor4��ǰ����:%d\r\n",motor4_encoder.Capture_Count);
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
//�����������
void SteeringGear_task(void *pdata)
{
	u8 err;
	pdata = pdata;
	delay_ms(400);
	//���λ�ó�ʼ��
	SteeringGearSendCommand_180(0,500,65);//��������
	
	SteeringGearSendCommand_270(1,100,47);//���ư壬������
	SteeringGearSendCommand_180(2,100,20);//���ư壬������
	
	SteeringGearSendCommand_180(3,250,165);//������������
	SteeringGearSendCommand_180(15,1500,115);//����������ƽ��
	
//	SteeringGearSendCommand_270(1,250,137);//���ư壬������
//	SteeringGearSendCommand_180(2,500,110);//���ư壬������
	
//	SteeringGearSendCommand_270(1,100,157);//���ư壬б����
//	SteeringGearSendCommand_180(2,500,130);//���ư壬б����
	
//	SteeringGearSendCommand_180(3,250,165);//������������
//	SteeringGearSendCommand_180(3,250,120);//������������
	
//	SteeringGearSendCommand_270(15,50,80);//����������ƽ��
//	delay_ms(500);
//	SteeringGearSendCommand_180(15,500,15);//����������ƽ��
//	SteeringGearSendCommand_180(15,1500,115);//����������ƽ��
	delay_ms(1500);
	while(1)
	{
		OSSemPend(SteeringGearSem,0,&err);
		IWDG_Feed();
		if(SG_Command!=0)
		{
			switch(SG_Command)//��������ĽǶȺ�ִ��ʱ��
			{
				//����
				case 'T':
						//���壬������
						SteeringGearSendCommand_270(1,250,137);//���ư壬������
						SteeringGearSendCommand_180(2,250,110);//���ư壬������
						delay_ms(250);
						//��������������
						SteeringGearSendCommand_180(3,250,65);//�������������� 115
						delay_ms(500);
						//���壬б����������������б����
						SteeringGearSendCommand_270(1,100,160);//���ư壬б����
						SteeringGearSendCommand_180(2,100,133);//���ư壬б����
						delay_ms(300);
						SteeringGearSendCommand_270(1,250,137);//���ư壬������
						SteeringGearSendCommand_180(2,250,110);//���ư壬������
						delay_ms(300);
						SteeringGearSendCommand_270(1,100,160);//���ư壬б����
						SteeringGearSendCommand_180(2,500,133);//���ư壬б����
						delay_ms(450);
						//ȫ����λ
						SteeringGearSendCommand_180(3,250,165);//������������չ
						SteeringGearSendCommand_270(1,150,47);//���ư壬������
						SteeringGearSendCommand_180(2,150,20);//���ư壬������
						delay_ms(800);
						
				break;
				//����(�����ӵ�������ȥ)
				case 'G':
						SteeringGearSendCommand_180(15,500,15);
						delay_ms(700);
						SteeringGearSendCommand_180(15,500,115);//��λ
				break;
				
				default:
				break;
			}
			SG_Command = 0;
		}
	}
}
//ֱ����ˢ�����������
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
//			//״̬ת��ʱ��ý��ж���ɲ�����Ա�������ģ��
//			brake = 1;
//			delay_ms(200);//ɲ��0.2s
//			brake = 0;
//			BM_Command_Last = BM_Command;
//		}
		IWDG_Feed();
		switch(BM_Command)//�������ø��������Ŀ��λ�ú��ٶ�
		{
			case 'W':
					IMU_Direction_Ctrl(0,0);//IMU_Direction_Ctrl(u8 direction,float yawtarget)
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES-1.0f,movespeed_left);//Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES�����ǰȦ��
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES+1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//ȡ��������PID���ƣ�����Ӳ��������ɵ���ת
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//ȡ����ͷ�������PID���ƣ�����Ӳ��������ɵ���ת
			break;
			case 'A':
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES-1.0f,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES-1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//ȡ��������PID���ƣ�����Ӳ��������ɵ���ת
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//ȡ����ͷ�������PID���ƣ�����Ӳ��������ɵ���ת			
			break;
			case 'S':
					IMU_Direction_Ctrl(1,0);
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES+1.0f,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES-1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//ȡ��������PID���ƣ�����Ӳ��������ɵ���ת
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//ȡ����ͷ�������PID���ƣ�����Ӳ��������ɵ���ת
			break;
			case 'D':
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES+1.0f,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES+1.0f,movespeed_right);
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,0);//ȡ��������PID���ƣ�����Ӳ��������ɵ���ת
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,0);//ȡ����ͷ�������PID���ƣ�����Ӳ��������ɵ���ת			
			break;
			case 'X'://ֹͣ
					IMU_Direction_Ctrl(0,0);
					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);
					BM_Command = '!';
					SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES,30);//ȡ��������PID���ƣ�����Ӳ��������ɵ���ת
					SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,40);//ȡ����ͷ�������PID���ƣ�����Ӳ��������ɵ���ת	
			break;
//			case 'Q'://ǰ���̶�����
//					SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES - 5.0f,movespeed_left);
//					SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES + 5.0f,movespeed_right);
//					BM_Command = '!';
//			break;

//			case 'E'://��ͷ
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
//			case 'Z'://���ʮ
//					if(!(yaw<91 && yaw > 89))
//					{
//						SetMotorX_Speed_Position(&Motor1PID,--Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
//						SetMotorX_Speed_Position(&Motor2PID,++Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);						
//					}
//			break;
//			
//			case 'C'://�Ҿ�ʮ
//					if(!(yaw>-91 && yaw < -89))
//					{
//						SetMotorX_Speed_Position(&Motor1PID,++Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
//						SetMotorX_Speed_Position(&Motor2PID,--Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);						
//					}
//			break;

			//����
			case 'F':
				SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES+0.11f,30);
				delay_ms(1000);
				SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES-0.01f,30);
				delay_ms(500);
				BM_Command = '!';
			break;

			//���+עˮ
			case 'K':
					//��������
					Mag2ON;
					//�·�
					SetMotorX_Speed_Position(&Motor3PID,-7.15f,60);
					Mag1ON;//����עˮ
					delay_ms(3000);
					Mag1OFF;//�ر�עˮ
					SetMotorX_Speed_Position(&Motor3PID,-7.15f,15);
					delay_ms(21000);
					//����
					SetMotorX_Speed_Position(&Motor3PID,0.0f,40);//�ȿ�������
					delay_ms(7900);
					SetMotorX_Speed_Position(&Motor3PID,-1.0f,15);//����������
					delay_ms(3000);
					//�رյ���
					Mag2OFF;
					//�˳�����
					BM_Command = '!';
			break;

			//�޼�������ͷ�½�
			case 'T':
				SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES-1.0f,25);
				BM_Command = '?';
			break;
			//�޼�������ͷ����
			case 'N':
				if(Motor3PID.pid_location.actual_val>50) SetMotorX_Speed_Position(&Motor3PID,0,25);
				else SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES+1.0f,25);
				BM_Command = '?';
			break;
			case '?'://�����޼�
				SetMotorX_Speed_Position(&Motor3PID,Motor3PID.pid_location.actual_val/motor3_encoder.PER_CYCLE_PULSES,25);
				BM_Command = '!';
			break;
			//��ͷ��ת
			case 'O':
				Mag2ON;
				BM_Command = '!';
			break;
			//��ͷͣת
			case 'I':
				Mag2OFF;
				BM_Command = '!';
			break;
			default:
			break;
		}
	}
}
//�ߵ��Ƕ���Ϣ
void posture_task(void *pdata)
{
	u8 err;
	u8 RestartFlag=0;
	pdata = pdata;
	//��һ��JY901��ʼ�����
	delay_ms(2000);
	//���ٶ�У׼����ȷ��ģ���ʼ̬��
	AccCalibrate();
	while(1)
	{
		OSSemPend(PostureSem,0,&err);
		IWDG_Feed();
		//����Ƕ���Ϣ��ӡ
//		printf("roll=%hd\r\n",roll);
//		printf("pitch=%hd\r\n",pitch);
//		printf("yaw=%hd\r\n",yaw);
//		printf("*****************\r\n");
		//��������ò��ŵĹߵ���������Զ�����ִ��
		if(AutoPerform == 1)
		{
			/****��ǰ���̶�����****/
				SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES - 5.0f,movespeed_left);
				SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES + 5.0f,movespeed_right);
				delay_ms(6000);
			/********���*******/
				Mag1ON;
				delay_ms(300);//�Ե�ת��
				//�·�
				SetMotorX_Speed_Position(&Motor3PID,-1,40);
				delay_ms(5000);
				//����
				SetMotorX_Speed_Position(&Motor3PID,0,40);
				delay_ms(5000);
				//�رյ���
				Mag1OFF;
				delay_ms(300);//�Ե�ͣ��
			/******����******/
				SetMotorX_Speed_Position(&Motor4PID,Motor4PID.pid_location.actual_val/motor4_encoder.PER_CYCLE_PULSES+1.0f,130);
				delay_ms(600);
			/******����******/
				SteeringGearSendCommand_180(15,500,15);
				delay_ms(700);
				SteeringGearSendCommand_180(15,500,115);//��λ	
				delay_ms(400);
			/******����******/
				//���壬������
				SteeringGearSendCommand_270(1,250,137);//���ư壬������
				SteeringGearSendCommand_180(2,250,110);//���ư壬������
				delay_ms(250);
				//��������������
				SteeringGearSendCommand_180(3,250,115);//��������������
				delay_ms(500);
				//���壬б����������������б����
				SteeringGearSendCommand_270(1,100,160);//���ư壬б����
				SteeringGearSendCommand_180(2,100,133);//���ư壬б����
				delay_ms(500);
				SteeringGearSendCommand_270(1,250,137);//���ư壬������
				SteeringGearSendCommand_180(2,250,110);//���ư壬������
				delay_ms(500);
				SteeringGearSendCommand_270(1,100,160);//���ư壬б����
				SteeringGearSendCommand_180(2,500,133);//���ư壬б����
				delay_ms(500);
				//ȫ����λ
				SteeringGearSendCommand_180(3,250,165);//������������չ
				SteeringGearSendCommand_270(1,150,47);//���ư壬������
				SteeringGearSendCommand_180(2,150,20);//���ư壬������
				delay_ms(200);
			/*******��ǰ���̶�����*******/
				SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES - 2.0f,movespeed_left);
				SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES + 2.0f,movespeed_right);
				delay_ms(3000);
			AutoPerform = 0;
		}
		//���������ص���ʼ̬
		else if(AutoPerform == 2)
		{
//			//ͣ��
//			brake = 1;//״̬ת��ʱ��ý��ж���ɲ�����Ա�������ģ��
//			delay_ms(200);//ɲ��0.2s
//			brake = 0;
			IMU_Direction_Ctrl(0,0);
			SetMotorX_Speed_Position(&Motor1PID,Motor1PID.pid_location.actual_val/motor1_encoder.PER_CYCLE_PULSES,movespeed_left);
			SetMotorX_Speed_Position(&Motor2PID,Motor2PID.pid_location.actual_val/motor2_encoder.PER_CYCLE_PULSES,movespeed_right);			
			//�̵����ر�
			Mag1OFF;Mag2OFF;
			//4������ص���ʼ̬
			SteeringGearSendCommand_270(1,100,47);//���ư壬������
			SteeringGearSendCommand_180(2,100,20);//���ư壬������
			SteeringGearSendCommand_180(3,250,165);//������������
			SteeringGearSendCommand_180(15,1500,115);//����������ƽ��
			//��ˢ����ص���ʼ̬�����������ã�
			SetMotorX_Speed_Position(&Motor3PID,0,60);
			AutoPerform = 0;
		}
		delay_ms(350);
		switch(BM_Command)
		{
			//עˮ
			case 'P':
			{
				static u8 i = 0;
				if(i==0){Mag1ON;i++;}
				else {Mag1OFF;i=0;}
				//�˳�����
				BM_Command = '!';
			}
			break;

			//����
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
			CancelMotorControl = 1;//�����Ե���Ŀ��ƣ��������¸�λ
			while(1);//�޷�ι��������������
		}
	}
}
//����ϵͳ�Ķ�ʱ����
void tmr1_callback(void *p_tmr, void *p_arg)
{
	
}
