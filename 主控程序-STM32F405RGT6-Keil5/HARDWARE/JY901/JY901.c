#include "sys.h"
#include "delay.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "UART2.h"
#include "JY901.h"
#include "BrushPID.h"
#include "task.h"

_pid Yaw_PID_Loop = {0};
short int aacx,aacy,aacz;
short int roll,pitch,yaw;
//JY901S�����ݲɼ�ʹ�õ���ؽṹ����ⲿ������
extern struct STime		stcTime;
extern struct SAcc 		stcAcc;
extern struct SGyro 	stcGyro;
extern struct SAngle 	stcAngle;
extern struct SMag 		stcMag;
extern struct SDStatus  stcDStatus;
extern struct SPress 	stcPress;
extern struct SLonLat 	stcLonLat;
extern struct SGPSV 	stcGPSV;
extern struct SQ        stcQ;
//��������ָ��
char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//������ٶ�У׼ģʽ
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//���浱ǰ����
//�ô���2��JYģ�鷢��ָ��
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++) UART2_Put_Char(cmd[i]);
}
//CopeSerialDataΪ����2�жϵ��ú���������ÿ�յ�һ�����ݣ�����һ������������Զ��յ������ݽ��н�����
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	
	ucRxBuffer[ucRxCnt++]=ucData;	//���յ������ݴ��뻺������
	if (ucRxBuffer[0]!=0x55) //����ͷ���ԣ������¿�ʼѰ��0x55����ͷ
	{
		ucRxCnt=0;
		return;//�˳�����
	}
	//����ͷ��ȷ�����Ҹú����ۼƵ�����11�Σ������һ��������������Ϣ������ϣ�Ȼ��ͨ��switch���ж�Ӧ����
	if (ucRxCnt<11) {return;}//���ݲ���11�����򷵻أ��˳�������
	else
	{
		switch(ucRxBuffer[1])//�ж��������������ݣ�Ȼ���俽������Ӧ�Ľṹ���С���Щ���ݰ���Ҫͨ����λ���򿪶�Ӧ������󣬲��ܽ��յ�������ݰ������ݡ�
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpyΪ�������Դ����ڴ濽��������������"string.h"�������ǽ����ջ��������ַ����������ݽṹ�����棬�Ӷ�ʵ�����ݵĽ�����
			case 0x51:	memcpy(&stcAcc,&ucRxBuffer[2],8);break;
			case 0x52:	memcpy(&stcGyro,&ucRxBuffer[2],8);break;
			case 0x53:	memcpy(&stcAngle,&ucRxBuffer[2],8);break;
			case 0x54:	memcpy(&stcMag,&ucRxBuffer[2],8);break;
			case 0x55:	memcpy(&stcDStatus,&ucRxBuffer[2],8);break;
			case 0x56:	memcpy(&stcPress,&ucRxBuffer[2],8);break;
			case 0x57:	memcpy(&stcLonLat,&ucRxBuffer[2],8);break;
			case 0x58:	memcpy(&stcGPSV,&ucRxBuffer[2],8);break;
			case 0x59:	memcpy(&stcQ,&ucRxBuffer[2],8);break;
		}
		ucRxCnt=0;//��ջ�������ʵ������û��գ�����Ϊ����֮ǰ��������׼����
	}
}
//ת������1�յ������ݸ�����2��JYģ�飩
void CopeSerial1Data(unsigned char ucData)
{
	UART2_Put_Char(ucData);
}
//���ٶ�У׼����
void AccCalibrate(void)
{
//	printf("\r\n���ڽ��м��ٶ�У׼\r\n");
	sendcmd(ACCCALSW);delay_ms(100);//�ȴ�ģ���ڲ��Զ�У׼�ã�ģ���ڲ����Զ�������Ҫһ����ʱ��
	sendcmd(SAVACALSW);delay_ms(100);//���浱ǰ����
//	printf("\r\n���ٶ�У׼���\r\n");
}
//ȡ����ֵ����
u8 ABS(short int x)
{
	if(x<0)
	{
		x = -x;
	}
	return x;
}
//���ٶ����ݻ�ȡ������printf���е��ԣ�
void JY901_Get_Accelerometer(void)
{
	aacx = (short int)((float)stcAcc.a[0]/32768*16*10);
	aacy = (short int)((float)stcAcc.a[1]/32768*16*10);
	aacz = (short int)((float)stcAcc.a[2]/32768*16*10);
}
//�Ƕ����ݻ�ȡ������printf���е��ԣ�
void JY901_Get_Angle(void)
{
	roll  = (short int)((float)stcAngle.Angle[0]/32768*180);
	pitch = (short int)((float)stcAngle.Angle[1]/32768*180);
	yaw   = (short int)((float)stcAngle.Angle[2]/32768*180);
}
//λ��ʽPID����(����λ�ã��Ƕȣ������������Ϊ�ٶȻ���Ŀ��ֵ)
int32_t  PID_PosLocCalc(_pid *pid,float Now_Point)//λ��ʽ
{
    float Now_Error,d_Error;
	Now_Error=pid->target_val - Now_Point;
	pid->integral+=Now_Error;
	//�����޷�
    if(pid->integral>4000) pid->integral=4000;
	else if(pid->integral<-4000) pid->integral=-4000;
	
	d_Error=Now_Error - pid->err_last;
    pid->err_last = Now_Error;
	pid->output_val=pid->Kp*Now_Error+
		            pid->Ki*pid->integral+
	                pid->Kd*d_Error;
	//�ٶ�Ŀ��ֵ���޷�
	if(pid->output_val>200)  pid->output_val= 200;
	if(pid->output_val<-200) pid->output_val=-200;
	return pid->output_val;
}
//�����Զ�����
void IMU_Direction_Ctrl(u8 direction,float yawtarget)
{
	static u8 j = 0;
	float movespeed_base;
	if(j == 0)//PID��ʼ��
	{
		movespeed_base =  movespeed_left = movespeed_right;
		j=1;
		Yaw_PID_Loop.Kp = 20;
		Yaw_PID_Loop.Kd = 2;
		Yaw_PID_Loop.target_val = yawtarget;
	}
	Yaw_PID_Loop.output_val = PID_PosLocCalc(&Yaw_PID_Loop,yaw);//���û�ȡ����yaw�ǽ���PID���㡣
	if(direction == 1) Yaw_PID_Loop.output_val=-Yaw_PID_Loop.output_val;//���˷���
	//��������
	if((Yaw_PID_Loop.output_val<1.0f) && (Yaw_PID_Loop.output_val>-1.0f)) Yaw_PID_Loop.output_val=0;
	//�ٶȿ���
	movespeed_left = movespeed_base - Yaw_PID_Loop.output_val;
	movespeed_right =movespeed_base + Yaw_PID_Loop.output_val;
	//�ٶ��޷�
	if(movespeed_left<0) movespeed_left=0;
	if(movespeed_right<0) movespeed_left=0;
	if(movespeed_left>200) movespeed_left=200;
	if(movespeed_right>200) movespeed_left=200;
}
