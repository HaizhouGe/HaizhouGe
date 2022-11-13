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
//JY901S的数据采集使用的相关结构体的外部声明。
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
//定义命令指令
char ACCCALSW[5] = {0XFF,0XAA,0X01,0X01,0X00};//进入加速度校准模式
char SAVACALSW[5]= {0XFF,0XAA,0X00,0X00,0X00};//保存当前配置
//用串口2给JY模块发送指令
void sendcmd(char cmd[])
{
	char i;
	for(i=0;i<5;i++) UART2_Put_Char(cmd[i]);
}
//CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数，以对收到的数据进行解析。
void CopeSerial2Data(unsigned char ucData)
{
	static unsigned char ucRxBuffer[250];
	static unsigned char ucRxCnt = 0;
	
	ucRxBuffer[ucRxCnt++]=ucData;	//将收到的数据存入缓冲区中
	if (ucRxBuffer[0]!=0x55) //数据头不对，则重新开始寻找0x55数据头
	{
		ucRxCnt=0;
		return;//退出函数
	}
	//数据头正确，并且该函数累计调用了11次，则表明一个完整的数据信息接受完毕，然后通过switch进行对应处理。
	if (ucRxCnt<11) {return;}//数据不满11个，则返回（退出函数）
	else
	{
		switch(ucRxBuffer[1])//判断数据是哪种数据，然后将其拷贝到对应的结构体中。有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据。
		{
			case 0x50:	memcpy(&stcTime,&ucRxBuffer[2],8);break;//memcpy为编译器自带的内存拷贝函数，需引用"string.h"。作用是将接收缓冲区的字符拷贝到数据结构体里面，从而实现数据的解析。
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
		ucRxCnt=0;//清空缓存区（实际这里没清空，而是为覆盖之前数据做了准备）
	}
}
//转发串口1收到的数据给串口2（JY模块）
void CopeSerial1Data(unsigned char ucData)
{
	UART2_Put_Char(ucData);
}
//加速度校准函数
void AccCalibrate(void)
{
//	printf("\r\n正在进行加速度校准\r\n");
	sendcmd(ACCCALSW);delay_ms(100);//等待模块内部自动校准好，模块内部会自动计算需要一定的时间
	sendcmd(SAVACALSW);delay_ms(100);//保存当前配置
//	printf("\r\n加速度校准完成\r\n");
}
//取绝对值函数
u8 ABS(short int x)
{
	if(x<0)
	{
		x = -x;
	}
	return x;
}
//加速度数据获取（利用printf进行调试）
void JY901_Get_Accelerometer(void)
{
	aacx = (short int)((float)stcAcc.a[0]/32768*16*10);
	aacy = (short int)((float)stcAcc.a[1]/32768*16*10);
	aacz = (short int)((float)stcAcc.a[2]/32768*16*10);
}
//角度数据获取（利用printf进行调试）
void JY901_Get_Angle(void)
{
	roll  = (short int)((float)stcAngle.Angle[0]/32768*180);
	pitch = (short int)((float)stcAngle.Angle[1]/32768*180);
	yaw   = (short int)((float)stcAngle.Angle[2]/32768*180);
}
//位置式PID计算(用于位置（角度）环，其输出作为速度环的目标值)
int32_t  PID_PosLocCalc(_pid *pid,float Now_Point)//位置式
{
    float Now_Error,d_Error;
	Now_Error=pid->target_val - Now_Point;
	pid->integral+=Now_Error;
	//积分限幅
    if(pid->integral>4000) pid->integral=4000;
	else if(pid->integral<-4000) pid->integral=-4000;
	
	d_Error=Now_Error - pid->err_last;
    pid->err_last = Now_Error;
	pid->output_val=pid->Kp*Now_Error+
		            pid->Ki*pid->integral+
	                pid->Kd*d_Error;
	//速度目标值的限幅
	if(pid->output_val>200)  pid->output_val= 200;
	if(pid->output_val<-200) pid->output_val=-200;
	return pid->output_val;
}
//方向自动控制
void IMU_Direction_Ctrl(u8 direction,float yawtarget)
{
	static u8 j = 0;
	float movespeed_base;
	if(j == 0)//PID初始化
	{
		movespeed_base =  movespeed_left = movespeed_right;
		j=1;
		Yaw_PID_Loop.Kp = 20;
		Yaw_PID_Loop.Kd = 2;
		Yaw_PID_Loop.target_val = yawtarget;
	}
	Yaw_PID_Loop.output_val = PID_PosLocCalc(&Yaw_PID_Loop,yaw);//利用获取到的yaw角进行PID计算。
	if(direction == 1) Yaw_PID_Loop.output_val=-Yaw_PID_Loop.output_val;//倒退反向
	//死区设置
	if((Yaw_PID_Loop.output_val<1.0f) && (Yaw_PID_Loop.output_val>-1.0f)) Yaw_PID_Loop.output_val=0;
	//速度控制
	movespeed_left = movespeed_base - Yaw_PID_Loop.output_val;
	movespeed_right =movespeed_base + Yaw_PID_Loop.output_val;
	//速度限幅
	if(movespeed_left<0) movespeed_left=0;
	if(movespeed_right<0) movespeed_left=0;
	if(movespeed_left>200) movespeed_left=200;
	if(movespeed_right>200) movespeed_left=200;
}
