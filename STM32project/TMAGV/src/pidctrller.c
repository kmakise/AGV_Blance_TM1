/**
  ******************************************************************************
  * @file    pidctrller.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-12-17
  * @brief   平衡车pid
  ******************************************************************************
  */
/*include file ---------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "pidctrller.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "comHandle.h"
#include "usart.h"
#include "stdio.h"
/*Golbal data space ----------------------------------------------------------*/
#define MOTOR_OUTPUT_MAX					999
#define MOTOR_SPEED_PID_KP				10.0
#define MOTOR_SPEED_PID_KI				0.05
#define MOTOR_SPEED_PID_KD				0.000
#define PI												3.1415926535897932384626433832795


IMUDataTypedef g_IMU = 
{
	.pitch = 0, .roll  = 0, .yaw   = 0,
	.aacx  = 0, .aacy  = 0, .aacz  = 0,
	.gyrox = 0, .gyroy = 0, .gyroz = 0,
	.temp  = 0,
};

const PIDParamBaseTypedef PARAM_PID = {//电机pid速度控制参数

	.kp = MOTOR_SPEED_PID_KP,				//比例权重
	.ki = MOTOR_SPEED_PID_KI,				//积分权重
	.kd = MOTOR_SPEED_PID_KD,				//微分权重
};

float 	g_TargetSpeed[2]	 = {0,0};		//目的速度
float 	g_MotorSpeed[2]		 = {0,0};		//当前速度
int16_t g_SpdCtrlOut[2]		 = {0,0};		//速度闭环输出控制
int16_t g_AngelSet				 = 0;				//偏置角度
int16_t g_AngelSpd				 = 0;				//设置角速度

//设定角度偏置
void setAngelOffset(int16_t angel)
{
	g_AngelSet = angel;
}
//设置角速度
void setAngelSpd(int16_t spd)
{
	g_AngelSpd = spd;
}

//设定目的速度
void SetTargetSpeed(int16_t s1,int16_t s2)
{
	g_TargetSpeed[0] = s1;
	g_TargetSpeed[1] = s2;
}
//速度值更新
void updataSpeed(void)
{
	int16_t etr[2];
	
	//ML1 TIM2 MR2 TIM3
	
	//获得编码器值
	etr[0] = TIM2->CNT;//M1
	TIM2->CNT = 0;
	etr[1] = -TIM3->CNT;//M2
	TIM3->CNT = 0;
	
	for(int i = 0;i < 2;i++)
	{
		float etrtemp = 0;
		
		//编码器脉冲计算轮角速度
		//360度常量 2*80 = 160t
		etrtemp = ((float)(etr[i] % 30000)) / (2.0 * 80.0);
		//角速度计算轮线速度
		etrtemp = etrtemp * (75 * PI);
		//写入线速度到全局
		g_MotorSpeed[i] = etrtemp * 10.0 * 0.25;
	}
}

//设定输出脉宽
void setMotorPWM(int16_t s1,int16_t s2)
{
	s2 = 0 - s2;
	//ML1 CH34 MR2 CH12
	if(s1 > 0)
	{
		TIM1->CCR3 = s1;
		TIM1->CCR4 = 0;
	}
	else
	{
		TIM1->CCR3 = 0;
		TIM1->CCR4 = 0 - s1;
	}
	
	if(s2 > 0)
	{
		TIM1->CCR1 = 0;
		TIM1->CCR2 = s2;
	}
	else
	{
		TIM1->CCR1 = 0 - s2;
		TIM1->CCR2 = 0;
	}
}



/**
  * @brief  MotorSpeedPidCtrl.
  * @note		PWM动态函数采用算法“位置式离散比例积分微分方程”
  *         Out = Kp[e(k)]+ki∑e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
  *         其中积分项和微分项受大扰动误差阀值范围生效标志位控制
  * @retval None
  */
void MotorSpeedPidCtrl(void)
{
	static PIDDateBaseTypedef pid[2] = {
		[0].de	= 0,	[1].de	= 0,
		[0].fe	= 0,  [1].fe	= 0,
		[0].de1 = 0,  [1].de1 = 0,
		[0].de2 = 0,  [1].de2 = 0,
		[0].out = 0,  [1].out = 0,
	};
	
	//进行PID控制计算
	for(int i = 0;i < 2;i++)
	{
		//计算当前误差并移动历史误差
		pid[i].de2	=  pid[i].de1;
		pid[i].de1	=  pid[i].de;
		
		//误差计算与一阶低通滤波器 这里0 是目的速度
		pid[i].de		=  pid[i].de * 0.7 + 0.3 * (g_MotorSpeed[i] - 0);//g_TargetSpeed[i] - g_MotorSpeed[i];
		pid[i].fe	 +=  pid[i].de - g_TargetSpeed[i] * 10;
		
		//积分幅度限制
		if(pid[i].fe > 10000)  pid[i].fe = 10000; 
		if(pid[i].fe <-10000)	 pid[i].fe =-10000; 
		
		
		//积分项的清除
		
		//pid[i]控制器核心方程
		pid[i].out = 	PARAM_PID.kp * pid[i].de 	+ 
									PARAM_PID.ki * pid[i].fe  + 
									PARAM_PID.kd * ( pid[i].de - 2 * pid[i].de1		+ pid[i].de2) * (pid[i].de < 100);
		//输出限制幅度
		pid[i].out = (pid[i].out > MOTOR_OUTPUT_MAX) ? MOTOR_OUTPUT_MAX : pid[i].out;
		pid[i].out = (pid[i].out <-MOTOR_OUTPUT_MAX) ?-MOTOR_OUTPUT_MAX : pid[i].out;
		
		//输出死区限制
	}
	//输出到电机控制
	//setMotorPWM(pid[0].out,pid[1].out);
	g_SpdCtrlOut[0] = pid[0].out;
	g_SpdCtrlOut[1] = pid[1].out;
	
	
	//uint8_t str[50];
	//sprintf((char *)str,"%d,%d,%d,%d\r\n",pid[0].out,pid[1].out,pid[2].out,pid[3].out);
	//USART_SendString(USART1,str);
}

//平台位姿数据更新
void AGVImuDataUpdate(void)
{
	//获取欧拉角
	if(mpu_dmp_get_data(&(g_IMU.pitch),&(g_IMU.roll),&(g_IMU.yaw))==0)
	{		
		g_IMU.temp = MPU_Get_Temperature();					//得到温度值
		MPU_Get_Accelerometer(&(g_IMU.aacx),&(g_IMU.aacy),&(g_IMU.aacz));	//得到加速度传感器数据
		MPU_Get_Gyroscope(&(g_IMU.gyrox),&(g_IMU.gyroy),&(g_IMU.gyroz));	//得到陀螺仪数据
	}
}


//平台平衡控制
void AGVBalancePdCtrl(void)
{
	int16_t vset = (g_SpdCtrlOut[0] + g_SpdCtrlOut[1]) / 2;
	
	float kp = 150.0;
	float kd = 0.15;
	
	//角度环
	float post = kp * (g_IMU.roll - g_AngelSet) + kd * g_IMU.gyrox;
	
	if(post > 900)
	{
		post = 900;
	}
	//post = 0;
	
	//转向环
	float kpz = 0.4;
	int 	turn;
	turn = kpz * (g_IMU.gyroz + g_AngelSpd * 10);
	
	//turn = 0;
	//vset = 0;
	
	int16_t out1 = post + vset + turn;
	int16_t out2 = post + vset - turn;
	
	setMotorPWM(out1,out2);
}

//打印测试信息
void testMSG(void)
{
  float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;			//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp = 0;
	
	uint8_t str[50];
	
	//获取欧拉角
	if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
	{		
		temp=MPU_Get_Temperature();					//得到温度值
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
		
		sprintf(str,"pitch = %3.2f,roll = %3.2f,yaw = %3.2f \r\n",pitch,roll,yaw);
		//sprintf(str,"gyrox = %4d,gyroy = %4d,gyroz = %4d \r\n",gyrox,gyroy,gyroz);
		Usart2TxStr(str);
	}
}

//pid控制器执行周期分频器
void PID_Divider(void)
{
	static uint32_t div = 0;
	static uint32_t div1 = 0;
	
	div1++;
	if(div1 >= 10)
	{
		div1 = 0;
		//平台位姿更新
		AGVImuDataUpdate();
		//平台平衡控制
		AGVBalancePdCtrl();
	}
	
	div++;
	if(div >= 100)
	{
		div = 0;
		//更新当前速度
		updataSpeed();
		//执行电机PID控制器
		MotorSpeedPidCtrl();
		//指令有效周期
	}
}

