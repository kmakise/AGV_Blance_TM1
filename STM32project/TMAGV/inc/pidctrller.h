#ifndef __PIDCTRLLER_H
#define __PIDCTRLLER_H

#include "stm32f4xx.h"

/* Structure type definition -------------------------------------------------*/
typedef struct
{
	float			de ;					//各个电机当前误差值 difference error
	float			fe ;					//各个电机误差积分
	float			de1;					//各个电机历史误差1
	float			de2;					//各个电机历史误差2
	int				out;					//各个通道最终PWM输出
	
}PIDDateBaseTypedef;			//pid数据结构类型

typedef struct
{
	float			kp;						//比例权重
	float			ki;						//积分权重
	float			kd;						//微分权重
	
}PIDParamBaseTypedef;			//pid参数结构类型

typedef struct
{
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;			//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;
}IMUDataTypedef;


//设定角度偏置
void setAngelOffset(int16_t angel);
//设置角速度
void setAngelSpd(int16_t spd);
//设定目的速度
void SetTargetSpeed(int16_t s1,int16_t s2);
//设定输出脉宽
void setMotorPWM(int16_t s1,int16_t s2);
//设定指令有效周期 单位 百毫秒
void setCmdTimeOutVal(uint16_t num);
//pid控制器执行周期分频器
void PID_Divider(void);

#endif /*__PIDCTRLLER_H*/
