#ifndef __FILTER_H
#define __FILTER_H

#include "stm32f4xx.h"

typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
 const float a[3];
 const float b[3];
}Butter_Parameter;

/*一维kalman滤波结构体定义,A=1,B=0,H=1*/
typedef struct Kalman_filter
{
	float C_last;				    /*上次预测过程协方差矩阵 C(k|k-1)*/
	float X_last;				    /*系统状态预测矩阵，列矩阵*/
	
	float Q;						/*过程噪声协方差*/
	float R;						/*量测噪声协方差*/
	
	float K;						/*卡尔曼增益，列矩阵*/
	float X;						/*最优估计输出矩阵，列矩阵*/
	float C;						/*最优估计协方差矩阵C(k|k)*/
                            
	float input;				    /*量测值，即Z(k)*/
}
kal_filter;

//线性回归 拟合
void LSMCurveFitting(float * xdata,float * ydata,float * res,uint16_t num);

#endif /*__FILTER_H*/
