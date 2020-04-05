/**
  ******************************************************************************
  * @file    filter.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-7-10
  * @brief   滤波 拟合 数据融合 计算相关
  ******************************************************************************
  */

/*--Include-start-------------------------------------------------------------*/
#include "filter.h"
#include "math.h"
/*--Include-end---------------------------------------------------------------*/


//-----Butterworth变量-----//
Butter_Parameter Butter_80HZ_Parameter_Acce={
  //200hz---80hz
1,     1.14298050254,   0.4128015980962,
0.638945525159,    1.277891050318,    0.638945525159
};

Butter_Parameter Butter_60HZ_Parameter_Acce={
  //200hz---60hz
1,   0.3695273773512,   0.1958157126558,
0.3913357725018,   0.7826715450035,   0.3913357725018
};

Butter_Parameter Butter_50HZ_Parameter_Acce={
  //200hz---50hz
1,-1.300707181133e-16,   0.1715728752538,
0.2065720838261,   0.4131441676523,   0.2065720838261,
};


Butter_Parameter Butter_30HZ_Parameter_Acce={
  //200hz---30hz
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Butter_20HZ_Parameter_Acce={
  //200hz---20hz
  1,    -1.14298050254,   0.4128015980962,
  0.06745527388907,   0.1349105477781,  0.06745527388907
};
Butter_Parameter Butter_15HZ_Parameter_Acce={
  //200hz---15hz
  1,   -1.348967745253,   0.5139818942197,
  0.04125353724172,  0.08250707448344,  0.04125353724172
};

Butter_Parameter Butter_10HZ_Parameter_Acce={
  //200hz---10hz
  1,   -1.561018075801,   0.6413515380576,
  0.02008336556421,  0.04016673112842,  0.02008336556421};
Butter_Parameter Butter_5HZ_Parameter_Acce={
  //200hz---5hz
  1,   -1.778631777825,   0.8008026466657,
  0.005542717210281,  0.01108543442056, 0.005542717210281
};

Butter_Parameter Butter_2HZ_Parameter_Acce={
  //200hz---2hz
  1,   -1.911197067426,   0.9149758348014,
  0.0009446918438402,  0.00188938368768,0.0009446918438402
};
Butter_Parameter Butter_1HZ_Parameter_Acce={
  //200hz---1hz
  1,   -1.955578240315,   0.9565436765112,
  0.000241359049042, 0.000482718098084, 0.000241359049042
};

//巴特沃斯滤波器
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter) 
{
	/* 获取最新x(n) */
	static int LPB_Cnt=0;
	Buffer->Input_Butter[2]=curr_input;
	if(LPB_Cnt>=100)
	{
	/* Butterworth滤波 */
		Buffer->Output_Butter[2]	= Parameter->b[0] * Buffer->Input_Butter [2]
															+ Parameter->b[1] * Buffer->Input_Butter [1]
															+ Parameter->b[2] * Buffer->Input_Butter [0]
															- Parameter->a[1] * Buffer->Output_Butter[1]
															- Parameter->a[2] * Buffer->Output_Butter[0];
	}
  else
	{
		Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
		LPB_Cnt++;
	}
	/* x(n) 序列保存 */
	Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
	Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
	Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
	Buffer->Output_Butter[1]=Buffer->Output_Butter[2];

	return Buffer->Output_Butter[2];
}
/*线性拟合 最小二乘法---------------------------------------------------------------------*/

//获得当前数组的平均值
float getArrayAverage(float * data, uint16_t num)
{
	double sum = 0;
	for (int i = 0; i < num; i++)
	{
		sum += data[i];
	}
	return sum / num;
}

/**
  * @brief  线性回归 计算
  *           
  * @note   所采用的线性回归方程
	*							n i=1∑(xi-xA)(yi-yA)				n i=1∑xiyi-nxAyA
	*					b=-------------------------- = -------------------
	*							n i=1∑(xi - xA)^2						n i=1∑xi^2-nxA^2
	*
	*					a=yA-bxA
	*
	*					y = bx + a
  *             
  * @param  xdata x方向数据列
	*					ydata y方向数据列
	*					res		【0】拟合斜率 【1】拟合截距
	*					num		序列中的元素数量
  * @retval void
  */
void LSMCurveFitting(float * xdata,float * ydata,float * res,uint16_t num)
{
	float xa = 0;
	float ya = 0;

	float bu = 0;
	float bd = 0;

	float b = 0;
	float a = 0;

	xa = getArrayAverage(xdata, num);
	ya = getArrayAverage(ydata, num);

	for (int i = 0; i < num; i++)
	{
		bu += (xdata[i] - xa) * (ydata[i] - ya);
		bd += (xdata[i] - xa) * (xdata[i] - xa);
	}

	b = bu / bd;

	a = ya - b * xa;

	res[0] = a;
	res[1] = b;
}

float K1 =0.02; 
float angle, angle_dot; 	
float Q_angle=0.001;// 过程噪声的协方差
float Q_gyro=0.003;//0.03 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
float R_angle=0.5;// 测量噪声的协方差 既测量偏差
float dt=0.005;//                 
char  C_0 = 1;
float Q_bias, Angle_err;
float PCt_0, PCt_1, E;
float K_0, K_1, t_0, t_1;
float Pdot[4] ={0,0,0,0};
float PP[2][2] = { { 1, 0 },{ 0, 1 } };

/**************************************************************************
函数功能：简易卡尔曼滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Kalman_Filter(float Accel,float Gyro)		
{
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分

	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // =先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
		
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	angle_dot   = Gyro - Q_bias;	 //输出值(后验估计)的微分=角速度
}

/**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度、角速度
返回  值：无
**************************************************************************/
void Yijielvbo(float angle_m, float gyro_m)
{
   angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * 0.005);
}

/*******************************************************************************
* Function Name  : kalman_filter
* Description    : 卡尔曼滤波
* Input          : 
* Output         : 
* Return         : 
*******************************************************************************/
float kalman_filter(kal_filter* k_flt, float input)
{
	/*量测更新，3组方程*/
	k_flt->input = input;
	k_flt->K = (k_flt->C_last)/(k_flt->C_last + k_flt->R);
	k_flt->X  = k_flt->X_last + k_flt->K * (k_flt->input - k_flt->X_last);
	k_flt->C =  (1-k_flt->K)*(k_flt->C_last);

	/*时间更新，2组方程*/
	k_flt->X_last = k_flt->X;										
	k_flt->C_last = k_flt->C + k_flt->Q;			
	
	return k_flt->X;
}







