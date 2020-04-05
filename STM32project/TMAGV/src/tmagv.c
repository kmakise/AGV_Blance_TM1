/**
  ******************************************************************************
  * @file    tmagv.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-12-12
  * @brief   平衡车主程序
  ******************************************************************************
  */
	
/*--Include-start-------------------------------------------------------------*/
#include "tmagv.h"
#include "stm32f4xx.h"
#include "tim.h"
#include "usart.h"
#include "comHandle.h"
#include "pidctrller.h"
#include "IICcom.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "stdio.h"

void TMAGV_Setup(void)
{	
	uint8_t str[50];
	uint8_t temp = 1;
	
	//timer ETR and PWM start
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	//usart it config
	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(&huart2,UART_IT_IDLE);
	
	//IMU Init
	while(temp)
	{
		temp = mpu_dmp_init();
		sprintf(str,"IMU Init code : %d \r\n",temp);
		Usart2TxStr(str);
		HAL_Delay(100);
	}
	
	//set angel offset
	HAL_Delay(1000);
	setAngelOffset(21);
	
}

void TMAGV_Loop(void)
{
	comHandle_Loop();
}


void TMAGV_Interrupt(void)
{
	PID_Divider();
}









