/**
  ******************************************************************************
  * @file    comHandle.c
  * @author  张东
  * @version V1.0.0
  * @date    2019-12-16
  * @brief   通信处理函数
  ******************************************************************************
  */
#include "comHandle.h"
#include "usart.h"
#include "pidctrller.h"
#include "stdio.h"

#define UART_COM_STAT   '<'	//串口通信开始字节
#define UART_COM_END		'>'	//串口通信结束字节

UartBufTypedef g_uartComData = {.Over = 0 ,.Len = 0};//串口接收缓冲区

//串口字节流写入到缓冲区
void UsartRxToBuf(uint8_t data)
{
	//上一次消息处理结束
	if(g_uartComData.Over == 0)
	{
		//开始
		if(data == UART_COM_STAT )
		{
			//长度清零
			g_uartComData.Len = 0;
		}
		//结束
		else if(data == UART_COM_END)
		{
			//接收结束
			g_uartComData.Over = 1;
		}
		//数据
		else
		{
			//写入数据
			g_uartComData.Buf[g_uartComData.Len] = data;
			//移动光标
			g_uartComData.Len = (g_uartComData.Len + 1) % 100;
		}
	}
}
//Usart2 data transmit
void Usart2Tx(uint8_t * data,uint8_t len)
{
	while(len--)
	{
		while((USART2->SR&0X40)==0);	
		USART2->DR = (uint8_t)*data++;
	}
}
//Usart2 Str transmit
void Usart2TxStr(uint8_t * data)
{
	while(*data)
	{
		while((USART2->SR&0X40)==0);	
		USART2->DR = (uint8_t)*data++;
	}
}

/*receive analysis ------------------------------------------------------*/

void MoveCtrl(uint8_t * cmd)
{
	//解析控制模式
	uint8_t ms1 = cmd[1];
	uint8_t ms2 = cmd[5];
		
	//解析控制数据
	uint16_t pwm1 = (cmd[2] - 0x30) * 100+ 
									(cmd[3] - 0x30) * 10+ 
									(cmd[4] - 0x30);
	
	uint16_t pwm2 = (cmd[6] - 0x30) * 100+ 
									(cmd[7] - 0x30) * 10+ 
									(cmd[8] - 0x30);
	
	//修改控制器目标数值
	int16_t tg1 = (ms1 == 'A') ? pwm1 : -pwm1;
	int16_t tg2 = (ms2 == 'A') ? pwm2 : -pwm2;
	
	uint8_t str[50];
	sprintf((char *)str,"P1 = %d,P2 = %d\r\n",tg1,tg2);
	Usart2TxStr(str);
	
	SetTargetSpeed(tg1,tg2);
}


void AngelCtrl(uint8_t * cmd)
{
	//解析控制模式
	uint8_t ms1 = cmd[1];
	
	//解析控制数据
	uint16_t spd  = (cmd[2] - 0x30) * 100+ 
									(cmd[3] - 0x30) * 10+ 
									(cmd[4] - 0x30);
	
	//修改控制器目标数值
	int16_t tg1 = (ms1 == 'A') ? spd : -spd;
	
	uint8_t str[50];
	sprintf((char *)str,"Spd = %d\r\n",tg1);
	Usart2TxStr(str);
	
	setAngelSpd(tg1);
	
}

//消息接收解析
void RxBufAnalysis(UartBufTypedef * buf)
{
	//小完整性被使能
	if(buf->Over == 1)
	{	
		//通信协议
		//开始 ‘<’ 结束 '>'
		//[0] 标识 [n] 数据
		switch(buf->Buf[0])
		{
			case 'M'://电机控制 <MSXXXSXXX> S方向 A，B
			{
				MoveCtrl(buf->Buf);
				break;
			}
			case 'V'://设置自转角速度
			{
				AngelCtrl(buf->Buf);
				break;
			}
		}
		//清除未完成标志位
		buf->Over = 0;
	}
}

//通信处理循环
void comHandle_Loop(void)
{
	//消息处理
	RxBufAnalysis(&g_uartComData);
}




	