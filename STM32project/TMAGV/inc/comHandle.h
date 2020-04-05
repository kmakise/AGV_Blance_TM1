#ifndef __COMHANDLE_H
#define __COMHANDLE_H

#include "stm32f4xx.h"
typedef struct
{
	uint8_t Buf[100];	//串口接收缓冲区
	uint8_t Over;			//串口接受检查
	uint8_t Len;				//串口接受长度
}UartBufTypedef;

//串口字节流写入到缓冲区
void UsartRxToBuf(uint8_t data);
//通信处理循环
void comHandle_Loop(void);
//Usart2 data transmit
void Usart2Tx(uint8_t * data,uint8_t len);
//Usart2 Str transmit
void Usart2TxStr(uint8_t * data);


#endif /*__COMHANDLE_H*/

