#ifndef _ANO_VOFA_H_
#define _ANO_VOFA_H_

#include "stm32h7xx_hal.h"
#include "driver_usart.h"
#include "usart.h"


/*-------------------------------匿名上位机---------------------------------*/
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
void ANO_Send_Up_Computer(UART_HandleTypeDef* uartx,
	int16_t user1,int16_t user2,int16_t user3,int16_t user4,int16_t user5,int16_t user6);
void UsartDmaPrintf(UART_HandleTypeDef* uartx, const char *format,...);
uint8_t vofa_Callback(uint8_t * recBuffer, uint16_t len);

extern UART_RxBuffer_t uart5_buffer;
extern uint8_t Usart5_TxBuffer_ANO[20]__attribute__((at(0x24020000))); // 匿名上位机
extern uint8_t Usart5_TxBuffer_Vofa[128]__attribute__((at(0x24020200))); // VOFA+


#endif
