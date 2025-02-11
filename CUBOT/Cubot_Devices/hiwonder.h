#ifndef _HIWONDER_H_
#define _HIWONDER_H_

#include "stm32h7xx_hal.h"
#include "driver_usart.h"
#include "usart.h"

#define HIWONDER_rxBufferLengh 10

typedef struct
{
	int16_t online_cnt;
} Servo_t;

extern UART_RxBuffer_t uart4_buffer;

void ServoMove(uint16_t id, int16_t position, uint16_t time, UART_HandleTypeDef *huart);
uint8_t Servo_Callback(uint8_t *recBuffer, uint16_t len);
#endif
