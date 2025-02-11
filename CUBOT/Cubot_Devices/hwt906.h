#ifndef _HWT906_H_
#define _HWT906_H_

#include "stm32h7xx_hal.h"
#include "driver_usart.h"


#define RtA 	57.324841f		//180/3.1415 角度制转化为弧度制	


typedef struct
{
	struct
	{
		float Acc_x;
		float Acc_y;
		float Acc_z;
		uint8_t CRC_Acc;
	}Acc;
	
	struct
	{
		float Speed_x;
		float Speed_y;
		float Speed_z;
		uint8_t CRC_Speed;
	}Speed;
	
	struct
	{
		float Angle_Roll;
		float Angle_Pitch;
		float Angle_Yaw;
		uint8_t CRC_Angle;
	}angle;
	
	struct//云台姿态角四元数
	{
		float q0,q1,q2,q3;
		uint8_t CRC_Q;
		
	}Quarternion;
	
	float temperature;
	
}HWT906_t;

extern HWT906_t hwt906;

void HWT906_Data_Deal(uint8_t *data_receive,HWT906_t* hwt906);


extern uint8_t Usart4_RxBuffer[50];
extern uint8_t Usart4_TxBuffer[50];
extern UART_RxBuffer_t uart4_buffer;

#endif
