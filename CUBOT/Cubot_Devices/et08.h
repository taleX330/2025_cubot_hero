#include "stm32h7xx_hal.h"
#include "usart.h"
#include "driver_usart.h"
#ifndef ET08_H
#define ET08_H

#define ET08_rxBufferLengh 25	 //< et08���ջ��������ݳ���
#define ET08_dataLengh 25	 //< et08�������ݳ���
typedef struct { 
 struct {       
	 uint16_t ch0;       
	 uint16_t ch1;       
	 uint16_t ch2;      
	 uint16_t ch3;  
     uint8_t  sA; 	 
	 uint8_t  s1; 
	 uint8_t  s2;
     uint8_t  sD;	 
	 uint8_t  s1_last; 
	 uint8_t  s2_last; 
	}rc; 
 	uint8_t  isUnpackaging;  	 //< ����״̬��־λ����������в���ȡ����
	uint8_t  isOnline;
	uint32_t onlineCheckCnt;
}RC_Ctrl_ET; 
 

uint8_t ET08_callback(uint8_t * recBuffer, uint16_t len);
void ET08Init(RC_Ctrl_ET* RC_Ctl);
void ET08_online_protection(RC_Ctrl_ET* RC_Ctl);
void ET08_DataUnpack(RC_Ctrl_ET* rc_ctrl, uint8_t * recBuffer, uint16_t len );

extern RC_Ctrl_ET rc_Ctrl_et;
#endif
