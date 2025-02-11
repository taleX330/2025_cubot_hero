#ifndef SUPERCAP_H
#define SUPERCAP_H
#include "driver_can.h"
#include "dr16.h"

typedef struct
{
	int16_t online_cnt;
	struct
	{
		float cap_voltage;
		float motor_current;
		float motor_power;
		uint8_t Supercap_Mode;
		uint8_t Supercap_Flag;
		uint8_t Supercap_Charge;
		uint8_t Supercap_Charge_mode;
		uint8_t voltage_flag;
		uint16_t consume_low_power_cnt;
		uint8_t consume_low_power ;
	} capState;
} Supercap_t;

extern Supercap_t heroSupercap;
uint8_t SupercapRxCallBack(CAN_RxBuffer_t bufferRx, Supercap_t *cap);
void SupercapControl(CAN_Instance_t can, Supercap_t *cap);

#endif
