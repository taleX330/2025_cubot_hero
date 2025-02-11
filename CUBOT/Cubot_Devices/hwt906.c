#include "hwt906.h"

HWT906_t hwt906;
UART_RxBuffer_t uart4_buffer={
	.Data = Usart4_RxBuffer,
	.Size = 50
};
uint8_t Usart4_RxBuffer[50];
uint8_t Usart4_TxBuffer[50];

/**
  * @brief   维特串口陀螺仪回调函数，波特率921600
  * @param[in]
  */
void HWT906_Data_Deal(uint8_t *data_receive,HWT906_t* hwt_906)
{
	if(data_receive[0]==0x55 && data_receive[1] == 0x51)
	{
		hwt_906->Acc.Acc_x = (float)((int16_t)(data_receive[3]<<8|data_receive[2]))*32768*16*9.8f;
		hwt_906->Acc.Acc_y = (float)((int16_t)(data_receive[5]<<8|data_receive[4]))*32768*16*9.8f;
		hwt_906->Acc.Acc_z = (float)((int16_t)(data_receive[7]<<8|data_receive[6]))*32768*16*9.8f;
		hwt_906->temperature = (float)((int16_t)(data_receive[9]<<8|data_receive[8]))/100;
	}
	
	if(data_receive[11]==0x55 && data_receive[12] == 0x52)
	{
		hwt_906->Speed.Speed_x = (float)((int16_t)(data_receive[14]<<8|data_receive[13]))/32768*2000;
		hwt_906->Speed.Speed_y = (float)((int16_t)(data_receive[16]<<8|data_receive[15]))/32768*2000;
		hwt_906->Speed.Speed_z = (float)((int16_t)(data_receive[18]<<8|data_receive[17]))/32768*2000;
	}
	
	if(data_receive[22]==0x55 && data_receive[23] == 0x53)
	{
		hwt_906->angle.Angle_Roll 	= (float)((int16_t)(data_receive[25]<<8|data_receive[24]))/32768*180;
		hwt_906->angle.Angle_Pitch 	= (float)((int16_t)(data_receive[27]<<8|data_receive[26]))/32768*180;
		hwt_906->angle.Angle_Yaw 	= (float)((int16_t)(data_receive[29]<<8|data_receive[28]))/32768*180;
	}
	
	if(data_receive[33]==0x55 && data_receive[34]==0x59)
	{
		hwt_906->Quarternion.q0 = (float)((int16_t)(data_receive[36]<<8|data_receive[35]))/32768;
		hwt_906->Quarternion.q1 = (float)((int16_t)(data_receive[38]<<8|data_receive[37]))/32768;
		hwt_906->Quarternion.q2 = (float)((int16_t)(data_receive[40]<<8|data_receive[39]))/32768;
		hwt_906->Quarternion.q3 = (float)((int16_t)(data_receive[42]<<8|data_receive[41]))/32768;
	}
}

