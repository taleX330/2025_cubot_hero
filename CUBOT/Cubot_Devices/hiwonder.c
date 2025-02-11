#include "hiwonder.h"
#include "user_lib.h"

uint8_t hiwonderBuffer[HIWONDER_rxBufferLengh];

UART_TxBuffer_t uart4_txbuffer = {
    .Data = hiwonderBuffer,
    .Size = HIWONDER_rxBufferLengh};

Servo_t hiwonderServo;
/**
 * @brief 計算校驗和Checksum
 *
 * @param Checksum = ~ (ID + Length + Cmd+ Prm1 + ... PrmN)若括号内的计算和超出255,则取最低的一个字节
 */
static uint8_t HiwonderCheckSum(uint8_t buf[])
{
	uint8_t i;
	uint16_t temp = 0;
	for (i = 2; i < buf[3] + 2; i++) 
	{
		temp += buf[i];
	}
	temp = ~temp;//~取反
	i = (uint8_t)temp;
	return i;
}
/**
 * @brief 舵機位置控制
 *
 * @param position 角度值 0-1000對應0-270
 * @param time 轉到給定角度值時間 0-30000ms
 */
void ServoMove(uint16_t id, int16_t position, uint16_t time, UART_HandleTypeDef *huart)
{
	position          = LIMIT(position,0,1000);
	time              = LIMIT(time,1,30000);
	hiwonderBuffer[0] = 0x55;//幀頭
	hiwonderBuffer[1] = 0x55;//幀頭
	hiwonderBuffer[2] = id; 
	hiwonderBuffer[3] = 7;//數據長度
	hiwonderBuffer[4] = 1;//指令類型
	hiwonderBuffer[5] = (uint8_t)(position);
	hiwonderBuffer[6] = (uint8_t)(position>> 8);
	hiwonderBuffer[7] = (uint8_t)(time);
	hiwonderBuffer[8] = (uint8_t)(time>> 8);
	hiwonderBuffer[9] = HiwonderCheckSum(hiwonderBuffer);
	
	HAL_UART_Transmit_DMA(huart, hiwonderBuffer, HIWONDER_rxBufferLengh);
}

/**
 * @brief 串口4舵機回調函數
 *
 * @param recBuffer
 * @param len
 * @return uint8_t
 */
uint8_t Servo_Callback(uint8_t *recBuffer, uint16_t len)
{
    return 0;
}
