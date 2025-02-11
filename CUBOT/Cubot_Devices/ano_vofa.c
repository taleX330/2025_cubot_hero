#include "ano_vofa.h"
#include <stdarg.h>
#include <stdio.h>

uint8_t Usart5_RxBuffer[128];
UART_RxBuffer_t uart5_buffer = {
    .Data = Usart5_RxBuffer,
    .Size = 128};
uint8_t Usart5_TxBuffer_ANO[20];
uint8_t Usart5_TxBuffer_Vofa[128];

/*-------------------------------匿名上位机---------------------------------*/
void ANO_Send_Up_Computer(UART_HandleTypeDef *uartx, int16_t user1, int16_t user2, int16_t user3, int16_t user4, int16_t user5, int16_t user6)
{
    uint8_t _cnt                = 0;
    Usart5_TxBuffer_ANO[_cnt++] = 0xAA;
    Usart5_TxBuffer_ANO[_cnt++] = 0xFF;
    Usart5_TxBuffer_ANO[_cnt++] = 0xF1;
    Usart5_TxBuffer_ANO[_cnt++] = 12;

    Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user1);
    Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user1);

    Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user2);
    Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user2);

    Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user3);
    Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user3);

    Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user4);
    Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user4);

    Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user5);
    Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user5);

    Usart5_TxBuffer_ANO[_cnt++] = BYTE0(user6);
    Usart5_TxBuffer_ANO[_cnt++] = BYTE1(user6);

    uint8_t sc = 0;
    uint8_t ac = 0;

    for (uint8_t i = 0; i < (Usart5_TxBuffer_ANO[3] + 4); i++) {
        sc += Usart5_TxBuffer_ANO[i];
        ac += sc;
    }
    Usart5_TxBuffer_ANO[_cnt++] = sc;
    Usart5_TxBuffer_ANO[_cnt++] = ac;

    HAL_UART_Transmit_DMA(uartx, Usart5_TxBuffer_ANO, _cnt);
}
/*------------------------------- VOFA+ ---------------------------------*/
void UsartDmaPrintf(UART_HandleTypeDef *uartx, const char *format, ...)
{
    uint16_t len;
    va_list args;
    va_start(args, format);
    len = vsnprintf((char *)Usart5_TxBuffer_Vofa, sizeof(Usart5_TxBuffer_Vofa) + 1, (char *)format, args);
    va_end(args);
    HAL_UART_Transmit_DMA(uartx, Usart5_TxBuffer_Vofa, len);
}
uint8_t vofa_Callback(uint8_t *recBuffer, uint16_t len)
{
   return 0;
}
