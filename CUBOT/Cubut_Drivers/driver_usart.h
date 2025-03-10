#ifndef _DRIVER_USART_H_
#define _DRIVER_USART_H_

#include "stm32h7xx_hal.h"

/**
 * @brief   串口用户回调函数解算
 * @param[in]  rx_buffer		串口接受数据的数组首地址
 * @param[in]  size	    	串口接收数据的数组长度
 * @note 相当于声明了一个函数指针类型
 */
typedef uint8_t (*UART_RxIdleCallback)(uint8_t *rx_buffer, uint16_t size);

/**
 * @brief	UART接收缓冲区
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} UART_RxBuffer_t;

/**
 * @brief	UART发送缓冲区
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} UART_TxBuffer_t;

/**
 * @brief   串口结构体,包含句柄,接收回调和缓存区
 */
typedef struct
{
    UART_HandleTypeDef *Handle;
    UART_RxIdleCallback RxIdleCallback;
} UART_Object;

/**
 * @brief   串口初始化，将句柄和接收回调拷贝至串口结构体
 * @param[in]  h_usart		        串口句柄
 * @param[in]  rxIdleCallback		接收回调函数
 */
void UARTx_Init(UART_HandleTypeDef *h_usart, UART_RxIdleCallback rxIdleCallback);

/**
 * @brief  串口调用dma发送数据，数据需拆分为字节
 * @param[in]  uart		    串口结构体，标明使用的串口号
 * @param[in]  txBuffer		发送缓存区.用户定义
 * @retval
 */
uint32_t UART_Send(UART_Object *uart, UART_TxBuffer_t *txBuffer);

/**
 * @brief  串口设备中断函数，执行中断DMA操作，调用串口用户回调函数，需要将其添加到stm32h7xx_it.c中
 * @param[in]  uart		    串口结构体, 标明串口句柄和回调函数
 * @param[in]  rx_buffer		发送缓存区, 用户定义。注意！ 此处的rxBuffer应该与 UART_Open中调用rxBuffer一致
 * @retval
 */
void UART_Idle_Handler(UART_Object *uart, UART_RxBuffer_t *rx_buffer);

/**
 * @brief  串口管理器结构体参数已经预先填写好的串口设备初始化
 * @param[in]  uart		    串口结构体, 标明串口句柄
 * @param[in]  rx_buffer		接收缓存区,用户定义
 * @retval
 */
void UART_Receive_DMA(UART_Object *uart, UART_RxBuffer_t *rx_buffer);

void USART_Idle_Handler_DoubleBuffer(UART_Object *uart, uint8_t *rx0_buf, uint8_t *rx1_buf, DMA_HandleTypeDef *hdma, DMA_Stream_TypeDef *DMAx_Streamx);
void USART1_DoubleBuffer_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num, UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma);

extern UART_Object uart1;
extern UART_Object uart2;
extern UART_Object uart3;
extern UART_Object uart4;
extern UART_Object uart5;
extern UART_Object uart6;

#endif
