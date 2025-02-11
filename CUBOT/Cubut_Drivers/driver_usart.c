/**
 **********************************************************************************
 * @file       	driver_usart.c
 * @brief   	驱动层，串口管理器配置文件，用户回调重定义
 * @details  	主要包括构建串口管理器，提供串口初始化和用户回调重定义
 * @author      RyanJiao  any question please send mail to 1095981200@qq.com
 * @date        2024-07-24
 * @version     V1.2
 * @copyright    Copyright (c) 2021-2121  中国矿业大学CUBOT战队
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date        	<th>Version  	<th>Author    	<th>Description
 * <tr><td>2021-08-12  	<td>1.0      	<td>RyanJiao  	<td>创建初始版本
 * <tr><td>2024-04-12  	<td>1.2      	<td>EmberLuo  	<td>为接收回调增加DMA双缓冲
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                          How to use this driver
 ==============================================================================

    添加driver_can.h

    1. 调UARTx_Init() 将 句柄 和 用户定义的接收回调函数 拷贝至UART结构体  （回调函数中对接收到的数据进行 ID识别 和 合并解算）

    2. 用户编写 UART_RxBuffer，填入 目标缓存区地址 和 数据长度。

    3. 调用UART_Open() 传入 UART_Object 和 用户编写 UART_RxBuffer。

    4. 将 UART_Idle_Handler 添加到 stm32H7xx_it.c 的 USARTx_IRQHandler() 中，调用用户编写的同一个 UART_RxBuffer_t 。

    5. 应用层编写 UART_TxBuffer_t （发送缓存区结构体），填入待发送字节数组首地址和字节长度

    6. 调UART_Send()传入 UART设备结构体 和 UART_TxBuffer结构体，将数据发送出去

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
    对DMA中NDTR寄存器描述：
        This register can be written only
        when the stream is disabled. When the stream is enabled, this register is read-only,
        indicating the remaining data items to be transmitted. This register decrements after each
        DMA transfer.
        指明了DMA中待传输的剩余数据个数 每次DMA传输完成后自动减一
        参考手册中对idle空闲中断触发条件的描述：
 **********************************************************************************
 */
#include "driver_usart.h"
#include "usart.h"
//#include "dr16.h"

UART_Object uart1;
UART_Object uart2;
UART_Object uart3;
UART_Object uart4;
UART_Object uart5;
UART_Object uart6;

/**
 * @brief 串口初始化，将句柄和接收回调拷贝至串口结构体
 *
 * @param h_usart
 * @param rxIdleCallback
 */
void UARTx_Init(UART_HandleTypeDef *h_usart, UART_RxIdleCallback rxIdleCallback)
{
	__HAL_UART_CLEAR_IDLEFLAG(h_usart);
    // 初始化uart1
    if (h_usart->Instance == USART1) {
        uart1.Handle         = h_usart;
        uart1.RxIdleCallback = rxIdleCallback;
    }

    // 初始化uart2
    if (h_usart->Instance == USART2) {
        uart2.Handle         = h_usart;
        uart2.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // 初始化uart3
    if (h_usart->Instance == USART3) {
        uart3.Handle         = h_usart;
        uart3.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // 初始化uart4
    if (h_usart->Instance == UART4) {
        uart4.Handle         = h_usart;
        uart4.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // 初始化uart5
    if (h_usart->Instance == UART5) {
        uart5.Handle         = h_usart;
        uart5.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // 初始化uart6
    if (h_usart->Instance == USART6) {
        uart6.Handle         = h_usart;
        uart6.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }
}

/**
 * @brief  串口调用dma发送数据，数据需拆分为字节
 */
uint32_t UART_Send(UART_Object *uart, UART_TxBuffer_t *txBuffer)
{
    return HAL_UART_Transmit_DMA(uart->Handle, txBuffer->Data, txBuffer->Size);
}

/**
 * @brief  串口管理器结构体参数已经预先填写好的串口设备初始化
 */
void UART_Receive_DMA(UART_Object *uart, UART_RxBuffer_t *rx_buffer)
{
    HAL_UART_Receive_DMA(uart->Handle, rx_buffer->Data, rx_buffer->Size);
}

/**
 * @brief  串口设备中断函数，执行中断DMA操作，调用串口用户回调函数
 */
void UART_Idle_Handler(UART_Object *uart, UART_RxBuffer_t *rx_buffer)
{
    assert_param(uart != NULL);

    uint16_t usart_rx_num;

    if ((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET)) {
        HAL_UART_DMAStop(uart->Handle);          //< 关闭DMA，防止解算过程中数据更新，造成丢失数据
        __HAL_UART_CLEAR_IDLEFLAG(uart->Handle); //< 清楚idle标志位，防止再次进入中断
        __HAL_UART_CLEAR_OREFLAG(uart->Handle);

        // 总计数减去未传输的数据个数，得到已经接收的数据个数
        usart_rx_num = rx_buffer->Size - ((DMA_Stream_TypeDef *)uart->Handle->hdmarx->Instance)->NDTR;

        // 如果发送方先启动，则接收方开始接收后是否需要匹配头字节和尾字节
        if ((*uart).RxIdleCallback != NULL)
            uart->RxIdleCallback(rx_buffer->Data, usart_rx_num); //<用户回调

        HAL_UART_DMAResume(uart->Handle);
        HAL_UART_Receive_DMA(uart->Handle, rx_buffer->Data, rx_buffer->Size);
    }
}
