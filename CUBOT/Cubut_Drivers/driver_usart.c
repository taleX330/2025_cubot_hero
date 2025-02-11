/**
 **********************************************************************************
 * @file       	driver_usart.c
 * @brief   	�����㣬���ڹ����������ļ����û��ص��ض���
 * @details  	��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
 * @author      RyanJiao  any question please send mail to 1095981200@qq.com
 * @date        2024-07-24
 * @version     V1.2
 * @copyright    Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * @par �޸���־:
 * <table>
 * <tr><th>Date        	<th>Version  	<th>Author    	<th>Description
 * <tr><td>2021-08-12  	<td>1.0      	<td>RyanJiao  	<td>������ʼ�汾
 * <tr><td>2024-04-12  	<td>1.2      	<td>EmberLuo  	<td>Ϊ���ջص�����DMA˫����
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                          How to use this driver
 ==============================================================================

    ���driver_can.h

    1. ��UARTx_Init() �� ��� �� �û�����Ľ��ջص����� ������UART�ṹ��  ���ص������жԽ��յ������ݽ��� IDʶ�� �� �ϲ����㣩

    2. �û���д UART_RxBuffer������ Ŀ�껺������ַ �� ���ݳ��ȡ�

    3. ����UART_Open() ���� UART_Object �� �û���д UART_RxBuffer��

    4. �� UART_Idle_Handler ��ӵ� stm32H7xx_it.c �� USARTx_IRQHandler() �У������û���д��ͬһ�� UART_RxBuffer_t ��

    5. Ӧ�ò��д UART_TxBuffer_t �����ͻ������ṹ�壩������������ֽ������׵�ַ���ֽڳ���

    6. ��UART_Send()���� UART�豸�ṹ�� �� UART_TxBuffer�ṹ�壬�����ݷ��ͳ�ȥ

 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
    ��DMA��NDTR�Ĵ���������
        This register can be written only
        when the stream is disabled. When the stream is enabled, this register is read-only,
        indicating the remaining data items to be transmitted. This register decrements after each
        DMA transfer.
        ָ����DMA�д������ʣ�����ݸ��� ÿ��DMA������ɺ��Զ���һ
        �ο��ֲ��ж�idle�����жϴ���������������
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
 * @brief ���ڳ�ʼ����������ͽ��ջص����������ڽṹ��
 *
 * @param h_usart
 * @param rxIdleCallback
 */
void UARTx_Init(UART_HandleTypeDef *h_usart, UART_RxIdleCallback rxIdleCallback)
{
	__HAL_UART_CLEAR_IDLEFLAG(h_usart);
    // ��ʼ��uart1
    if (h_usart->Instance == USART1) {
        uart1.Handle         = h_usart;
        uart1.RxIdleCallback = rxIdleCallback;
    }

    // ��ʼ��uart2
    if (h_usart->Instance == USART2) {
        uart2.Handle         = h_usart;
        uart2.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // ��ʼ��uart3
    if (h_usart->Instance == USART3) {
        uart3.Handle         = h_usart;
        uart3.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // ��ʼ��uart4
    if (h_usart->Instance == UART4) {
        uart4.Handle         = h_usart;
        uart4.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // ��ʼ��uart5
    if (h_usart->Instance == UART5) {
        uart5.Handle         = h_usart;
        uart5.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }

    // ��ʼ��uart6
    if (h_usart->Instance == USART6) {
        uart6.Handle         = h_usart;
        uart6.RxIdleCallback = rxIdleCallback;
        __HAL_UART_ENABLE_IT(h_usart, UART_IT_IDLE);
    }
}

/**
 * @brief  ���ڵ���dma�������ݣ���������Ϊ�ֽ�
 */
uint32_t UART_Send(UART_Object *uart, UART_TxBuffer_t *txBuffer)
{
    return HAL_UART_Transmit_DMA(uart->Handle, txBuffer->Data, txBuffer->Size);
}

/**
 * @brief  ���ڹ������ṹ������Ѿ�Ԥ����д�õĴ����豸��ʼ��
 */
void UART_Receive_DMA(UART_Object *uart, UART_RxBuffer_t *rx_buffer)
{
    HAL_UART_Receive_DMA(uart->Handle, rx_buffer->Data, rx_buffer->Size);
}

/**
 * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص�����
 */
void UART_Idle_Handler(UART_Object *uart, UART_RxBuffer_t *rx_buffer)
{
    assert_param(uart != NULL);

    uint16_t usart_rx_num;

    if ((__HAL_UART_GET_FLAG(uart->Handle, UART_FLAG_IDLE) != RESET)) {
        HAL_UART_DMAStop(uart->Handle);          //< �ر�DMA����ֹ������������ݸ��£���ɶ�ʧ����
        __HAL_UART_CLEAR_IDLEFLAG(uart->Handle); //< ���idle��־λ����ֹ�ٴν����ж�
        __HAL_UART_CLEAR_OREFLAG(uart->Handle);

        // �ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
        usart_rx_num = rx_buffer->Size - ((DMA_Stream_TypeDef *)uart->Handle->hdmarx->Instance)->NDTR;

        // ������ͷ�������������շ���ʼ���պ��Ƿ���Ҫƥ��ͷ�ֽں�β�ֽ�
        if ((*uart).RxIdleCallback != NULL)
            uart->RxIdleCallback(rx_buffer->Data, usart_rx_num); //<�û��ص�

        HAL_UART_DMAResume(uart->Handle);
        HAL_UART_Receive_DMA(uart->Handle, rx_buffer->Data, rx_buffer->Size);
    }
}
