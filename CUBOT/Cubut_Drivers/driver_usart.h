#ifndef _DRIVER_USART_H_
#define _DRIVER_USART_H_

#include "stm32h7xx_hal.h"

/**
 * @brief   �����û��ص���������
 * @param[in]  rx_buffer		���ڽ������ݵ������׵�ַ
 * @param[in]  size	    	���ڽ������ݵ����鳤��
 * @note �൱��������һ������ָ������
 */
typedef uint8_t (*UART_RxIdleCallback)(uint8_t *rx_buffer, uint16_t size);

/**
 * @brief	UART���ջ�����
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} UART_RxBuffer_t;

/**
 * @brief	UART���ͻ�����
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} UART_TxBuffer_t;

/**
 * @brief   ���ڽṹ��,�������,���ջص��ͻ�����
 */
typedef struct
{
    UART_HandleTypeDef *Handle;
    UART_RxIdleCallback RxIdleCallback;
} UART_Object;

/**
 * @brief   ���ڳ�ʼ����������ͽ��ջص����������ڽṹ��
 * @param[in]  h_usart		        ���ھ��
 * @param[in]  rxIdleCallback		���ջص�����
 */
void UARTx_Init(UART_HandleTypeDef *h_usart, UART_RxIdleCallback rxIdleCallback);

/**
 * @brief  ���ڵ���dma�������ݣ���������Ϊ�ֽ�
 * @param[in]  uart		    ���ڽṹ�壬����ʹ�õĴ��ں�
 * @param[in]  txBuffer		���ͻ�����.�û�����
 * @retval
 */
uint32_t UART_Send(UART_Object *uart, UART_TxBuffer_t *txBuffer);

/**
 * @brief  �����豸�жϺ�����ִ���ж�DMA���������ô����û��ص���������Ҫ������ӵ�stm32h7xx_it.c��
 * @param[in]  uart		    ���ڽṹ��, �������ھ���ͻص�����
 * @param[in]  rx_buffer		���ͻ�����, �û����塣ע�⣡ �˴���rxBufferӦ���� UART_Open�е���rxBufferһ��
 * @retval
 */
void UART_Idle_Handler(UART_Object *uart, UART_RxBuffer_t *rx_buffer);

/**
 * @brief  ���ڹ������ṹ������Ѿ�Ԥ����д�õĴ����豸��ʼ��
 * @param[in]  uart		    ���ڽṹ��, �������ھ��
 * @param[in]  rx_buffer		���ջ�����,�û�����
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
