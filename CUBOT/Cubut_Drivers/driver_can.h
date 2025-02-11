#ifndef _DRIVER_CAN_H_
#define _DRIVER_CAN_H_

#include "stm32h7xx_hal.h"
#include "linux_list.h"

/**
 * @brief	CAN�豸��ö��
 */
typedef enum {
    CAN1 = 0x01U,
    CAN2 = 0x02U
} CanNumber;

/**
 * @brief	CAN���ջ�����
 */
typedef struct
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
} CAN_RxBuffer_t;

/**
 * @brief	CAN���ͻ�����
 */
typedef struct
{
    FDCAN_TxHeaderTypeDef txHeader;
    uint8_t data[8];
} CAN_TxBuffer_t;

/**
 * @brief	CAN�豸ʵ���ṹ��
 */
typedef struct _CAN_Instance_t {
    FDCAN_HandleTypeDef *canHandler;                    // CAN���
    list_t devicesList;                                 // CAN�豸����
    CAN_RxBuffer_t rxBuffer;                            // ���ջ������ṹ��
    CAN_TxBuffer_t txBuffer;                            // ���ͻ������ṹ��
    uint8_t (*RxCallBackCAN)(struct _CAN_Instance_t *); // ���ջص�����
} CAN_Instance_t;

/**
 * @brief   CAN�û��ص�����
 */
typedef uint8_t (*CAN_RxCpltCallback)(CAN_Instance_t *);

/**
 * @brief  CAN��ʼ����������ͽ��ջص�������CAN�ṹ��
 * @param[in]  h_can		CAN���
 */
void CANx_Init(FDCAN_HandleTypeDef *h_can, CAN_RxCpltCallback rxCallback);

/**
 * @brief ��CAN�豸�����ù�����Ϊ�գ�ʹ��fifo0���յ�����Ϣ�жϣ�ע���û��ص�
 * @param[in]	can	CAN�豸
 */
uint8_t CAN_Open(CAN_Instance_t *can);

/**
 * @brief ͨ��CAN�豸��������
 * @param[in] txBuffer CAN�ķ��ͻ�����
 *
 */
uint8_t CAN_Send(CAN_Instance_t *can, CAN_TxBuffer_t *txBuffer);

/**
 * @brief ͨ��CAN�豸��������
 *	@param[in] txBuffer CAN�ķ��ͻ�����
 *
 */
uint8_t CAN1_rxCallBack(CAN_Instance_t *canObject);
uint8_t CAN2_rxCallBack(CAN_Instance_t *canObject);

extern CAN_Instance_t can1;
extern CAN_Instance_t can2;

#endif
