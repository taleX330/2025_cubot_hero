/**
 **********************************************************************************
 * @file        driver_can.c
 * @brief       �����㣬CAN���������ļ�
 * @details     ��Ҫ����CAN��������ʼ����ָ���жϻص���������ͬCAN_ID�µ������շ�����
 * @date        2024-07-24
 * @version     V1.3
 * @copyright   Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * @par �޸���־:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2021-08-12   <td>1.0         <td>RyanJiao    <td>������ʼ�汾
 * <tr><td>2021-10-09   <td>1.0         <td>RyanJiao    <td>�淶��������ȷ����CAN_TxBuffer�ṹ
 * <tr><td>2024-06-04   <td>1.3         <td>EmberLuo    <td>����CAN_Instance_t�ṹ�壬�������շ��������ṹ��
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    ���driver_can.h

    1. ���� (*CAN_RxCpltCallback)(CAN_Instance_t *) ���͵��û��ص�

    2. ���� CANx_Init() �� ��� �� �û�����Ľ��ջص����� ������CAN�ṹ�壨�ص������жԽ��յ������ݽ���IDʶ��ͺϲ����㣩

    3. ���� CAN_Open() ����ʵ�����Ľṹ�壬����can�豸

    4. Ӧ�ò��д CAN_TxBuffer_t �����ͻ������ṹ�壩����������͵��ֽ����ݺ�Ŀ��ID

    5. ���� CAN_Send() ���� can�豸�ṹ�� �� TxBuffer�ṹ�壬�����ݷ��ͳ�ȥ

 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
    CAN�豸����

        1.������:

        2.FIFO����: ���ο����ϣ�https://blog.csdn.net/flydream0/article/details/8155942 ��

            ��bxCAN���յ����ģ��������������˺󣬻Ὣ���Ĵ洢��FIFO�С�ÿ���������鶼�����һ��FIFO��--> FIFO0��FIFO1
            ���FIFOΪ3��������ȣ�ÿ��FIFO������������ɣ�������ȫ��Ӳ����������ԼCPU��Դ������  FIFO0 --> 3 x mailbox
            �������֤�����ݵ�һ���ԡ�Ӧ�ó���ֻ��ͨ����ȡFIFO������䣬����ȡFIFO�������յ��ı��ġ�

            FIFO�������״̬����״̬���Һ�1״̬���Һ�2״̬���Һ�3״̬�����״̬

            �ڳ�ʼ��״̬ʱ��FIFO�Ǵ��ڿ�״̬�ģ������յ�һ������ʱ��������Ĵ洢��FIFO�ڲ�
            �������У���ʱ��FIFO��״̬��ɹҺ�1״̬�����Ӧ�ó���ȡ�������Ϣ����FIFO�ָ���״̬��
            ���ڼ���FIFO���ڹҺ�1״̬�����ѽ��յ�һ�����ģ���Ӧ�ó���û���ü�ȡ�߽��յ��ı��ģ�
            ��ʱ���ٴν��յ�һ�����ģ���ôFIFO����ɹҺ�2״̬���Դ����ƣ�����FIFO����3�����䣬ֻ�ܻ���3�����ģ�
            ��ˣ������յ�3�����ģ������ڼ�Ӧ�ó����δȡ���κα��ģ�ʱ����ʱFIFO������������һ������ʱ��
            ���޷��ٴ洢����ʱFIFO��������״̬��

            STM32����CAN������ص��ж���������
            �����жϣ�ÿ��bxCAN���յ�һ������ʱ����һ���жϡ�
            FIFO���жϣ���FIFO��ʱ�����洢��3������ʱ�������жϡ�
            FIFO����жϣ���FIFO���ʱ�������жϡ�
 **********************************************************************************
 */
#include "driver_can.h"
#include "rm_motor.h"
#include "Supercap.h"
#include "external_ecd.h"
#include "holder_task.h"

CAN_Instance_t can1 = {
    .txBuffer.txHeader =
        {
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0x00,
        },
    .devicesList = {
        &(can1.devicesList),
        &(can1.devicesList),
    },
};

CAN_Instance_t can2 = {
    .txBuffer.txHeader =
        {
            .IdType              = FDCAN_STANDARD_ID,
            .TxFrameType         = FDCAN_DATA_FRAME,
            .DataLength          = FDCAN_DLC_BYTES_8,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch       = FDCAN_BRS_OFF,
            .FDFormat            = FDCAN_CLASSIC_CAN,
            .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
            .MessageMarker       = 0x00,
        },
    .devicesList = {
        &(can2.devicesList),
        &(can2.devicesList),
    },
};

/**
 * @brief  CAN��ʼ����������ͽ��ջص�������CAN�豸�ṹ��
 */
void CANx_Init(FDCAN_HandleTypeDef *h_can, CAN_RxCpltCallback rxCallback)
{
    //< ��ʼ��can1
    if (h_can->Instance == FDCAN1) {
        can1.canHandler  = h_can;
        can1.RxCallBackCAN = rxCallback;
    }

    //< ��ʼ��can2
    if (h_can->Instance == FDCAN2) {
        can2.canHandler  = h_can;
        can2.RxCallBackCAN = rxCallback;
    }
}

/**
 * @brief CAN�豸��ʼ�������ù�����Ϊ�գ�ʹ��fifo0���յ�����Ϣ�ж� ��ע���û��ص�
 */
uint8_t CAN_Open(CAN_Instance_t *can)
{
    FDCAN_FilterTypeDef filter[2]; //< �����ֲ����� can�������ṹ��

    filter[0].IdType       = FDCAN_STANDARD_ID;       //< id����Ϊ��׼id
    filter[0].FilterIndex  = 0;                       //< ��ֵɸѡ���ı�ţ���׼idѡ��0-127
    filter[0].FilterType   = FDCAN_FILTER_MASK;       //< ���ù���ģʽΪ����ģʽ/*classic filter*/
    filter[0].FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //< ���������˵����ݴ洢�� fifo0
    filter[0].FilterID1    = 0x000;
    filter[0].FilterID2    = 0x000;                                   /* Recive all */
    if (HAL_FDCAN_ConfigFilter(can->canHandler, &filter[0]) != HAL_OK) //< ���ù�����
        return 1;

    filter[1].IdType       = FDCAN_STANDARD_ID;       //< id����Ϊ��׼id
    filter[1].FilterIndex  = 1;                       //< ��ֵɸѡ���ı�ţ���׼idѡ��0-127
    filter[1].FilterType   = FDCAN_FILTER_MASK;       //< ���ù���ģʽΪ����ģʽ/*classic filter*/
    filter[1].FilterConfig = FDCAN_FILTER_TO_RXFIFO1; //< ���������˵����ݴ洢�� fifo1
    filter[1].FilterID1    = 0x000;
    filter[1].FilterID2    = 0x000;                                   /* Recive all */
    if (HAL_FDCAN_ConfigFilter(can->canHandler, &filter[1]) != HAL_OK) //< ���ù�����
        return 1;

    HAL_FDCAN_ConfigGlobalFilter(can->canHandler, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_Start(can->canHandler); //< ʹ��can

    HAL_FDCAN_ActivateNotification(can->canHandler, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // ʹ��fifo0���յ�����Ϣ�ж�
    HAL_FDCAN_ActivateNotification(can->canHandler, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // ʹ��fifo1���յ�����Ϣ�ж�

    return 0;
}

/**
 * @brief CAN���ͺ���, ��CAN_Object�µ�txBuffer�е�data���ͳ�ȥ
 */
uint8_t CAN_Send(CAN_Instance_t *can, CAN_TxBuffer_t *bufferTx)
{
    can->txBuffer.txHeader.Identifier = bufferTx->txHeader.Identifier;
    can->txBuffer.txHeader.IdType     = bufferTx->txHeader.IdType;
    can->txBuffer.txHeader.DataLength = bufferTx->txHeader.DataLength;
    if (HAL_FDCAN_AddMessageToTxFifoQ(can->canHandler, &can->txBuffer.txHeader, bufferTx->data) != HAL_OK) {
        return 0;
    } else {
        return 1;
    }
}

/**
 * @brief CAN�豸�������ص�
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h_can, uint32_t RxFifo0ITs)
{
    if (h_can->Instance == FDCAN1) {
        if (HAL_FDCAN_GetRxMessage(h_can, FDCAN_RX_FIFO0, &(can1.rxBuffer.rxHeader), can1.rxBuffer.data) != HAL_ERROR) {
            can1.RxCallBackCAN(&can1);
        }
    }

    if (h_can->Instance == FDCAN2) {
        if (HAL_FDCAN_GetRxMessage(h_can, FDCAN_RX_FIFO0, &(can2.rxBuffer.rxHeader), can2.rxBuffer.data) != HAL_ERROR) {
            can2.RxCallBackCAN(&can2);
        }
    }
}

/**
 * @brief CAN�豸�������ص�
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *h_can, uint32_t RxFifo1ITs)
{

    if (h_can->Instance == FDCAN1) {
        if (HAL_FDCAN_GetRxMessage(h_can, FDCAN_RX_FIFO1, &(can1.rxBuffer.rxHeader), can1.rxBuffer.data) != HAL_ERROR) {
            can1.RxCallBackCAN(&can1);
        }
    }

    if (h_can->Instance == FDCAN2) {
        if (HAL_FDCAN_GetRxMessage(h_can, FDCAN_RX_FIFO1, &(can2.rxBuffer.rxHeader), can2.rxBuffer.data) != HAL_ERROR) {
            can2.RxCallBackCAN(&can2);
        }
    }
}

/**
 * @brief  CAN1�����жϻص�
 */
uint8_t CAN1_rxCallBack(CAN_Instance_t *canObject)
{
    MotorRxCallback(canObject);
    SupercapRxCallBack(canObject->rxBuffer, &heroSupercap);
    return 0;
}
/**
 * @brief  CAN2�����жϻص�
 */
uint8_t CAN2_rxCallBack(CAN_Instance_t *canObject)
{
    MotorRxCallback(canObject);
	DMiao_CanUpdata(&heroHolder.yaw.m4310);
    return 0;
}
