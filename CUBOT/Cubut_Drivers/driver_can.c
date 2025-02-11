/**
 **********************************************************************************
 * @file        driver_can.c
 * @brief       驱动层，CAN外设配置文件
 * @details     主要包括CAN过滤器初始化，指定中断回调函数，不同CAN_ID下的数据收发函数
 * @date        2024-07-24
 * @version     V1.3
 * @copyright   Copyright (c) 2021-2121  中国矿业大学CUBOT战队
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2021-08-12   <td>1.0         <td>RyanJiao    <td>创建初始版本
 * <tr><td>2021-10-09   <td>1.0         <td>RyanJiao    <td>规范变量名，确定了CAN_TxBuffer结构
 * <tr><td>2024-06-04   <td>1.3         <td>EmberLuo    <td>创建CAN_Instance_t结构体，整合了收发缓存区结构体
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    添加driver_can.h

    1. 创建 (*CAN_RxCpltCallback)(CAN_Instance_t *) 类型的用户回调

    2. 调用 CANx_Init() 将 句柄 和 用户定义的接收回调函数 拷贝至CAN结构体（回调函数中对接收到的数据进行ID识别和合并解算）

    3. 调用 CAN_Open() 传入实例化的结构体，开启can设备

    4. 应用层编写 CAN_TxBuffer_t （发送缓存区结构体），填入待发送的字节数据和目标ID

    5. 调用 CAN_Send() 传入 can设备结构体 和 TxBuffer结构体，将数据发送出去

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
    CAN设备讲解

        1.过滤器:

        2.FIFO队列: （参考资料：https://blog.csdn.net/flydream0/article/details/8155942 ）

            当bxCAN接收到报文，经过过滤器过滤后，会将报文存储到FIFO中。每个过滤器组都会关联一个FIFO，--> FIFO0和FIFO1
            这个FIFO为3级邮箱深度（每个FIFO由三个邮箱组成），且完全由硬件来管理，节约CPU资源，简化了  FIFO0 --> 3 x mailbox
            软件并保证了数据的一致性。应用程序只能通过读取FIFO输出邮箱，来读取FIFO中最先收到的报文。

            FIFO共有五个状态：空状态，挂号1状态，挂号2状态，挂号3状态，溢出状态

            在初始化状态时，FIFO是处于空状态的，当接收到一个报文时，这个报文存储到FIFO内部
            的邮箱中，此时，FIFO的状态变成挂号1状态，如果应用程序取走这个消息，则FIFO恢复空状态。
            现在假设FIFO处于挂号1状态，即已接收到一个报文，且应用程序不没来得及取走接收到的报文，
            此时若再次接收到一个报文，那么FIFO将变成挂号2状态，以此类推，由于FIFO共有3个邮箱，只能缓存3个报文，
            因此，当接收到3个报文（假设期间应用程序从未取走任何报文）时，此时FIFO已满，若再来一个报文时，
            已无法再存储，此时FIFO将变成溢出状态。

            STM32中与CAN接收相关的中断有三个：
            接收中断：每当bxCAN接收到一个报文时产生一个中断。
            FIFO满中断：当FIFO满时，即存储了3个报文时产生的中断。
            FIFO溢出中断：当FIFO溢出时产生此中断。
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
 * @brief  CAN初始化，将句柄和接收回调拷贝至CAN设备结构体
 */
void CANx_Init(FDCAN_HandleTypeDef *h_can, CAN_RxCpltCallback rxCallback)
{
    //< 初始化can1
    if (h_can->Instance == FDCAN1) {
        can1.canHandler  = h_can;
        can1.RxCallBackCAN = rxCallback;
    }

    //< 初始化can2
    if (h_can->Instance == FDCAN2) {
        can2.canHandler  = h_can;
        can2.RxCallBackCAN = rxCallback;
    }
}

/**
 * @brief CAN设备初始化，配置过滤器为空，使能fifo0接收到新信息中断 ，注册用户回调
 */
uint8_t CAN_Open(CAN_Instance_t *can)
{
    FDCAN_FilterTypeDef filter[2]; //< 声明局部变量 can过滤器结构体

    filter[0].IdType       = FDCAN_STANDARD_ID;       //< id设置为标准id
    filter[0].FilterIndex  = 0;                       //< 设值筛选器的编号，标准id选择0-127
    filter[0].FilterType   = FDCAN_FILTER_MASK;       //< 设置工作模式为掩码模式/*classic filter*/
    filter[0].FilterConfig = FDCAN_FILTER_TO_RXFIFO0; //< 将经过过滤的数据存储到 fifo0
    filter[0].FilterID1    = 0x000;
    filter[0].FilterID2    = 0x000;                                   /* Recive all */
    if (HAL_FDCAN_ConfigFilter(can->canHandler, &filter[0]) != HAL_OK) //< 配置过滤器
        return 1;

    filter[1].IdType       = FDCAN_STANDARD_ID;       //< id设置为标准id
    filter[1].FilterIndex  = 1;                       //< 设值筛选器的编号，标准id选择0-127
    filter[1].FilterType   = FDCAN_FILTER_MASK;       //< 设置工作模式为掩码模式/*classic filter*/
    filter[1].FilterConfig = FDCAN_FILTER_TO_RXFIFO1; //< 将经过过滤的数据存储到 fifo1
    filter[1].FilterID1    = 0x000;
    filter[1].FilterID2    = 0x000;                                   /* Recive all */
    if (HAL_FDCAN_ConfigFilter(can->canHandler, &filter[1]) != HAL_OK) //< 配置过滤器
        return 1;

    HAL_FDCAN_ConfigGlobalFilter(can->canHandler, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_Start(can->canHandler); //< 使能can

    HAL_FDCAN_ActivateNotification(can->canHandler, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0); // 使能fifo0接收到新信息中断
    HAL_FDCAN_ActivateNotification(can->canHandler, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0); // 使能fifo1接收到新信息中断

    return 0;
}

/**
 * @brief CAN发送函数, 将CAN_Object下的txBuffer中的data发送出去
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
 * @brief CAN设备弱函数回调
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
 * @brief CAN设备弱函数回调
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
 * @brief  CAN1接收中断回调
 */
uint8_t CAN1_rxCallBack(CAN_Instance_t *canObject)
{
    MotorRxCallback(canObject);
    SupercapRxCallBack(canObject->rxBuffer, &heroSupercap);
    return 0;
}
/**
 * @brief  CAN2接收中断回调
 */
uint8_t CAN2_rxCallBack(CAN_Instance_t *canObject)
{
    MotorRxCallback(canObject);
	DMiao_CanUpdata(&heroHolder.yaw.m4310);
    return 0;
}
