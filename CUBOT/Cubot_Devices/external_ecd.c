#include "external_ecd.h"
#include "fdcan.h"

/**
 * @brief 布瑞特编码器或Oid编码器的CAN数据发送缓存区
 * @note 编码器默认初始的CAN_ID为0x01
 */
CAN_TxBuffer_t txBuffer0x01forCAN = {
    .txHeader.Identifier = 0x01};


/**
 * @brief  布瑞特编码器或Oid编码器数据更新回调函数
 */
uint8_t EcdRxCallBack(CAN_RxBuffer_t bufferRx, BrtOidEcd_t *ecd)
{
    if (bufferRx.rxHeader.Identifier == 0x01)
    {
        ecd->raw_encoder = bufferRx.data[3] | bufferRx.data[4] << 8 | bufferRx.data[5] << 16 | bufferRx.data[6] << 24;
        ecd->treated_encoder = ecd->raw_encoder / 4;// 将4096分度转化为1024分度
    }
    return 0;
}

/**
 * @brief  将待发送数据填入CAN_TxBuffer_t并发送
 */
uint16_t EcdCanOutput(CAN_Instance_t can, BrtOidEcd_t *ecd)
{
    if(ecd->can_instruction == 0x01)
    {
        txBuffer0x01forCAN.data[0] = 0x04;
        txBuffer0x01forCAN.data[1] = 0x01;
        txBuffer0x01forCAN.data[2] = 0x01;
        txBuffer0x01forCAN.data[3] = 0x00;
        txBuffer0x01forCAN.data[4] = 0;
        txBuffer0x01forCAN.data[5] = 0;
        txBuffer0x01forCAN.data[6] = 0;
        txBuffer0x01forCAN.data[7] = 0;
    }
    if (can.canHandler == &hfdcan1)
    {
        CAN_Send(&can, &txBuffer0x01forCAN);
    }
    else if (can.canHandler == &hfdcan2)
    {
        CAN_Send(&can, &txBuffer0x01forCAN);
    }

    return 0;
}
