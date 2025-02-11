#ifndef __EXTERNAL_ECD_H__
#define __EXTERNAL_ECD_H__

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "driver_can.h"

/**
 * @brief  �����غ�ŷ���ϱ����������ݺͲ���
 */
typedef struct
{
    uint16_t raw_encoder;      // ��ʼ������ֵ
    float treated_encoder;            // �����ı�����ֵ
    uint8_t can_instruction;      // ������CANָ��
    int16_t online_cnt;
} BrtOidEcd_t;

uint8_t EcdRxCallBack(CAN_RxBuffer_t bufferRx, BrtOidEcd_t *ecd);
uint16_t EcdCanOutput(CAN_Instance_t can, BrtOidEcd_t *ecd);

#endif
