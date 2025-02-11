#ifndef __EXTERNAL_ECD_H__
#define __EXTERNAL_ECD_H__

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "driver_can.h"

/**
 * @brief  布瑞特和欧艾迪编码器的数据和参数
 */
typedef struct
{
    uint16_t raw_encoder;      // 初始编码器值
    float treated_encoder;            // 换算后的编码器值
    uint8_t can_instruction;      // 编码器CAN指令
    int16_t online_cnt;
} BrtOidEcd_t;

uint8_t EcdRxCallBack(CAN_RxBuffer_t bufferRx, BrtOidEcd_t *ecd);
uint16_t EcdCanOutput(CAN_Instance_t can, BrtOidEcd_t *ecd);

#endif
