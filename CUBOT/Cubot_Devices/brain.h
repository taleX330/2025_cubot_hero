#ifndef BRAIN_H__
#define BRAIN_H__
#include "stm32h7xx.h"
#include "driver_usart.h"
#include "holder_task.h"

#define BRAIN_TO_ROBOT_CMD  1
#define Brain_rxBufferLengh 50
#define Brain_txBufferLengh 22

/**
 * @brief  上位机大脑结构体
 */
typedef struct
{
    struct
    {
        float yaw_add;
        float pitch_add;
    } visionAngleAdd;
    struct
    {
        uint8_t open;                // 0关1开
        uint8_t open_last;           // 上一次自瞄开关
    } visionFlag;
    struct
    {
        int16_t pitch_move_times; // pitch手动移动的次数
        int16_t times_symbol;     // pitch手动移动次数的符号部分
        int16_t times_figure;     // pitch手动移动次数的数字部分
    } pitchMove;
	uint8_t fire;
	int16_t online_cnt;
} CubotBrain_t;

uint8_t Brain_Callback(uint8_t *recBuffer, uint16_t len);
void BrainRobotToBrain(Holder_t *holder);

extern CubotBrain_t cubotBrain;
extern UART_RxBuffer_t uart2_rxbuffer;

#endif
