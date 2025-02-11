#include "brain.h"
#include "string.h"
#include "mpu6050.h"
#include "dr16.h"
#include "hardware_config.h"
#include "driver_timer.h"
#include "shoot_task.h"

uint8_t Brain_recData[Brain_rxBufferLengh] __attribute__((at(0x24018000)));      // 机器人接收上位机的
uint8_t RobotToBrainBuffer[Brain_txBufferLengh] __attribute__((at(0x24002160))); // 机器人给上位机发
/**
 * @brief  创建上位机串口接收缓存区数据结构
 */
UART_RxBuffer_t uart2_rxbuffer = {
    .Data = Brain_recData,
    .Size = Brain_rxBufferLengh};
/**
 * @brief  创建上位机串口发送缓存区数据结构
 */
UART_TxBuffer_t uart2_txbuffer = {
    .Data = RobotToBrainBuffer,
    .Size = Brain_txBufferLengh};

CubotBrain_t cubotBrain;
	
/**
 * @brief  下位机向上位机发送时间戳以及四元数
 */
int16_t tmp0, tmp1, tmp2, tmp3, ThisSecond;
int32_t tmpYaw;
static void BrainRobotToBrain(Holder_t *holder)
{
    ThisSecond++; // 给每次的发送编号
    tmp0                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmp1                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmp2                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmp3                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmpYaw                 = 0;
    RobotToBrainBuffer[0]  = 0xAA;
    RobotToBrainBuffer[1]  = 0x07;              // Type，固定为0x07
    RobotToBrainBuffer[2]  = 0x01;              // coreID，目前固定为0x01
    RobotToBrainBuffer[3]  = (ThisSecond >> 8); // 索引，int16_t型
    RobotToBrainBuffer[4]  = (ThisSecond & 0xff);
    RobotToBrainBuffer[5]  = (tim14.clock_time >> 24); // 定时器时间，int32_t型
    RobotToBrainBuffer[6]  = ((tim14.clock_time >> 16) & 0xff);
    RobotToBrainBuffer[7]  = ((tim14.clock_time >> 8) & 0xff);
    RobotToBrainBuffer[8]  = ((tim14.clock_time & 0xff));
    RobotToBrainBuffer[9]  = 0;
    RobotToBrainBuffer[10] = tmp0 & 0xFF; // 四元数q0，float型                                                                                                                 
    RobotToBrainBuffer[11] = tmp0 >> 8;
    RobotToBrainBuffer[12] = tmp1 & 0xFF; // 四元数q1，float型
    RobotToBrainBuffer[13] = tmp1 >> 8;
    RobotToBrainBuffer[14] = tmp2 & 0xFF; // 四元数q2，float型
    RobotToBrainBuffer[15] = tmp2 >> 8;
    RobotToBrainBuffer[16] = tmp3 & 0xFF; // 四元数q3，float型
    RobotToBrainBuffer[17] = tmp3 >> 8;
    RobotToBrainBuffer[18] = tmpYaw >> 24; // yaw编码器角度值
    RobotToBrainBuffer[19] = (tmpYaw >> 16) & 0xFF;
    RobotToBrainBuffer[20] = (tmpYaw >> 8) & 0xFF;
    RobotToBrainBuffer[21] = 0xDD;
    UART_Send(&uart2, &uart2_txbuffer);
}
/**
 * @brief  
 */
static void VisionTargetAngleSet(CubotBrain_t *brain, Holder_t *holder)
{
	if(cubotBrain.visionFlag.open == 1)
	{
		if (brain->visionAngleAdd.yaw_add != 0)
			holder->yaw.target_angle += holder->yaw.vision_sens * brain->visionAngleAdd.pitch_add;
		else
			holder->yaw.target_angle = holder->yaw.angle;
		if (brain->visionAngleAdd.pitch_add != 0)
			holder->pitch.target_angle += holder->pitch.vision_sens * brain->visionAngleAdd.pitch_add;
		else
			holder->pitch.target_angle = holder->pitch.angle;
	}
}

/**
 * @brief      上位机数据拆分解算函数
 * @param[in]  brain      上位机数据结构体
 * @param[in]  holder     云台结构体
 * @param[in]  recBuffer  缓存区数组数据
 */
static void BrainDataUnpack(CubotBrain_t *brain, Holder_t *holder, uint8_t *recBuffer)
{
    if (recBuffer[0] == 0xAA && recBuffer[11] == 0xDD) 
	{
            tim14.vision_fps++;
            //< yaw偏转角
            if ((recBuffer[3] >> 6) == 0)
                brain->visionAngleAdd.yaw_add    = ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
            else if ((recBuffer[3] >> 6) == 1)
                brain->visionAngleAdd.yaw_add    = (-1) * ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
            //< pitch偏转角
            if ((recBuffer[5] >> 6) == 0)
                brain->visionAngleAdd.pitch_add  = ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100);
            else if ((recBuffer[5] >> 6) == 1)
                brain->visionAngleAdd.pitch_add  = (-1) * ((float)((recBuffer[5] & 0x3f) * 100 + recBuffer[6]) / 100);
            brain->fire = (recBuffer[8]);
    }
	VisionTargetAngleSet(brain, holder);
	BrainRobotToBrain(holder);
}
/**
 * @brief 串口2视觉回调函数
 *
 * @param recBuffer
 * @param len
 * @return uint8_t
 */
uint8_t Brain_Callback(uint8_t *recBuffer, uint16_t len)
{
    BrainDataUnpack(&cubotBrain, &heroHolder, recBuffer);
    return 0;
}


