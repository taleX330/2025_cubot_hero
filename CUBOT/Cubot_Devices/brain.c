#include "brain.h"
#include "string.h"
#include "mpu6050.h"
#include "dr16.h"
#include "hardware_config.h"
#include "driver_timer.h"
#include "shoot_task.h"

uint8_t Brain_recData[Brain_rxBufferLengh] __attribute__((at(0x24018000)));      // �����˽�����λ����
uint8_t RobotToBrainBuffer[Brain_txBufferLengh] __attribute__((at(0x24002160))); // �����˸���λ����
/**
 * @brief  ������λ�����ڽ��ջ��������ݽṹ
 */
UART_RxBuffer_t uart2_rxbuffer = {
    .Data = Brain_recData,
    .Size = Brain_rxBufferLengh};
/**
 * @brief  ������λ�����ڷ��ͻ��������ݽṹ
 */
UART_TxBuffer_t uart2_txbuffer = {
    .Data = RobotToBrainBuffer,
    .Size = Brain_txBufferLengh};

CubotBrain_t cubotBrain;
	
/**
 * @brief  ��λ������λ������ʱ����Լ���Ԫ��
 */
int16_t tmp0, tmp1, tmp2, tmp3, ThisSecond;
int32_t tmpYaw;
static void BrainRobotToBrain(Holder_t *holder)
{
    ThisSecond++; // ��ÿ�εķ��ͱ��
    tmp0                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmp1                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmp2                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmp3                   = (int16_t)(holder->holderAttitude.q[0] * 30000);
    tmpYaw                 = 0;
    RobotToBrainBuffer[0]  = 0xAA;
    RobotToBrainBuffer[1]  = 0x07;              // Type���̶�Ϊ0x07
    RobotToBrainBuffer[2]  = 0x01;              // coreID��Ŀǰ�̶�Ϊ0x01
    RobotToBrainBuffer[3]  = (ThisSecond >> 8); // ������int16_t��
    RobotToBrainBuffer[4]  = (ThisSecond & 0xff);
    RobotToBrainBuffer[5]  = (tim14.clock_time >> 24); // ��ʱ��ʱ�䣬int32_t��
    RobotToBrainBuffer[6]  = ((tim14.clock_time >> 16) & 0xff);
    RobotToBrainBuffer[7]  = ((tim14.clock_time >> 8) & 0xff);
    RobotToBrainBuffer[8]  = ((tim14.clock_time & 0xff));
    RobotToBrainBuffer[9]  = 0;
    RobotToBrainBuffer[10] = tmp0 & 0xFF; // ��Ԫ��q0��float��                                                                                                                 
    RobotToBrainBuffer[11] = tmp0 >> 8;
    RobotToBrainBuffer[12] = tmp1 & 0xFF; // ��Ԫ��q1��float��
    RobotToBrainBuffer[13] = tmp1 >> 8;
    RobotToBrainBuffer[14] = tmp2 & 0xFF; // ��Ԫ��q2��float��
    RobotToBrainBuffer[15] = tmp2 >> 8;
    RobotToBrainBuffer[16] = tmp3 & 0xFF; // ��Ԫ��q3��float��
    RobotToBrainBuffer[17] = tmp3 >> 8;
    RobotToBrainBuffer[18] = tmpYaw >> 24; // yaw�������Ƕ�ֵ
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
 * @brief      ��λ�����ݲ�ֽ��㺯��
 * @param[in]  brain      ��λ�����ݽṹ��
 * @param[in]  holder     ��̨�ṹ��
 * @param[in]  recBuffer  ��������������
 */
static void BrainDataUnpack(CubotBrain_t *brain, Holder_t *holder, uint8_t *recBuffer)
{
    if (recBuffer[0] == 0xAA && recBuffer[11] == 0xDD) 
	{
            tim14.vision_fps++;
            //< yawƫת��
            if ((recBuffer[3] >> 6) == 0)
                brain->visionAngleAdd.yaw_add    = ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
            else if ((recBuffer[3] >> 6) == 1)
                brain->visionAngleAdd.yaw_add    = (-1) * ((float)((recBuffer[3] & 0x3f) * 100 + recBuffer[4]) / 100);
            //< pitchƫת��
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
 * @brief ����2�Ӿ��ص�����
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


