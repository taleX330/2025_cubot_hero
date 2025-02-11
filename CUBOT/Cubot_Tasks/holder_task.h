#ifndef _HOLDERTASK_H_
#define _HOLDERTASK_H_

#include "stm32h7xx_hal.h"
#include "rm_motor.h"
#include "pid.h"
#include "ins.h"

#define HOLDER_ENABLE 1
#define PITCHANGLETOROUNT 1000  //���Ƕ�ֵ�D�Q��늙CȦ��

typedef struct
{
    struct
    {
        uint8_t reset_done;    // ��̨��λ�ɹ�
        uint8_t micro_move;    // ����ģʽ
		uint8_t reset_ready;
    } holderFlag;
	
	struct
	{
		uint16_t reset_done_cnt;
		uint16_t ready_time;
		uint32_t round_count;
	}holderCount;
	
    struct
	{
    float target_angle;          // Ŀ��Ƕ�
	float reset_angle;
    float remote_sens;           // ң����������
    float mouse_sens;            // ���������
    float vision_sens;           // ����������
    float angle;                 // �����ǻ�õĽǶ�
    float omega;                 // �����ǻ�õĽ��ٶ�
	float delta_angle;
    Motor_t m2006;               // ���ʵ��
	SinglePID_t lockPID;		 // ���iPID
    DualPID_t resetPID;          // ��̨��ң�����ϵ縴λPID
    DualPID_t normalPID;         // ����PID
	DualPID_t autoPID;           // ����PID
	} pitch; // �����������

    struct
	{
    uint16_t target_encoder_ori; // ��̨���е�ʱ��ı������Ƕ�
    float target_angle;          // Ŀ��Ƕ�
	float reset_angle;
    float remote_sens;           // ң����������
    float mouse_sens;            // ���������
    float vision_sens;           // ����������
    float angle;                 // �����ǻ�õĽǶ�
    float omega;                 // �����ǻ�õĽ��ٶ�
    DMiao_t m4310;               // ���ʵ��
	Motor_t m6020;               // ���ʵ��	
    SinglePID_t resetPID;        // ��̨��ң�����ϵ縴λPID
    DualPID_t encoderPID;        // ������PID
    DualPID_t gyroPID;           // ������PID
	DualPID_t autoPID;           // ����PID
	} yaw;   // ƫ���������

    Attitude_t holderAttitude; // ��̨��̬

} Holder_t;

extern Holder_t heroHolder;

void HolderInit(Holder_t *holder);
void Holder_Task(void);

#endif
