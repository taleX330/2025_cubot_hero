#ifndef _HOLDERTASK_H_
#define _HOLDERTASK_H_

#include "stm32h7xx_hal.h"
#include "rm_motor.h"
#include "pid.h"
#include "ins.h"

#define HOLDER_ENABLE 1
#define PITCHANGLETOROUNT 1000  //將角度值轉換為電機圈數

typedef struct
{
    struct
    {
        uint8_t reset_done;    // 云台复位成功
        uint8_t micro_move;    // 吊射模式
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
    float target_angle;          // 目标角度
	float reset_angle;
    float remote_sens;           // 遥控器灵敏度
    float mouse_sens;            // 鼠标灵敏度
    float vision_sens;           // 自瞄灵敏度
    float angle;                 // 陀螺仪获得的角度
    float omega;                 // 陀螺仪获得的角速度
	float delta_angle;
    Motor_t m2006;               // 电机实例
	SinglePID_t lockPID;		 // 自鎖PID
    DualPID_t resetPID;          // 云台开遥控器上电复位PID
    DualPID_t normalPID;         // 手瞄PID
	DualPID_t autoPID;           // 自瞄PID
	} pitch; // 俯仰相关数据

    struct
	{
    uint16_t target_encoder_ori; // 云台正中的时候的编码器角度
    float target_angle;          // 目标角度
	float reset_angle;
    float remote_sens;           // 遥控器灵敏度
    float mouse_sens;            // 鼠标灵敏度
    float vision_sens;           // 自瞄灵敏度
    float angle;                 // 陀螺仪获得的角度
    float omega;                 // 陀螺仪获得的角速度
    DMiao_t m4310;               // 电机实例
	Motor_t m6020;               // 电机实例	
    SinglePID_t resetPID;        // 云台开遥控器上电复位PID
    DualPID_t encoderPID;        // 编码器PID
    DualPID_t gyroPID;           // 陀螺仪PID
	DualPID_t autoPID;           // 自瞄PID
	} yaw;   // 偏航相关数据

    Attitude_t holderAttitude; // 云台姿态

} Holder_t;

extern Holder_t heroHolder;

void HolderInit(Holder_t *holder);
void Holder_Task(void);

#endif
