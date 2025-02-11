#ifndef _CHASSISTASK_H_
#define _CHASSISTASK_H_

#include "stm32h7xx_hal.h"
#include "user_lib.h"
#include "pid.h"
#include "rm_motor.h"


#define CHASSIS_ENABLE       1
#define CHASSIS_CURRENT_MAX  64000         // 最大总装载，即所有电机输出的电流值最多是多少
#define TORQUE_COEFFICIENT   1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55，其中9.55约等于60/2pi
#define CONSTANT_COEFFICIENT 1.381f

/**
 * @brief  麦轮底盘结构体
 */
typedef struct
{
    struct
    {
        uint8_t  CapState;      // 超电状态
		uint8_t  follow;        // 跟随
		uint8_t  cat_walk;      // 貓步
		uint16_t cat_walk_cnt;
    } chassisFlag;

    struct
    {
        Motor_t     m3508[4];             // 底盘电机结构体
        SinglePID_t chassisSpeedPID[4];  // 速度控制PID参数
        SinglePID_t chassisFollowPID; // 底盘跟随PID参数
    } wheelMotor;

    struct
    {
        int16_t v_x;   // 前后运动的速度
        int16_t v_y;   // 左右运动的速度
        int16_t omega; // 旋转的角速度
        int16_t delta_v_x;
        int16_t delta_v_y;
        int16_t rotated_v_x;
        int16_t rotated_v_y;
        int16_t target_speed[4];   // 经过解算的未进行功率控制的速度
        struct
        {
            float v_x_sens;
            float v_y_sens;
        } sensitivity;
    } movement;

    struct
    {
        float speed_limit;              // 根据不同等级底盘功率限制得到的速度
        float theoretical_power_sum;    // 想让缓冲能量保持在10J所需要的底盘总功率理论值
        float target_require_power_sum; // 直接进行速度PID计算后得到的控制电流值之和，用以表示功率实际值
        float scaling_ratio;            // 缩放比例 = 功率理论值 / 功率实际值
        float initial_give_power[4];    // initial power from PID calculation
        float scaled_give_power[4];
        float torque_term_k1;           // k1
        float speed_term_k2;            // k2
        SinglePID_t chassisPowerPID;    // 功率控制PID参数
        struct
        {
            float    real_time_power;          // 实时功率
            uint8_t  powermngmt_chassis_out;   // 电管底盘接口的输出
            uint16_t energy_buffer;            // 缓冲能量
            uint16_t max_power;                // 功率上限
        } refereeData;
		struct
		{
			uint16_t cap_power;
		}cap;
    } power;

} MecanumChassis_t;

extern MecanumChassis_t heroChassis;

void MecanumChassisInit(MecanumChassis_t *chassis);
void Chassis_Task(void);

#endif
