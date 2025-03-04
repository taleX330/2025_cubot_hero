#ifndef _SHOOTTASK_H_
#define _SHOOTTASK_H_

#include "stm32h7xx_hal.h"
#include "rm_motor.h"
#include "pid.h"
#include "dr16.h"
#include "ladrc.h"

#define SHOOT_ENABLE 1

/**
 * @brief 摩擦轮电机对象
 *
 */
typedef struct
{
    Motor_t m3508;                // 电机的参数和数据
    int16_t target_speed_config;  // 最终目标转速
    int16_t target_speed_current; // 当前目标转速
	Ladrc_t fricSpeedLadrc;		  // 摩擦轮速度控制LADRC参数
    SinglePID_t fricSpeedPID;     // 摩擦轮速度控制PID参数
} FricInstance_t;
/**
 * @brief 打弹数据
 *
 */
typedef struct
{
    // 标志位
    struct
    {
		uint8_t fric_ready;				  // 摩擦轮准备完毕
		uint8_t shoot_ready;			  // 微动开关标志位
        uint8_t fire;					  // 发弹指令
        uint8_t jam;                      // 链路卡弹
		uint8_t load_start;               // 拨弹盘启动
		uint8_t fric_close;               // 摩擦轮停止
    } shootFlag;    
    // 计数
    struct 
	{
        uint16_t shoot_count; // 打弹计数，打一次加一，初始化或切换射击模式后清零
        uint16_t shoot_count_last;
        struct
        {
            float slowopen_time;  // 摩擦轮缓启动时间
            float slowclose_time; // 摩擦轮缓关闭时间
        } fric;
        struct
        {
            uint16_t jammed_time;  		 // 拨弹盘卡弹判断计时
            uint16_t load_turnback_time; // 拨弹盘反转时间
            uint16_t filled_time;        // 链路填满判断计时
			uint32_t load_time;          // 链路填满判断计时
        } loader;
    } shootCount;
    struct
    {
        FricInstance_t  top;
        FricInstance_t  left;
        FricInstance_t  right;
		uint16_t        speed_top;
		uint16_t        speed_left;
		uint16_t        speed_right;
    } booster;
    struct
    {
		float       angle;
		float       last_angle;
		float       total_angle;
		float       axis_angle;
		float       target_angle;
		float       unit_target_angle;
		Motor_t     m3508;                 // 电机的参数和数据
        int16_t     backward_speed;       // 大拨弹盘反转目标速度
		DualPID_t   loadPID;
        SinglePID_t LoadBackSpeedPID; 
    } loader;
} Shoot_t;

extern Shoot_t heroShoot;

void Shoot_Task(void);
void ShootInit(Shoot_t *shoot);
#endif

