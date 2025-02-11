#include "chassis_task.h"
#include "dr16.h"
#include "Supercap.h"
#include "referee_task.h"
#include "holder_task.h"

MecanumChassis_t heroChassis =
    {
        .chassisFlag =
            {
                .follow = 1,
            },
        .movement.sensitivity =
            {
				.v_x_sens = 0.04,
                .v_y_sens = 0.06,
            },
        .power =
            {
                .torque_term_k1 = 2.17e-07,
                .speed_term_k2  = 3.45e-07,
            },
};

/**
 * @brief  麦轮底盘初始化
 */
void MecanumChassisInit(MecanumChassis_t *chassis)
{
    MotorInit(&chassis->wheelMotor.m3508[0], 0, Motor3508, 1, CAN2, 0x201);
    MotorInit(&chassis->wheelMotor.m3508[1], 0, Motor3508, 1, CAN2, 0x202);
    MotorInit(&chassis->wheelMotor.m3508[2], 0, Motor3508, 1, CAN2, 0x203);
    MotorInit(&chassis->wheelMotor.m3508[3], 0, Motor3508, 1, CAN2, 0x204);
}

/**
 * @brief 根据不同等级底盘功率限制给速度分级
 * @note  在不加功率控制时底盘电机转速
 *
 */
static void GradeSpeedBasedPower(MecanumChassis_t *chassis)
{
    float targetpower = chassis->power.refereeData.max_power;
    float speed_limited;
    speed_limited = (float)(0.003924735846L * pow(targetpower, 3) -
                            1.566692684619L * targetpower * targetpower +
                            229.718753161904L * targetpower - 3996.318459737213L);
    if (speed_limited <= 3200)
        speed_limited = 3200;
    chassis->power.speed_limit = speed_limited;
}

/**
 * @brief maomao猫步模式
 * @note  在学弟四方博的思路下 此模式在应对自瞄方面有很大作为
 *
 */
static void ChassisCatWalk(MecanumChassis_t *chassis, Holder_t *holder)
{
    chassis->chassisFlag.cat_walk_cnt++;
    if (chassis->chassisFlag.cat_walk_cnt < 500) 
	{
        chassis->movement.omega = One_Pid_Ctrl(38, holder->yaw.m6020.treatedData.angle, &heroChassis.wheelMotor.chassisFollowPID);
    } 
	else 
	{
        chassis->chassisFlag.cat_walk_cnt = 1000;
        if (holder->yaw.m6020.treatedData.angle >= 37)
            chassis->movement.omega = One_Pid_Ctrl(-38, holder->yaw.m6020.treatedData.angle, &heroChassis.wheelMotor.chassisFollowPID);
        else if (holder->yaw.m6020.treatedData.angle <= -37)
            chassis->movement.omega = One_Pid_Ctrl(38, holder->yaw.m6020.treatedData.angle, &heroChassis.wheelMotor.chassisFollowPID);
        chassis->movement.omega = LIMIT((chassis->movement.omega), -(chassis->power.speed_limit) + 2000, (chassis->power.speed_limit) - 2000);
    }
}

/**
 * @brief  读取接收机控制数据，进行麦轮底盘逆运动学解算，Inverse Kinematics, 根据 chassis 结构体中的 movement 结构体解算转速。
 * @note   处理过后将未进行功率控制的电流参数填写到Motor结构体中。
 */
static void MecanumChassisMotionCtrl(MecanumChassis_t *chassis, Holder_t *holder)
{
    GradeSpeedBasedPower(&heroChassis);
    
	if (chassis->chassisFlag.follow == 1 ) // 在跟随状态下
	{
		if (chassis->chassisFlag.cat_walk == 1) // 猫步模式
			ChassisCatWalk(chassis, holder); 
		else 
		{
			chassis->chassisFlag.cat_walk_cnt = 0;
			if (ABS(holder->yaw.m6020.treatedData.angle) > 1.0f)
				chassis->movement.omega = One_Pid_Ctrl(0, holder->yaw.m6020.treatedData.angle, &heroChassis.wheelMotor.chassisFollowPID); // 对跟随速度不做限制
		}
	} 
	else 
		chassis->movement.omega = 0;
	
    chassis->movement.rotated_v_x     = (chassis->movement.v_x * cos(holder->yaw.m6020.treatedData.angle * 0.01745329f) + chassis->movement.v_y) * sin(holder->yaw.m6020.treatedData.angle * 0.01745329f);
    chassis->movement.rotated_v_y     = (chassis->movement.v_y * cos(holder->yaw.m6020.treatedData.angle * 0.01745329f) - chassis->movement.v_x) * sin(holder->yaw.m6020.treatedData.angle * 0.01745329f);
    chassis->movement.target_speed[0] = 1 * chassis->movement.rotated_v_x  + 1 * chassis->movement.rotated_v_y;
    chassis->movement.target_speed[1] = 1 * chassis->movement.rotated_v_x + (-1) * chassis->movement.rotated_v_y;
    chassis->movement.target_speed[2] = (-1) * chassis->movement.rotated_v_x + (-1) * chassis->movement.rotated_v_y;
    chassis->movement.target_speed[3] = (-1) * chassis->movement.rotated_v_x + 1 * chassis->movement.rotated_v_y;
    for (int8_t i = 0; i < 4; i++) // 对平移速度做限制
    {
        chassis->movement.target_speed[i] = LIMIT((chassis->movement.target_speed[i]), -(chassis->power.speed_limit), (chassis->power.speed_limit));
    }
    chassis->movement.target_speed[0] = chassis->movement.target_speed[0] - chassis->movement.omega;
    chassis->movement.target_speed[1] = chassis->movement.target_speed[1] - chassis->movement.omega;
    chassis->movement.target_speed[2] = chassis->movement.target_speed[2] - chassis->movement.omega;
    chassis->movement.target_speed[3] = chassis->movement.target_speed[3] - chassis->movement.omega;
}
/**
 * @brief 控制超电功率
 *
 */
static uint16_t Cap_input(MecanumChassis_t *chassis, Supercap_t *supercap)
{
    if (supercap->capState.Supercap_Flag == 1 && supercap->capState.cap_voltage > 5)
    {
        if (supercap->capState.cap_voltage < 15)
			chassis->power.cap.cap_power = 10;// 略大于最大功率，避免电容充电，提高能量利用率
        else 
            chassis->power.cap.cap_power = 150;
    } 
	else 
		chassis->power.cap.cap_power = 0;
	return chassis->power.cap.cap_power;
}

/**
 * @brief 底盘功率控制
 *
 */
static void MecanumChassisPowerCtrl(MecanumChassis_t *chassis, Supercap_t *supercap)
{
#if OPEN_REFEREE

	
    chassis->power.refereeData.energy_buffer          = referee2024.power_heat_data.chassis_power_buffer;         // 缓冲能量
    chassis->power.refereeData.real_time_power        = referee2024.power_heat_data.chassis_power;                // 实时功率
    chassis->power.refereeData.max_power              = 80;        // 功率上限 referee2024.game_robot_status.chassis_power_limit
    chassis->power.refereeData.powermngmt_chassis_out = referee2024.game_robot_status.mains_power_chassis_output; // 电管底盘接口输出
    chassis->power.theoretical_power_sum              = chassis->power.refereeData.max_power
														+ ABS(One_Pid_Ctrl(10, chassis->power.refereeData.energy_buffer, &heroChassis.power.chassisPowerPID))
														+ Cap_input(chassis, supercap); 
	
    chassis->power.target_require_power_sum = 0;
    for (uint8_t i = 0; i < 4; i++) 
	{
        chassis->wheelMotor.m3508[i].treatedData.motor_output    = One_Pid_Ctrl(chassis->movement.target_speed[i],
                                                            chassis->wheelMotor.m3508[i].rawData.speed_rpm, &heroChassis.wheelMotor.chassisSpeedPID[i]);
        chassis->power.initial_give_power[i] = chassis->wheelMotor.m3508[i].treatedData.motor_output * TORQUE_COEFFICIENT * chassis->wheelMotor.m3508[i].rawData.speed_rpm +
                                               chassis->power.speed_term_k2 * chassis->wheelMotor.m3508[i].rawData.speed_rpm * chassis->wheelMotor.m3508[i].rawData.speed_rpm +
                                               chassis->power.torque_term_k1 * chassis->wheelMotor.m3508[i].treatedData.motor_output * chassis->wheelMotor.m3508[i].treatedData.motor_output + CONSTANT_COEFFICIENT;
        if (chassis->power.initial_give_power[i] < 0) // negative power not included (transitory)
            continue;
        chassis->power.target_require_power_sum += chassis->power.initial_give_power[i];
    }
    chassis->power.scaling_ratio = chassis->power.theoretical_power_sum / chassis->power.target_require_power_sum;
    chassis->power.scaling_ratio = LIMIT(chassis->power.scaling_ratio, 0, 1);
	for (uint8_t i = 0; i < 4; i++) 
	{
		float b = TORQUE_COEFFICIENT * chassis->wheelMotor.m3508[i].rawData.speed_rpm;
		float c = chassis->power.speed_term_k2 * chassis->wheelMotor.m3508[i].rawData.speed_rpm * chassis->wheelMotor.m3508[i].rawData.speed_rpm - chassis->power.scaled_give_power[i] + CONSTANT_COEFFICIENT;
		if (chassis->power.scaling_ratio==1) continue;
		else
		{
			chassis->power.scaled_give_power[i] = chassis->power.initial_give_power[i] * chassis->power.scaling_ratio; // get scaled power
			if (chassis->power.scaled_give_power[i] < 0)
				continue;
			if (chassis->wheelMotor.m3508[i].treatedData.motor_output > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(b * b - 4 * chassis->power.torque_term_k1 * c)) / (2 * chassis->power.torque_term_k1);
				if (temp > 16000) 
					chassis->wheelMotor.m3508[i].treatedData.motor_output  = 16000;
				else
					chassis->wheelMotor.m3508[i].treatedData.motor_output  = temp;
			} 
			else 
			{
				float temp = (-b - sqrt(b * b - 4 * chassis->power.torque_term_k1 * c)) / (2 * chassis->power.torque_term_k1);
				if (temp < -16000) 
					chassis->wheelMotor.m3508[i].treatedData.motor_output  = -16000;
				else
					chassis->wheelMotor.m3508[i].treatedData.motor_output  = temp;
			}
		}
	}
	if ((ABS(chassis->wheelMotor.m3508[0].treatedData.motor_output) < 900) && (ABS(chassis->wheelMotor.m3508[1].treatedData.motor_output) < 900) && (ABS(chassis->wheelMotor.m3508[2].treatedData.motor_output) < 900) && (ABS(chassis->wheelMotor.m3508[3].treatedData.motor_output) < 900)) 
	{
		for (uint8_t i = 0; i < 4; i++) 
		{
			chassis->wheelMotor.m3508[i].treatedData.motor_output  = 0;
		}
	}

#else
	
    for (int8_t i = 0; i < 4; i++)
	{
	    chassis->wheelMotor.m3508[i].treatedData.motor_output = One_Pid_Ctrl(heroChassis.movement.target_speed[i], heroChassis.wheelMotor.m3508[i].rawData.speed_rpm, &heroChassis.wheelMotor.chassisSpeedPID[i]);
    }
#endif
}

/**
 * @brief 底盘主任务
 *
 */
void Chassis_Task(void)
{
#if (CHASSIS_ENABLE==1)
        							
		for (int8_t i = 0; i < 4; i++) 
		{
			if (rc_Ctrl.is_online == 0 || heroHolder.holderFlag.reset_done == 0 || heroChassis.power.refereeData.powermngmt_chassis_out == 0)
				MotorFillData(&heroChassis.wheelMotor.m3508[i], 0);
			else 
			{
				MecanumChassisMotionCtrl(&heroChassis, &heroHolder);
				MecanumChassisPowerCtrl(&heroChassis, &heroSupercap);
				MotorFillData(&heroChassis.wheelMotor.m3508[i], heroChassis.wheelMotor.m3508[i].treatedData.motor_output);
			} 
		}
#endif
}

