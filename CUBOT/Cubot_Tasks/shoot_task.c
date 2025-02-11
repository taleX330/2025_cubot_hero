#include "shoot_task.h"
#include "user_lib.h"
#include "filter.h"
#include "referee_task.h"
#include "online_ctrl_task.h"
#include "holder_task.h"
Shoot_t heroShoot=
{
	 .booster = 
	{
		.speed_top     = 4890,
		.speed_left    = 4890,	
		.speed_right   = 4890,//4690
	},
	.loader.backward_speed = 800,
	.loader.unit_target_angle = 1.0f,
};

/**
 * @brief 电机初始化
 * @param shoot	
 */
void ShootInit(Shoot_t *shoot)
{
    MotorInit(&shoot->booster.top.m3508   , 0, Motor3508, 1, CAN1, 0x201);
	MotorInit(&shoot->booster.left.m3508  , 0, Motor3508, 1, CAN1, 0x202);
	MotorInit(&shoot->booster.right.m3508 , 0, Motor3508, 1, CAN1, 0x203);
	MotorInit(&shoot->loader.m3508        , 0, Motor3508, 1, CAN2, 0x205);
}

/**
 * @brief 获取拨弹盘数据 
 *
 * @param shoot
 */
static void GetLoadData(Shoot_t *shoot)
{
	shoot->shootCount.loader.load_time++;
	shoot->loader.angle = shoot->loader.m3508.treatedData.angle;
	shoot->loader.m3508.treatedData.filter_speed_rpm = LPFilter(shoot->loader.m3508.rawData.speed_rpm, &LPF_Load);
	
	if((shoot->loader.angle < -100) && (shoot->loader.last_angle > 100))
		shoot->loader.total_angle += 360 + shoot->loader.angle - shoot->loader.last_angle;
	else if(( shoot->loader.angle > 100) && (shoot->loader.last_angle < -100))
		shoot->loader.total_angle += -360 + shoot->loader.angle - shoot->loader.last_angle;
	else 
		shoot->loader.total_angle +=shoot->loader.angle-shoot->loader.last_angle;
	shoot->loader.last_angle = shoot->loader.angle;
	shoot->loader.axis_angle = shoot->loader.total_angle/27.0f;
	
	if(shoot->shootFlag.load_start == 1 && shoot->shootFlag.jam == 0)
	{
		if(shoot->shootCount.loader.load_time % 4 == 0)
			shoot->loader.target_angle -= shoot->loader.unit_target_angle;
	}
	else 
		shoot->loader.target_angle = shoot->loader.axis_angle;	
}

/**
 * @brief 获取發射機構数据 
 *
 * @param shoot
 */
static void ShootGetData(Shoot_t *shoot)
{
	shoot->booster.top.target_speed_config   = -(shoot->booster.speed_top);
	shoot->booster.left.target_speed_config  = -(shoot->booster.speed_left);
	shoot->booster.right.target_speed_config = shoot->booster.speed_right;
	
	shoot->booster.top.m3508.treatedData.filter_speed_rpm   = LPFilter(shoot->booster.top.m3508.rawData.speed_rpm,   &LPF_Fric_top);
	shoot->booster.left.m3508.treatedData.filter_speed_rpm  = LPFilter(shoot->booster.left.m3508.rawData.speed_rpm,  &LPF_Fric_left);
	shoot->booster.right.m3508.treatedData.filter_speed_rpm = LPFilter(shoot->booster.right.m3508.rawData.speed_rpm, &LPF_Fric_right);

	GetLoadData(shoot);
}

/**
 * @brief 摩擦轮缓启动缓关闭
 * @param shoot
 * @param rc_ctrl
 */
static void SlowOpenAndClose(Shoot_t *shoot, RC_Ctrl *rc_ctrl)
{
	if(onlineFlag.fric_top ==1 && onlineFlag.fric_left == 1 && onlineFlag.fric_right == 1 && rc_ctrl->is_online == 1 && heroHolder.holderFlag.reset_done == 1)
	{
		shoot->shootFlag.fric_close = 0;
		shoot->shootCount.fric.slowclose_time = 0;
		if(shoot->shootCount.fric.slowopen_time < 1000)
		{
			shoot->shootCount.fric.slowopen_time++;
			shoot->booster.top.target_speed_current   = (int16_t)shoot->booster.top.target_speed_config   * 0.001f * shoot->shootCount.fric.slowopen_time;
 			shoot->booster.left.target_speed_current  = (int16_t)shoot->booster.left.target_speed_config  * 0.001f * shoot->shootCount.fric.slowopen_time;
			shoot->booster.right.target_speed_current = (int16_t)shoot->booster.right.target_speed_config * 0.001f * shoot->shootCount.fric.slowopen_time;
		}
		else
		{
			shoot->booster.top.target_speed_current   = (int16_t)shoot->booster.top.target_speed_config;
			shoot->booster.left.target_speed_current  = (int16_t)shoot->booster.left.target_speed_config;
			shoot->booster.right.target_speed_current = (int16_t)shoot->booster.right.target_speed_config;
			shoot->shootCount.fric.slowopen_time 	  = 1000;
		}
	}
	else if(shoot->shootFlag.fric_ready == 1)
	{
		shoot->shootCount.fric.slowopen_time = 0;
		if(shoot->shootCount.fric.slowclose_time < 1500)
		{
			shoot->shootCount.fric.slowclose_time++;
			shoot->booster.top.target_speed_current   = (int16_t)shoot->booster.top.target_speed_config   * (1500-shoot->shootCount.fric.slowclose_time) / 1500;
			shoot->booster.left.target_speed_current  = (int16_t)shoot->booster.left.target_speed_config  * (1500-shoot->shootCount.fric.slowclose_time) / 1500;
			shoot->booster.right.target_speed_current = (int16_t)shoot->booster.right.target_speed_config * (1500-shoot->shootCount.fric.slowclose_time) / 1500;
		}
		else
		{
			shoot->booster.top.m3508.treatedData.motor_output   = 0;
			shoot->booster.left.m3508.treatedData.motor_output  = 0;
			shoot->booster.right.m3508.treatedData.motor_output = 0;
			shoot->shootCount.fric.slowclose_time       	    = 1500;
			shoot->shootFlag.fric_ready                         = 0;
		}
	}
	else
		shoot->shootFlag.fric_close = 1;
}

/**
 * @brief 摩擦轮控制函数 
 *
 * @param shoot
 * @param rc_ctrl
 */
static void ShootControl(Shoot_t *shoot, RC_Ctrl *rc_ctrl)
{
	SlowOpenAndClose(shoot, rc_ctrl);
	if(shoot->shootFlag.fric_close == 1)
	{
		shoot->booster.top.m3508.treatedData.motor_output   = 0;
		shoot->booster.left.m3508.treatedData.motor_output  = 0;
		shoot->booster.right.m3508.treatedData.motor_output = 0;
	}
	else
	{
		shoot->booster.top.m3508.treatedData.motor_output   = One_Pid_Ctrl(shoot->booster.top.target_speed_current,   shoot->booster.top.m3508.treatedData.filter_speed_rpm,   &shoot->booster.top.fricSpeedPID);
		shoot->booster.left.m3508.treatedData.motor_output  = One_Pid_Ctrl(shoot->booster.left.target_speed_current,  shoot->booster.left.m3508.treatedData.filter_speed_rpm,  &shoot->booster.left.fricSpeedPID);	
		shoot->booster.right.m3508.treatedData.motor_output = One_Pid_Ctrl(shoot->booster.right.target_speed_current, shoot->booster.right.m3508.treatedData.filter_speed_rpm, &shoot->booster.right.fricSpeedPID);		
		if(shoot->booster.top.m3508.rawData.torque_current < -1500 && shoot->shootFlag.fire == 1)
		{
			shoot->booster.top.m3508.treatedData.motor_output   = -16000;
			shoot->booster.left.m3508.treatedData.motor_output  = -16000;
			shoot->booster.right.m3508.treatedData.motor_output = 16000;
		}
	}
}

/**
 * @brief 堵转判断
 *
 * @param shoot
 */
static void JamJudge(Shoot_t *shoot)
{
	if(ABS(shoot->loader.m3508.treatedData.motor_output) >= 15000)
		shoot->shootCount.loader.jammed_time ++;
	if(shoot->shootCount.loader.jammed_time > 1000)
	{
		shoot->shootFlag.jam = 1;
		shoot->shootCount.loader.load_turnback_time ++;
	}
	if(shoot->shootCount.loader.load_turnback_time > 300)
	{
		shoot->shootFlag.jam = 0;
		shoot->shootCount.loader.load_turnback_time = 0;
		shoot->shootCount.loader.jammed_time        = 0;
	}
}

/**
 * @brief 拨弹盘控制函数 发弹逻辑
 *
 * @param shoot
 */
static void Loadcontrol(Shoot_t *shoot)
{
	JamJudge(shoot);		
	if(shoot->shootCount.fric.slowopen_time == 1000)
	{	
		shoot->shootFlag.fric_ready = 1;
		if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_3) == 1 )
			heroShoot.shootFlag.shoot_ready = 1;
		else
			heroShoot.shootFlag.shoot_ready = 0;
		if(shoot->shootFlag.jam == 0)
		{
			if((shoot->shootFlag.fire == 1 && shoot->shootFlag.shoot_ready == 1) || shoot->shootFlag.shoot_ready == 0)
				shoot->shootFlag.load_start = 1;
			else
				shoot->shootFlag.load_start = 0;
			shoot->loader.m3508.treatedData.motor_output = Double_Pid_Ctrl(shoot->loader.target_angle,shoot->loader.axis_angle, shoot->loader.m3508.treatedData.filter_speed_rpm, &shoot->loader.loadPID);
		}
		else
			shoot->loader.m3508.treatedData.motor_output = One_Pid_Ctrl(shoot->loader.backward_speed, shoot->loader.m3508.treatedData.filter_speed_rpm, &shoot->loader.LoadBackSpeedPID);
	}
	else
		shoot->loader.m3508.treatedData.motor_output = 0;
	if(ABS(shoot->booster.top.m3508.rawData.speed_rpm) < 4500)
		shoot->shootFlag.fire = 0;
}

/**
 * @brief 将电流值发送至缓存区
 *
 * @param shoot
 */
static void ShootOutputCtrl(Shoot_t *shoot)
{
    MotorFillData(&shoot->booster.top.m3508   , shoot->booster.top.m3508.treatedData.motor_output);
	MotorFillData(&shoot->booster.left.m3508  , shoot->booster.left.m3508.treatedData.motor_output);
	MotorFillData(&shoot->booster.right.m3508 , shoot->booster.right.m3508.treatedData.motor_output);
	MotorFillData(&shoot->loader.m3508 		  , shoot->loader.m3508.treatedData.motor_output);
}
void Shoot_Task(void)
{
#if(SHOOT_ENABLE == 1)
		if(heroHolder.holderFlag.reset_done == 1)
			Loadcontrol(&heroShoot);
		else
		{
			heroShoot.loader.m3508.treatedData.motor_output = 0;
			heroShoot.loader.axis_angle                     = 0;
			heroShoot.shootFlag.load_start                  = 0;
		}
		ShootGetData(&heroShoot);
		ShootControl(&heroShoot,&rc_Ctrl);
		ShootOutputCtrl(&heroShoot);
#endif	
}
