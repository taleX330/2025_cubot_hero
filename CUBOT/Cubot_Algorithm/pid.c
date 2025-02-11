#include "pid.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "holder_task.h"
#include "shoot_task.h"

/*Pitch轴复位内外环*/
SinglePID_t shellRestAngle;
SinglePID_t coreRestAngle;
/*Yaw轴编码器内外环*/
SinglePID_t shellYawEncoderAngle;
SinglePID_t coreYawEncoderSpeed;
/*Pitch轴编码器内外环*/
SinglePID_t shellPitchEncoderAngle;
SinglePID_t corePitchEncoderSpeed;
/*Yaw轴陀螺仪内外环*/
SinglePID_t shellYawGyroAngle;
SinglePID_t coreYawGyroSpeed;
/*Pitch轴陀螺仪内外环*/
SinglePID_t shellPitchGyroAngle;
SinglePID_t corePitchGyroSpeed;
/*Yaw轴自瞄内外环*/
SinglePID_t shellYawAutoAngle;
SinglePID_t coreYawAutoSpeed;
/*Pitch轴自瞄内外环*/
SinglePID_t shellPitchAutoAngle;
SinglePID_t corePitchAutoSpeed;
/*拨弹盘内外环*/
SinglePID_t shellLoadAngle;
SinglePID_t coreLoadSpeed;
/**
 * @brief 单环PID初始化
 *
 * @param single_pid 被赋值的结构体地址
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param pMaxlimit 比例部分控制量输出限幅
 * @param iMaxlimit 积分部分控制量输出限幅
 * @param dMaxlimit 微分部分控制量输出限幅
 * @param iPartDetach 积分部分死区
 * @param OutputLimit 总输出限幅
 */
static void BasePID_Init(SinglePID_t *single_pid,
                         float kp, float ki, float kd,
                         float pMaxlimit, float iMaxlimit, float dMaxlimit,
                         float iPartDetach_lower, float iPartDetach_upper,
						 float OutputLimit)
{
    single_pid->P            		  = kp;
    single_pid->I            		  = ki;
    single_pid->D             		  = kd;
    single_pid->p_part_maxlimit 	  = pMaxlimit;
    single_pid->i_part_maxlimit 	  = iMaxlimit;
    single_pid->i_part_detach_upper   = iPartDetach_upper;
	single_pid->i_part_detach_lower   = iPartDetach_lower;
    single_pid->d_part_maxlimit 	  = dMaxlimit;
    single_pid->max_limit      		  = OutputLimit;
}
/**
 * @brief 双环PID初始化
 *
 * @param dual_pid 被赋值的结构体地址
 * @param shell_single_pid 赋外环PID值的结构体地址
 * @param core_single_pid 赋内环PID值的结构体地址
 */
static void DualPID_Init(DualPID_t *dual_pid, SinglePID_t *shell_single_pid, SinglePID_t *core_single_pid)
{
    // 外环赋值
    dual_pid->shell.shell_P               = shell_single_pid->P;
    dual_pid->shell.shell_I               = shell_single_pid->I;
    dual_pid->shell.shell_D               = shell_single_pid->D;
    dual_pid->shell.shell_p_part_maxlimit = shell_single_pid->p_part_maxlimit;
    dual_pid->shell.shell_i_part_maxlimit = shell_single_pid->i_part_maxlimit;
    dual_pid->shell.shell_i_part_detach_upper   = shell_single_pid->i_part_detach_upper;
	dual_pid->shell.shell_i_part_detach_lower   = shell_single_pid->i_part_detach_lower;
    dual_pid->shell.shell_d_part_maxlimit = shell_single_pid->d_part_maxlimit;
    dual_pid->shell.shell_max_limit       = shell_single_pid->max_limit;
    // 内环赋值
    dual_pid->core.core_P               = core_single_pid->P;
    dual_pid->core.core_I               = core_single_pid->I;
    dual_pid->core.core_D               = core_single_pid->D;
    dual_pid->core.core_p_part_maxlimit = core_single_pid->p_part_maxlimit;
    dual_pid->core.core_i_part_maxlimit = core_single_pid->i_part_maxlimit;
    dual_pid->core.core_i_part_detach_upper   = core_single_pid->i_part_detach_upper;
	dual_pid->core.core_i_part_detach_lower   = core_single_pid->i_part_detach_lower;
    dual_pid->core.core_d_part_maxlimit = core_single_pid->d_part_maxlimit;
    dual_pid->core.core_max_limit       = core_single_pid->max_limit;
}
/**
 * @brief 单环PID计算
 *
 * @param target 目标值
 * @param feedback 反馈值
 * @param PID 单环PID结构体地址
 * @return float
 */
float One_Pid_Ctrl(float target, float feedback, SinglePID_t *PID)
{
    PID->delta = target - feedback;
    /************ P操作 ************/
    PID->p_part = PID->delta * PID->P;
    PID->p_part = LIMIT((PID->p_part), -(PID->p_part_maxlimit), (PID->p_part_maxlimit));
    /************ I操作 ************/
    PID->i_delta_sum += PID->delta;
    PID->i_part = PID->i_delta_sum * PID->I;
    if (ABS(PID->delta) > PID->i_part_detach_upper)
        PID->i_delta_sum = 0;
	if(ABS(PID->delta) < PID->i_part_detach_lower)
		PID->i_delta_sum = 0;
    PID->i_part = LIMIT((PID->i_part), -(PID->i_part_maxlimit), (PID->i_part_maxlimit));
    /************ D操作 ************/
    PID->d_part     = (PID->delta - PID->delta_last) * PID->D;
    PID->d_part     = LIMIT((PID->d_part), -(PID->d_part_maxlimit), (PID->d_part_maxlimit));
    PID->delta_last = PID->delta;
    /************  输出 ************/
    PID->out = (PID->p_part + PID->i_part + PID->d_part);
    PID->out = LIMIT((PID->out), -(PID->max_limit), (PID->max_limit));
    return PID->out;
}
/**
 * @brief 双环PID计算
 *
 * @param shell_target 目标值
 * @param shell_feedback 外环反馈值
 * @param core_feedback 内环反馈值
 * @param PID 双环PID结构体地址
 * @return float
 */
float Double_Pid_Ctrl(float shell_target, float shell_feedback, float core_feedback, DualPID_t *PID)
{
    PID->shell.shell_delta = shell_target - shell_feedback;
    /************ 外环P操作 ************/
    PID->shell.shell_p_part = PID->shell.shell_delta * PID->shell.shell_P;
    PID->shell.shell_p_part = LIMIT((PID->shell.shell_p_part), -(PID->shell.shell_p_part_maxlimit), (PID->shell.shell_p_part_maxlimit));
    /************ 外环I操作 ************/
    PID->shell.shell_i_part += PID->shell.shell_delta * PID->shell.shell_I;
    if (ABS(PID->shell.shell_delta) > PID->shell.shell_i_part_detach_upper)
        PID->shell.shell_i_part = 0;
	if (ABS(PID->shell.shell_delta) < PID->shell.shell_i_part_detach_lower)
        PID->shell.shell_i_part = 0;
    PID->shell.shell_i_part = LIMIT((PID->shell.shell_i_part), -(PID->shell.shell_i_part_maxlimit), (PID->shell.shell_i_part_maxlimit));
    /************ 外环D操作 ************/
    PID->shell.shell_d_part     = (PID->shell.shell_delta - PID->shell.shell_delta_last) * PID->shell.shell_D;
    PID->shell.shell_d_part     = LIMIT((PID->shell.shell_d_part), -(PID->shell.shell_d_part_maxlimit), (PID->shell.shell_d_part_maxlimit));
    PID->shell.shell_delta_last = PID->shell.shell_delta;
    /************ 外环输出 ************/
    PID->shell.shell_out = (PID->shell.shell_p_part + PID->shell.shell_i_part + PID->shell.shell_d_part);
    PID->shell.shell_out = LIMIT((PID->shell.shell_out), -(PID->shell.shell_max_limit), (PID->shell.shell_max_limit));
    PID->core.core_delta = PID->shell.shell_out - core_feedback;
    /************ 内环P操作 ************/
    PID->core.core_p_part = PID->core.core_delta * PID->core.core_P;
    PID->core.core_p_part = LIMIT((PID->core.core_p_part), -(PID->core.core_p_part_maxlimit), (PID->core.core_p_part_maxlimit));
    /************ 内环I操作 ************/
    PID->core.core_i_part += PID->core.core_delta * PID->core.core_I;
    if (ABS(PID->core.core_delta) > PID->core.core_i_part_detach_upper)
        PID->core.core_i_part = 0;
	if (ABS(PID->core.core_delta) < PID->core.core_i_part_detach_lower)
        PID->core.core_i_part = 0;
    PID->core.core_i_part = LIMIT((PID->core.core_i_part), -(PID->core.core_i_part_maxlimit), (PID->core.core_i_part_maxlimit));
    /************ 内环D操作 ************/
    PID->core.core_d_part     = (PID->core.core_delta - PID->core.core_delta_last) * PID->core.core_D;
    PID->core.core_d_part     = LIMIT((PID->core.core_d_part), -(PID->core.core_d_part_maxlimit), (PID->core.core_d_part_maxlimit));
    PID->core.core_delta_last = PID->core.core_delta;
    /************ 内环输出 ************/
    PID->core.core_out = (PID->core.core_p_part + PID->core.core_i_part + PID->core.core_d_part);
    PID->core.core_out = LIMIT((PID->core.core_out), -(PID->core.core_max_limit), (PID->core.core_max_limit));
    return PID->core.core_out;
}

void BasePID_Init_All(void)
{
    /************ 底盘PID初始化 ************/
	
    //< 底盘速度
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[0], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[1], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[2], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[3], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    //< 底盘跟随
    BasePID_Init(&heroChassis.wheelMotor.chassisFollowPID, 150, 0, 7000, 8000, 0, 7000, 0, 0,  15000);
    //< 底盘功率
    BasePID_Init(&heroChassis.power.chassisPowerPID, 3, 0.001, 0, 250, 250, 250, 3, 0, 250);
	
    /************ Yaw軸PID初始化 ************/
	
    //< Yaw轴复位单环
    BasePID_Init(&heroHolder.yaw.resetPID, 650, 0, 2, 29000, 0, 5000, 0, 0, 29000);
    //< Yaw轴陀螺仪角度双环
    BasePID_Init(&shellYawGyroAngle, 65,0.013,0, 210, 210, 210, 2, 0.001, 210);//1, 0, 40,
    BasePID_Init(&coreYawGyroSpeed, 115,0,0, 30000, 10000, 5000, 0.5, 0, 30000);//-1250, 0, 0
    DualPID_Init(&heroHolder.yaw.gyroPID, &shellYawGyroAngle, &coreYawGyroSpeed);
    //< Yaw轴编码器角度双环
    BasePID_Init(&shellYawEncoderAngle, 65,0.013,0, 210, 210, 210, 2, 0.001, 210);//1, 0, 40,
    BasePID_Init(&coreYawEncoderSpeed, 115,0,0, 30000, 10000, 5000, 0.5, 0, 30000);//-1250, 0, 0
    DualPID_Init(&heroHolder.yaw.encoderPID, &shellYawEncoderAngle, &coreYawEncoderSpeed);
    //< Yaw轴自瞄角度双环
    BasePID_Init(&shellYawAutoAngle, 10, 0, 0, 210, 0, 0, 0.5, 0, 210);
    BasePID_Init(&coreYawAutoSpeed, -2300, 0, 0, 30000, 10000, 5000, 0.5, 0, 30000);
    DualPID_Init(&heroHolder.yaw.autoPID, &shellYawAutoAngle, &coreYawAutoSpeed);
	
    /************ Pitch軸PID初始化 ************/
	
	//< Pitch轴复位双环
    BasePID_Init(&shellRestAngle, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    BasePID_Init(&coreRestAngle, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	DualPID_Init(&heroHolder.pitch.resetPID, &shellRestAngle, &coreRestAngle);
    //< Pitch軸手瞄轴角度双环
    BasePID_Init(&shellPitchEncoderAngle,10, 0.1, 0, 100, 50, 100, 7, 0.001, 500);//2, 0.3, 60
    BasePID_Init(&corePitchEncoderSpeed,80, 0, 0,  30000, 2000, 8000, 0.5, 0, 30000);//1000, 2.5, 0
    DualPID_Init(&heroHolder.pitch.normalPID, &shellPitchEncoderAngle, &corePitchEncoderSpeed);
	//< Pitch軸自鎖速度單环
	BasePID_Init(&heroHolder.pitch.lockPID,0, 0, 0, 0, 0, 0, 0, 0, 0);
    //< Pitch轴自瞄角度双环
    BasePID_Init(&shellPitchAutoAngle, -2, 0, 0, 100, 50, 100, 0.2, 0, 500);
    BasePID_Init(&corePitchAutoSpeed, 0, 0, 0, 30000, 2000, 8000, 0.5, 0, 30000);
    DualPID_Init(&heroHolder.pitch.autoPID, &shellPitchAutoAngle, &corePitchAutoSpeed);
	
    /************ 打弹PID初始化 ************/
	
     //< 上摩擦轮速度
    BasePID_Init(&heroShoot.booster.top.fricSpeedPID, 38, 0, 0, 8000, 8000, 8000, 30, 10, 16000);
	//< 左摩擦轮速度
	BasePID_Init(&heroShoot.booster.left.fricSpeedPID, 38, 0, 0, 16000, 8000, 8000, 30, 10, 16000);
    //< 右摩擦轮速度
    BasePID_Init(&heroShoot.booster.right.fricSpeedPID, 38, 0, 0, 16000, 8000, 8000, 30, 10, 16000);
    //< 拨弹盘速度
    BasePID_Init(&heroShoot.loader.LoadBackSpeedPID, 5, 0, 0, 8000, 1000, 100, 50, 10, 10000);
	//< 拨弹盘角度积分控制速度
    BasePID_Init(&shellLoadAngle, 20, 0,  0, 30000, 2000, 100, 0, 0, 15000);
	BasePID_Init(&coreLoadSpeed, 15, 0,  0, 30000, 2000, 100, 0, 0, 15000);
	DualPID_Init(&heroShoot.loader.loadPID, &shellLoadAngle,&coreLoadSpeed);	
}
