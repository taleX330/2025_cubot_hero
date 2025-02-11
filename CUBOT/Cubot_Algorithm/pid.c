#include "pid.h"
#include "user_lib.h"
#include "chassis_task.h"
#include "holder_task.h"
#include "shoot_task.h"

/*Pitch�Ḵλ���⻷*/
SinglePID_t shellRestAngle;
SinglePID_t coreRestAngle;
/*Yaw����������⻷*/
SinglePID_t shellYawEncoderAngle;
SinglePID_t coreYawEncoderSpeed;
/*Pitch����������⻷*/
SinglePID_t shellPitchEncoderAngle;
SinglePID_t corePitchEncoderSpeed;
/*Yaw�����������⻷*/
SinglePID_t shellYawGyroAngle;
SinglePID_t coreYawGyroSpeed;
/*Pitch�����������⻷*/
SinglePID_t shellPitchGyroAngle;
SinglePID_t corePitchGyroSpeed;
/*Yaw���������⻷*/
SinglePID_t shellYawAutoAngle;
SinglePID_t coreYawAutoSpeed;
/*Pitch���������⻷*/
SinglePID_t shellPitchAutoAngle;
SinglePID_t corePitchAutoSpeed;
/*���������⻷*/
SinglePID_t shellLoadAngle;
SinglePID_t coreLoadSpeed;
/**
 * @brief ����PID��ʼ��
 *
 * @param single_pid ����ֵ�Ľṹ���ַ
 * @param kp ����ϵ��
 * @param ki ����ϵ��
 * @param kd ΢��ϵ��
 * @param pMaxlimit �������ֿ���������޷�
 * @param iMaxlimit ���ֲ��ֿ���������޷�
 * @param dMaxlimit ΢�ֲ��ֿ���������޷�
 * @param iPartDetach ���ֲ�������
 * @param OutputLimit ������޷�
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
 * @brief ˫��PID��ʼ��
 *
 * @param dual_pid ����ֵ�Ľṹ���ַ
 * @param shell_single_pid ���⻷PIDֵ�Ľṹ���ַ
 * @param core_single_pid ���ڻ�PIDֵ�Ľṹ���ַ
 */
static void DualPID_Init(DualPID_t *dual_pid, SinglePID_t *shell_single_pid, SinglePID_t *core_single_pid)
{
    // �⻷��ֵ
    dual_pid->shell.shell_P               = shell_single_pid->P;
    dual_pid->shell.shell_I               = shell_single_pid->I;
    dual_pid->shell.shell_D               = shell_single_pid->D;
    dual_pid->shell.shell_p_part_maxlimit = shell_single_pid->p_part_maxlimit;
    dual_pid->shell.shell_i_part_maxlimit = shell_single_pid->i_part_maxlimit;
    dual_pid->shell.shell_i_part_detach_upper   = shell_single_pid->i_part_detach_upper;
	dual_pid->shell.shell_i_part_detach_lower   = shell_single_pid->i_part_detach_lower;
    dual_pid->shell.shell_d_part_maxlimit = shell_single_pid->d_part_maxlimit;
    dual_pid->shell.shell_max_limit       = shell_single_pid->max_limit;
    // �ڻ���ֵ
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
 * @brief ����PID����
 *
 * @param target Ŀ��ֵ
 * @param feedback ����ֵ
 * @param PID ����PID�ṹ���ַ
 * @return float
 */
float One_Pid_Ctrl(float target, float feedback, SinglePID_t *PID)
{
    PID->delta = target - feedback;
    /************ P���� ************/
    PID->p_part = PID->delta * PID->P;
    PID->p_part = LIMIT((PID->p_part), -(PID->p_part_maxlimit), (PID->p_part_maxlimit));
    /************ I���� ************/
    PID->i_delta_sum += PID->delta;
    PID->i_part = PID->i_delta_sum * PID->I;
    if (ABS(PID->delta) > PID->i_part_detach_upper)
        PID->i_delta_sum = 0;
	if(ABS(PID->delta) < PID->i_part_detach_lower)
		PID->i_delta_sum = 0;
    PID->i_part = LIMIT((PID->i_part), -(PID->i_part_maxlimit), (PID->i_part_maxlimit));
    /************ D���� ************/
    PID->d_part     = (PID->delta - PID->delta_last) * PID->D;
    PID->d_part     = LIMIT((PID->d_part), -(PID->d_part_maxlimit), (PID->d_part_maxlimit));
    PID->delta_last = PID->delta;
    /************  ��� ************/
    PID->out = (PID->p_part + PID->i_part + PID->d_part);
    PID->out = LIMIT((PID->out), -(PID->max_limit), (PID->max_limit));
    return PID->out;
}
/**
 * @brief ˫��PID����
 *
 * @param shell_target Ŀ��ֵ
 * @param shell_feedback �⻷����ֵ
 * @param core_feedback �ڻ�����ֵ
 * @param PID ˫��PID�ṹ���ַ
 * @return float
 */
float Double_Pid_Ctrl(float shell_target, float shell_feedback, float core_feedback, DualPID_t *PID)
{
    PID->shell.shell_delta = shell_target - shell_feedback;
    /************ �⻷P���� ************/
    PID->shell.shell_p_part = PID->shell.shell_delta * PID->shell.shell_P;
    PID->shell.shell_p_part = LIMIT((PID->shell.shell_p_part), -(PID->shell.shell_p_part_maxlimit), (PID->shell.shell_p_part_maxlimit));
    /************ �⻷I���� ************/
    PID->shell.shell_i_part += PID->shell.shell_delta * PID->shell.shell_I;
    if (ABS(PID->shell.shell_delta) > PID->shell.shell_i_part_detach_upper)
        PID->shell.shell_i_part = 0;
	if (ABS(PID->shell.shell_delta) < PID->shell.shell_i_part_detach_lower)
        PID->shell.shell_i_part = 0;
    PID->shell.shell_i_part = LIMIT((PID->shell.shell_i_part), -(PID->shell.shell_i_part_maxlimit), (PID->shell.shell_i_part_maxlimit));
    /************ �⻷D���� ************/
    PID->shell.shell_d_part     = (PID->shell.shell_delta - PID->shell.shell_delta_last) * PID->shell.shell_D;
    PID->shell.shell_d_part     = LIMIT((PID->shell.shell_d_part), -(PID->shell.shell_d_part_maxlimit), (PID->shell.shell_d_part_maxlimit));
    PID->shell.shell_delta_last = PID->shell.shell_delta;
    /************ �⻷��� ************/
    PID->shell.shell_out = (PID->shell.shell_p_part + PID->shell.shell_i_part + PID->shell.shell_d_part);
    PID->shell.shell_out = LIMIT((PID->shell.shell_out), -(PID->shell.shell_max_limit), (PID->shell.shell_max_limit));
    PID->core.core_delta = PID->shell.shell_out - core_feedback;
    /************ �ڻ�P���� ************/
    PID->core.core_p_part = PID->core.core_delta * PID->core.core_P;
    PID->core.core_p_part = LIMIT((PID->core.core_p_part), -(PID->core.core_p_part_maxlimit), (PID->core.core_p_part_maxlimit));
    /************ �ڻ�I���� ************/
    PID->core.core_i_part += PID->core.core_delta * PID->core.core_I;
    if (ABS(PID->core.core_delta) > PID->core.core_i_part_detach_upper)
        PID->core.core_i_part = 0;
	if (ABS(PID->core.core_delta) < PID->core.core_i_part_detach_lower)
        PID->core.core_i_part = 0;
    PID->core.core_i_part = LIMIT((PID->core.core_i_part), -(PID->core.core_i_part_maxlimit), (PID->core.core_i_part_maxlimit));
    /************ �ڻ�D���� ************/
    PID->core.core_d_part     = (PID->core.core_delta - PID->core.core_delta_last) * PID->core.core_D;
    PID->core.core_d_part     = LIMIT((PID->core.core_d_part), -(PID->core.core_d_part_maxlimit), (PID->core.core_d_part_maxlimit));
    PID->core.core_delta_last = PID->core.core_delta;
    /************ �ڻ���� ************/
    PID->core.core_out = (PID->core.core_p_part + PID->core.core_i_part + PID->core.core_d_part);
    PID->core.core_out = LIMIT((PID->core.core_out), -(PID->core.core_max_limit), (PID->core.core_max_limit));
    return PID->core.core_out;
}

void BasePID_Init_All(void)
{
    /************ ����PID��ʼ�� ************/
	
    //< �����ٶ�
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[0], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[1], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[2], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    BasePID_Init(&heroChassis.wheelMotor.chassisSpeedPID[3], 3, 0, 0, 16000, 0, 0, 0, 0, 16000);
    //< ���̸���
    BasePID_Init(&heroChassis.wheelMotor.chassisFollowPID, 150, 0, 7000, 8000, 0, 7000, 0, 0,  15000);
    //< ���̹���
    BasePID_Init(&heroChassis.power.chassisPowerPID, 3, 0.001, 0, 250, 250, 250, 3, 0, 250);
	
    /************ Yaw�SPID��ʼ�� ************/
	
    //< Yaw�Ḵλ����
    BasePID_Init(&heroHolder.yaw.resetPID, 650, 0, 2, 29000, 0, 5000, 0, 0, 29000);
    //< Yaw�������ǽǶ�˫��
    BasePID_Init(&shellYawGyroAngle, 65,0.013,0, 210, 210, 210, 2, 0.001, 210);//1, 0, 40,
    BasePID_Init(&coreYawGyroSpeed, 115,0,0, 30000, 10000, 5000, 0.5, 0, 30000);//-1250, 0, 0
    DualPID_Init(&heroHolder.yaw.gyroPID, &shellYawGyroAngle, &coreYawGyroSpeed);
    //< Yaw��������Ƕ�˫��
    BasePID_Init(&shellYawEncoderAngle, 65,0.013,0, 210, 210, 210, 2, 0.001, 210);//1, 0, 40,
    BasePID_Init(&coreYawEncoderSpeed, 115,0,0, 30000, 10000, 5000, 0.5, 0, 30000);//-1250, 0, 0
    DualPID_Init(&heroHolder.yaw.encoderPID, &shellYawEncoderAngle, &coreYawEncoderSpeed);
    //< Yaw������Ƕ�˫��
    BasePID_Init(&shellYawAutoAngle, 10, 0, 0, 210, 0, 0, 0.5, 0, 210);
    BasePID_Init(&coreYawAutoSpeed, -2300, 0, 0, 30000, 10000, 5000, 0.5, 0, 30000);
    DualPID_Init(&heroHolder.yaw.autoPID, &shellYawAutoAngle, &coreYawAutoSpeed);
	
    /************ Pitch�SPID��ʼ�� ************/
	
	//< Pitch�Ḵλ˫��
    BasePID_Init(&shellRestAngle, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    BasePID_Init(&coreRestAngle, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	DualPID_Init(&heroHolder.pitch.resetPID, &shellRestAngle, &coreRestAngle);
    //< Pitch�S������Ƕ�˫��
    BasePID_Init(&shellPitchEncoderAngle,10, 0.1, 0, 100, 50, 100, 7, 0.001, 500);//2, 0.3, 60
    BasePID_Init(&corePitchEncoderSpeed,80, 0, 0,  30000, 2000, 8000, 0.5, 0, 30000);//1000, 2.5, 0
    DualPID_Init(&heroHolder.pitch.normalPID, &shellPitchEncoderAngle, &corePitchEncoderSpeed);
	//< Pitch�S���i�ٶȆλ�
	BasePID_Init(&heroHolder.pitch.lockPID,0, 0, 0, 0, 0, 0, 0, 0, 0);
    //< Pitch������Ƕ�˫��
    BasePID_Init(&shellPitchAutoAngle, -2, 0, 0, 100, 50, 100, 0.2, 0, 500);
    BasePID_Init(&corePitchAutoSpeed, 0, 0, 0, 30000, 2000, 8000, 0.5, 0, 30000);
    DualPID_Init(&heroHolder.pitch.autoPID, &shellPitchAutoAngle, &corePitchAutoSpeed);
	
    /************ ��PID��ʼ�� ************/
	
     //< ��Ħ�����ٶ�
    BasePID_Init(&heroShoot.booster.top.fricSpeedPID, 38, 0, 0, 8000, 8000, 8000, 30, 10, 16000);
	//< ��Ħ�����ٶ�
	BasePID_Init(&heroShoot.booster.left.fricSpeedPID, 38, 0, 0, 16000, 8000, 8000, 30, 10, 16000);
    //< ��Ħ�����ٶ�
    BasePID_Init(&heroShoot.booster.right.fricSpeedPID, 38, 0, 0, 16000, 8000, 8000, 30, 10, 16000);
    //< �������ٶ�
    BasePID_Init(&heroShoot.loader.LoadBackSpeedPID, 5, 0, 0, 8000, 1000, 100, 50, 10, 10000);
	//< �����̽ǶȻ��ֿ����ٶ�
    BasePID_Init(&shellLoadAngle, 20, 0,  0, 30000, 2000, 100, 0, 0, 15000);
	BasePID_Init(&coreLoadSpeed, 15, 0,  0, 30000, 2000, 100, 0, 0, 15000);
	DualPID_Init(&heroShoot.loader.loadPID, &shellLoadAngle,&coreLoadSpeed);	
}
