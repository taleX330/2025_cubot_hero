#include "holder_task.h"
#include "user_lib.h"
#include "brain.h"
#include "dr16.h"

Holder_t heroHolder;

/**
 * @brief ��̨��ʼ��
 */
void HolderInit(Holder_t *holder)
{
    MotorInit(&holder->pitch.m2006, 0, Motor2006, 1, CAN1, 0x206);
	DMiaoInit(&holder->yaw.m4310, can2, 1, MIT);
}

/**
 * @brief ��̨��ȡ����
 */
static void HolderGetData(Holder_t *holder)
{	
    /*�������������*/
    holder->holderAttitude = *INS_attitude;
    holder->pitch.angle    = holder->holderAttitude.pitch;
    holder->pitch.omega    = holder->holderAttitude.gyro[1];
    holder->yaw.angle      = holder->holderAttitude.yaw_total_angle - (holder->yaw.reset_angle);
    holder->yaw.omega      = holder->holderAttitude.gyro[2];
	
	/*���pitch�z�����P����*/
	if ((holder->pitch.m2006.treatedData.last_angle >= 50) && (holder->pitch.m2006.treatedData.angle <= -50))
		holder->holderCount.round_count++;
	else if ((holder->pitch.m2006.treatedData.last_angle <= -50) && (holder->pitch.m2006.treatedData.angle >= 50))
		holder->holderCount.round_count--;
	holder->pitch.delta_angle = holder->pitch.target_angle - holder->pitch.angle;
	holder->holderCount.round_count = holder->pitch.delta_angle * PITCHANGLETOROUNT;
	
	/*������ͬģʽ�x��ͬ�`����*/
	if(holder->holderFlag.micro_move == 1)
	{
		holder->yaw.remote_sens   = 0.0003f;
		holder->pitch.remote_sens = 0.0003f;
	}
	else
	{
		holder->yaw.remote_sens   = 0.003f;
		holder->pitch.remote_sens = 0.003f;
	}
		
	holder->holderCount.ready_time++;
	if(holder->holderCount.ready_time >= 20)
		holder->holderFlag.reset_ready = 1;
	if(holder->holderFlag.reset_ready == 1)
		holder->holderCount.ready_time = 20;
}

/**
 * @brief ��̨��λ����
 */
static void HoderReset(Holder_t *holder)
{
    /*������̨��λ*/
    if(holder->holderFlag.reset_ready == 1)
	{
		/*pitch*/
		holder->pitch.m2006.treatedData.motor_output = Double_Pid_Ctrl(0,holder->pitch.angle,holder->pitch.m2006.rawData.speed_rpm,&holder->pitch.resetPID);
		/*yaw*/
		
	}
    /*��̨��λ�ɹ��ж�*/
    if (ABS(holder->yaw.angle) < 5 && ABS(holder->pitch.angle) < 1) 
	{
        holder->holderCount.reset_done_cnt++;
        if (holder->holderCount.reset_done_cnt > 200) 
		{
            holder->holderCount.reset_done_cnt = 0;
            holder->holderFlag.reset_done      = 1;
			holder->yaw.reset_angle            = INS_attitude->yaw_total_angle;
        }
    } 
	else
        holder->holderFlag.reset_done = 0;
}

/**
 * @brief ������̨���˶�
 */
static void HolderCtrl(Holder_t *holder)
{
	/*pitch*/
	if((ABS(holder->holderCount.round_count) >= 5) && (ABS(rc_Ctrl.rc.ch3 - 1024) > 50))
		holder->pitch.m2006.treatedData.motor_output = Double_Pid_Ctrl( holder->pitch.target_angle,
																		holder->pitch.angle,
																		holder->pitch.m2006.rawData.speed_rpm,//Ҳ�����������ǽ��ٶȲ�֪���ĸ���
																		&holder->pitch.normalPID);
	else
	{
		holder->pitch.m2006.treatedData.motor_output = One_Pid_Ctrl(0, holder->pitch.m2006.rawData.speed_rpm, &holder->pitch.lockPID);
		holder->holderCount.round_count = 0;
	}
	/*yaw*/
	if (cubotBrain.visionFlag.open == 0)
    {
		
    } 
	else 
	{
		
    }
}

/**
 * @brief ������ֵ������������
 *
 * @param holder
 */
static void HoldertOutputCtrl(Holder_t *holder)
{
    MotorFillData(&holder->pitch.m2006, holder->pitch.m2006.treatedData.motor_output);
}

void Holder_Task(void)
{
#if HOLDER_ENABLE
	
    HolderGetData(&heroHolder);
     if (rc_Ctrl.is_online == 1) 
	{
		if(heroHolder.holderFlag.reset_done == 0) 
			HoderReset(&heroHolder);
		else 
			HolderCtrl(&heroHolder); 
		HoldertOutputCtrl(&heroHolder);
    }
	else 
	{
		heroHolder.pitch.target_angle = heroHolder.pitch.angle;
		heroHolder.yaw.target_angle = heroHolder.yaw.angle;
		MotorFillData(&heroHolder.pitch.m2006, 0);
    }
#endif
}
