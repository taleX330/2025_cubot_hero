#include "remote_ctrl_task.h"
#include "user_lib.h"
#include "brain.h"
#include "chassis_task.h"
#include "holder_task.h"
#include "shoot_task.h"
#include "referee_task.h"

/**
 * @brief ����ң����������
 *
 */
static void ChassisRemote_Ctrl(RC_Ctrl *rc_ctrl, MecanumChassis_t *chassis)
{
	/*---------------------------------��������ʼ---------------------------------*/
	ABS(rc_ctrl->rc.ch1 - 1024) > 10 ? (chassis->movement.delta_v_x = (rc_ctrl->rc.ch1 - 1024) - chassis->movement.v_x) : (chassis->movement.v_x);
	ABS(rc_ctrl->rc.ch0 - 1024) > 10 ? (chassis->movement.delta_v_y = (rc_ctrl->rc.ch0 - 1024) - chassis->movement.v_y) : (chassis->movement.v_y);
	// ������������ƽ��״̬
	if (chassis->movement.v_x >= 0)
		(chassis->movement.delta_v_x > 0) ? (chassis->movement.v_x += chassis->movement.delta_v_x * chassis->movement.sensitivity.v_x_sens) : (chassis->movement.v_x += chassis->movement.delta_v_x * chassis->movement.sensitivity.v_x_sens * 2);
	else
		(chassis->movement.delta_v_x > 0) ? (chassis->movement.v_x += chassis->movement.delta_v_x * chassis->movement.sensitivity.v_x_sens * 2) : (chassis->movement.v_x += chassis->movement.delta_v_x * chassis->movement.sensitivity.v_x_sens);
	// ��������ǰ������״̬
	if (chassis->movement.v_y >= 0) // ǰ����������ʱ ��С��ֵ������
		(chassis->movement.delta_v_y > 0) ? (chassis->movement.v_y += chassis->movement.delta_v_y * chassis->movement.sensitivity.v_y_sens) : (chassis->movement.v_y += chassis->movement.delta_v_y * chassis->movement.sensitivity.v_y_sens * 2);
	else
		(chassis->movement.delta_v_y > 0) ? (chassis->movement.v_y += chassis->movement.delta_v_y * chassis->movement.sensitivity.v_y_sens * 2) : (chassis->movement.v_y += chassis->movement.delta_v_y * chassis->movement.sensitivity.v_y_sens);
	// ң����ӳ�����
	if ((rc_ctrl->rc.ch1 - 1024) > 300)
		rc_ctrl->key_W = 1;
	else if ((rc_ctrl->rc.ch1 - 1024) < -300)
		rc_ctrl->key_S = 1;
	if ((rc_ctrl->rc.ch0 - 1024) > 300)
		rc_ctrl->key_D = 1;
	else if ((rc_ctrl->rc.ch0 - 1024) < -300)
		rc_ctrl->key_A = 1;
	if (ABS(rc_ctrl->rc.sw - 1024) > 200)
		rc_ctrl->key_R_flag = 1;
	else
		rc_ctrl->key_R_flag = 0;
	// ǰ����ټ��ٴ���
	if ((rc_ctrl->key_S - rc_ctrl->key_W) == 0) // ɲ��
	{
		if (rc_ctrl->chassis_y_integ > 0) // ǰ������
		{
			rc_ctrl->chassis_y_integ -= ACE_SHACHE * 15; // Ace_SenseΪ���ٶ�������
			if (rc_ctrl->chassis_y_integ < 0)
				rc_ctrl->chassis_y_integ = 0;
		}
		else // ���˼���
		{
			rc_ctrl->chassis_y_integ += ACE_SHACHE * 15;
			if (rc_ctrl->chassis_y_integ > 0)
				rc_ctrl->chassis_y_integ = 0;
		}
	}
	else // ����
	{
		rc_ctrl->chassis_y_integ += ACE_SENSE * 0.3f * (rc_ctrl->key_S - rc_ctrl->key_W); // ������ʻ
		rc_ctrl->chassis_y_integ = LIMIT(rc_ctrl->chassis_y_integ, -INTEG_LIMIT, INTEG_LIMIT);
	}
	// ���Ҽ��ټ��ٴ���
	if ((rc_ctrl->key_D - rc_ctrl->key_A) == 0)
	{
		if (rc_ctrl->chassis_x_integ > 0) // �Ҽ���
		{
			rc_ctrl->chassis_x_integ -= ACE_SHACHE * 15;
			if (rc_ctrl->chassis_x_integ < 0)
				rc_ctrl->chassis_x_integ = 0;
		}
		else // �����
		{
			rc_ctrl->chassis_x_integ += ACE_SHACHE * 15;
			if (rc_ctrl->chassis_x_integ > 0)
				rc_ctrl->chassis_x_integ = 0;
		}
	}
	else // ����
	{
		rc_ctrl->chassis_x_integ += ACE_SENSE * 0.3f * (rc_ctrl->key_A - rc_ctrl->key_D); // ������ʻ
		rc_ctrl->chassis_x_integ = LIMIT(rc_ctrl->chassis_x_integ, -INTEG_LIMIT, INTEG_LIMIT);
	}
	chassis->movement.v_x = SPEED_LIMIT * (rc_ctrl->chassis_x_integ); // ����ģʽʹ��б�º���
	chassis->movement.v_y = SPEED_LIMIT * (rc_ctrl->chassis_y_integ);
	
	// ����
	if((rc_Ctrl.rc.s2==2&&rc_Ctrl.rc.s2_last==3) || (rc_Ctrl.key_X_flag ==1 && rc_Ctrl.last_key_X_flag ==0))
		chassis->chassisFlag.follow = !chassis->chassisFlag.follow;
}
/**
 * @brief ��̨ң����������
 *
 */
static void HolderRemote_Ctrl(RC_Ctrl *rc_ctrl, CubotBrain_t *vision, Holder_t *holder)
{
	if (rc_Ctrl.rc.s2 == 1)
        vision->visionFlag.open = 1; // ��������
    else if (rc_Ctrl.rc.s2 == 3)
        vision->visionFlag.open = 0;
	
	if (holder->holderFlag.micro_move == 0)
	{
		holder->pitch.target_angle -= ((rc_ctrl->rc.ch3 - 1024) * holder->pitch.remote_sens - rc_ctrl->mouse.y * holder->pitch.mouse_sens);
		holder->yaw.target_angle   -= ((rc_ctrl->rc.ch2 - 1024) * holder->yaw.remote_sens + rc_ctrl->mouse.x * holder->yaw.mouse_sens);
	}
	else
	{
		holder->pitch.target_angle -= ((rc_ctrl->rc.ch3 - 1024) * holder->pitch.remote_sens * 0.2f - rc_ctrl->mouse.y * holder->pitch.mouse_sens);
		holder->yaw.target_angle   -= ((rc_ctrl->rc.ch2 - 1024) * holder->yaw.remote_sens * 0.2f + rc_ctrl->mouse.x * holder->yaw.mouse_sens);
	}
	
	holder->pitch.target_angle = LIMIT(holder->pitch.target_angle, -15, 45);
	holder->yaw.target_angle   = LIMIT(holder->yaw.target_angle, -90, 90);
}
/**
 * @brief ��ң����������
 *
 */
static void ShootRemote_Ctrl(RC_Ctrl *rc_ctrl, Holder_t *holder, Shoot_t *shoot)
{
	heroShoot.shootCount.shoot_count_last = heroShoot.shootCount.shoot_count;
	if ((((rc_Ctrl.mouse.press_l_flag == 1 && rc_Ctrl.mouse.last_press_l_flag == 0) 
		|| ((rc_ctrl->rc.s1 == 1) && rc_ctrl->rc.s1_last == 3) 
		|| cubotBrain.fire ==1)
		&& (shoot->shootFlag.shoot_ready == 1))
		 )//&& (referee2024.game_robot_status.shooter_id1_42mm_cooling_limit-99) > 0
	 {
	 	heroShoot.shootFlag.fire = 1;
	 	heroShoot.shootCount.shoot_count++;
	 }
}
static void ThisKeyToLast(void)
{
	rc_Ctrl.rc.s1_last = rc_Ctrl.rc.s1;
	rc_Ctrl.rc.s2_last = rc_Ctrl.rc.s2;
	rc_Ctrl.mouse.last_press_l_flag = rc_Ctrl.mouse.press_l_flag;
	rc_Ctrl.mouse.last_press_r_flag = rc_Ctrl.mouse.press_r_flag;
	rc_Ctrl.last_key_W_flag = rc_Ctrl.key_W_flag;
	rc_Ctrl.last_key_A_flag = rc_Ctrl.key_A_flag;
	rc_Ctrl.last_key_S_flag = rc_Ctrl.key_S_flag;
	rc_Ctrl.last_key_D_flag = rc_Ctrl.key_D_flag;
	rc_Ctrl.last_key_shift_flag = rc_Ctrl.key_shift_flag;
	rc_Ctrl.last_key_ctrl_flag = rc_Ctrl.key_ctrl_flag;
	rc_Ctrl.last_key_Q_flag = rc_Ctrl.key_Q_flag;
	rc_Ctrl.last_key_E_flag = rc_Ctrl.key_E_flag;
	rc_Ctrl.last_key_V_flag = rc_Ctrl.key_V_flag;
	rc_Ctrl.last_key_F_flag = rc_Ctrl.key_F_flag;
	rc_Ctrl.last_key_G_flag = rc_Ctrl.key_G_flag;
	rc_Ctrl.last_key_C_flag = rc_Ctrl.key_C_flag;
	rc_Ctrl.last_key_R_flag = rc_Ctrl.key_R_flag;
	rc_Ctrl.last_key_B_flag = rc_Ctrl.key_B_flag;
	rc_Ctrl.last_key_Z_flag = rc_Ctrl.key_Z_flag;
	rc_Ctrl.last_key_X_flag = rc_Ctrl.key_X_flag;
}
/**
 * @brief ң��������
 *
 */
void RemoteCtrl_Task(void)
{
		ChassisRemote_Ctrl(&rc_Ctrl, &heroChassis);
		HolderRemote_Ctrl(&rc_Ctrl, &cubotBrain, &heroHolder);
		ShootRemote_Ctrl(&rc_Ctrl, &heroHolder, &heroShoot);
		ThisKeyToLast();
}
