#include "online_ctrl_task.h"
#include "user_lib.h"
#include "dr16.h"
#include "Supercap.h"
#include "brain.h"
#include "referee_task.h"
#include "chassis_task.h"
#include "holder_task.h"
#include "shoot_task.h"

/*  ���߼�⣬1������0���� */
OnlineFlag_t onlineFlag     = {0};
OnlineFlag_t onlineFlagLast = {0};

/**
 * @brief   ���߼�����ݸ���
 * 
 */
static void OnlineCNTUpdate(void)
{
    /* ��־λ���� */
    onlineFlagLast = onlineFlag;
    /* ң���� */
    rc_Ctrl.online_cnt++;
	rc_Ctrl.online_cnt = LIMIT(rc_Ctrl.online_cnt, 0, 50);
    /* ���� */
    heroChassis.wheelMotor.m3508[0].online_cnt++;
    heroChassis.wheelMotor.m3508[1].online_cnt++;
    heroChassis.wheelMotor.m3508[2].online_cnt++;
    heroChassis.wheelMotor.m3508[3].online_cnt++;
	heroChassis.wheelMotor.m3508[0].online_cnt = LIMIT(heroChassis.wheelMotor.m3508[0].online_cnt, 0, 30);
	heroChassis.wheelMotor.m3508[1].online_cnt = LIMIT(heroChassis.wheelMotor.m3508[1].online_cnt, 0, 30);
	heroChassis.wheelMotor.m3508[2].online_cnt = LIMIT(heroChassis.wheelMotor.m3508[2].online_cnt, 0, 30);
	heroChassis.wheelMotor.m3508[3].online_cnt = LIMIT(heroChassis.wheelMotor.m3508[3].online_cnt, 0, 30);
    /* ��̨ */
    heroHolder.pitch.m2006.online_cnt++;
	heroHolder.pitch.m2006.online_cnt = LIMIT(heroHolder.pitch.m2006.online_cnt, 0, 30);
    heroHolder.yaw.m4310.online_cnt++;
	heroHolder.yaw.m4310.online_cnt = LIMIT(heroHolder.pitch.m2006.online_cnt, 0, 30);
    /* �� */
    heroShoot.booster.top.m3508.online_cnt++;
	heroShoot.booster.left.m3508.online_cnt++;
    heroShoot.booster.right.m3508.online_cnt++;
	heroShoot.booster.top.m3508.online_cnt   = LIMIT(heroShoot.booster.top.m3508.online_cnt, 0, 30);
	heroShoot.booster.left.m3508.online_cnt  = LIMIT(heroShoot.booster.left.m3508.online_cnt, 0, 30);
	heroShoot.booster.right.m3508.online_cnt = LIMIT(heroShoot.booster.right.m3508.online_cnt, 0, 30);	
    /* ���� */
    heroSupercap.online_cnt++;
	heroSupercap.online_cnt = LIMIT(heroSupercap.online_cnt, 0, 30);
    /* ���� */
    cubotBrain.online_cnt++;
	cubotBrain.online_cnt = LIMIT(cubotBrain.online_cnt, 0, 30);
}

static void RunOnlineCheck(void)
{
    /* ң���� */
    if (rc_Ctrl.online_cnt > OFF_TIME) {
        onlineFlag.dbus   = 0;
        rc_Ctrl.is_online = 0;
		DR16Init(&rc_Ctrl);
//		heroShoot.shootFlag.fire = 0;
    } else {
        onlineFlag.dbus   = 1;
        rc_Ctrl.is_online = 1;
    }
    /* ����ϵͳ */
    if (referee2024.online_cnt > OFF_TIME)
        onlineFlag.referee_sys = 0;
    else
        onlineFlag.referee_sys = 1;
    /* ���� */
    if (heroChassis.wheelMotor.m3508[0].online_cnt > OFF_TIME)
        onlineFlag.wheel[0] = 0;
    else
        onlineFlag.wheel[0] = 1;
    if (heroChassis.wheelMotor.m3508[1].online_cnt > OFF_TIME)
        onlineFlag.wheel[1] = 0;
    else
        onlineFlag.wheel[1] = 1;
    if (heroChassis.wheelMotor.m3508[2].online_cnt > OFF_TIME)
        onlineFlag.wheel[2] = 0;
    else
        onlineFlag.wheel[2] = 1;
    if (heroChassis.wheelMotor.m3508[3].online_cnt > OFF_TIME)
        onlineFlag.wheel[3] = 0;
    else
        onlineFlag.wheel[3] = 1;
    /* PITCH */
    if (heroHolder.pitch.m2006.online_cnt > OFF_TIME)
        onlineFlag.pitch = 0;
    else
        onlineFlag.pitch = 1;
    /* YAW */

    /* Ħ������ */
    if (heroShoot.booster.top.m3508.online_cnt > OFF_TIME)
        onlineFlag.fric_top = 0;
    else
        onlineFlag.fric_top = 1;
    /* Ħ������ */
    if (heroShoot.booster.left.m3508.online_cnt > OFF_TIME)
        onlineFlag.fric_left = 0;
    else
        onlineFlag.fric_left = 1;
    /* Ħ������ */
    if (heroShoot.booster.right.m3508.online_cnt > OFF_TIME)
        onlineFlag.fric_right = 0;
	else
		onlineFlag.fric_right = 1;
    /* ������ */
    if (heroShoot.loader.m3508.online_cnt > OFF_TIME)
        onlineFlag.load = 0;
    else
        onlineFlag.load = 1;
    /* �Ӿ� */
    if (cubotBrain.online_cnt > OFF_TIME)
        onlineFlag.vision = 0;
    else
        onlineFlag.vision = 1;
    /* �������� */
    if (heroSupercap.online_cnt > OFF_TIME)
        onlineFlag.super_cap = 0;
    else
        onlineFlag.super_cap = 1;

}


void OnlineCtrl_Task(void)
{
        /* ���ݸ��� */
        OnlineCNTUpdate();
        /* ���߼�� */
        RunOnlineCheck();
}
