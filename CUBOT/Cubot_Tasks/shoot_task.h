#ifndef _SHOOTTASK_H_
#define _SHOOTTASK_H_

#include "stm32h7xx_hal.h"
#include "rm_motor.h"
#include "pid.h"
#include "dr16.h"
#include "ladrc.h"

#define SHOOT_ENABLE 1

/**
 * @brief Ħ���ֵ������
 *
 */
typedef struct
{
    Motor_t m3508;                // ����Ĳ���������
    int16_t target_speed_config;  // ����Ŀ��ת��
    int16_t target_speed_current; // ��ǰĿ��ת��
	Ladrc_t fricSpeedLadrc;		  // Ħ�����ٶȿ���LADRC����
    SinglePID_t fricSpeedPID;     // Ħ�����ٶȿ���PID����
} FricInstance_t;
/**
 * @brief ������
 *
 */
typedef struct
{
    // ��־λ
    struct
    {
		uint8_t fric_ready;				  // Ħ����׼�����
		uint8_t shoot_ready;			  // ΢�����ر�־λ
        uint8_t fire;					  // ����ָ��
        uint8_t jam;                      // ��·����
		uint8_t load_start;               // ����������
		uint8_t fric_close;               // Ħ����ֹͣ
    } shootFlag;    
    // ����
    struct 
	{
        uint16_t shoot_count; // �򵯼�������һ�μ�һ����ʼ�����л����ģʽ������
        uint16_t shoot_count_last;
        struct
        {
            float slowopen_time;  // Ħ���ֻ�����ʱ��
            float slowclose_time; // Ħ���ֻ��ر�ʱ��
        } fric;
        struct
        {
            uint16_t jammed_time;  		 // �����̿����жϼ�ʱ
            uint16_t load_turnback_time; // �����̷�תʱ��
            uint16_t filled_time;        // ��·�����жϼ�ʱ
			uint32_t load_time;          // ��·�����жϼ�ʱ
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
		Motor_t     m3508;                 // ����Ĳ���������
        int16_t     backward_speed;       // �󲦵��̷�תĿ���ٶ�
		DualPID_t   loadPID;
        SinglePID_t LoadBackSpeedPID; 
    } loader;
} Shoot_t;

extern Shoot_t heroShoot;

void Shoot_Task(void);
void ShootInit(Shoot_t *shoot);
#endif

