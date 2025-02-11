#ifndef _CHASSISTASK_H_
#define _CHASSISTASK_H_

#include "stm32h7xx_hal.h"
#include "user_lib.h"
#include "pid.h"
#include "rm_motor.h"


#define CHASSIS_ENABLE       1
#define CHASSIS_CURRENT_MAX  64000         // �����װ�أ������е������ĵ���ֵ����Ƕ���
#define TORQUE_COEFFICIENT   1.99688994e-6f // (20/16384)*(0.3)*(187/3591)/9.55������9.55Լ����60/2pi
#define CONSTANT_COEFFICIENT 1.381f

/**
 * @brief  ���ֵ��̽ṹ��
 */
typedef struct
{
    struct
    {
        uint8_t  CapState;      // ����״̬
		uint8_t  follow;        // ����
		uint8_t  cat_walk;      // ؈��
		uint16_t cat_walk_cnt;
    } chassisFlag;

    struct
    {
        Motor_t     m3508[4];             // ���̵���ṹ��
        SinglePID_t chassisSpeedPID[4];  // �ٶȿ���PID����
        SinglePID_t chassisFollowPID; // ���̸���PID����
    } wheelMotor;

    struct
    {
        int16_t v_x;   // ǰ���˶����ٶ�
        int16_t v_y;   // �����˶����ٶ�
        int16_t omega; // ��ת�Ľ��ٶ�
        int16_t delta_v_x;
        int16_t delta_v_y;
        int16_t rotated_v_x;
        int16_t rotated_v_y;
        int16_t target_speed[4];   // ���������δ���й��ʿ��Ƶ��ٶ�
        struct
        {
            float v_x_sens;
            float v_y_sens;
        } sensitivity;
    } movement;

    struct
    {
        float speed_limit;              // ���ݲ�ͬ�ȼ����̹������Ƶõ����ٶ�
        float theoretical_power_sum;    // ���û�������������10J����Ҫ�ĵ����ܹ�������ֵ
        float target_require_power_sum; // ֱ�ӽ����ٶ�PID�����õ��Ŀ��Ƶ���ֵ֮�ͣ����Ա�ʾ����ʵ��ֵ
        float scaling_ratio;            // ���ű��� = ��������ֵ / ����ʵ��ֵ
        float initial_give_power[4];    // initial power from PID calculation
        float scaled_give_power[4];
        float torque_term_k1;           // k1
        float speed_term_k2;            // k2
        SinglePID_t chassisPowerPID;    // ���ʿ���PID����
        struct
        {
            float    real_time_power;          // ʵʱ����
            uint8_t  powermngmt_chassis_out;   // ��ܵ��̽ӿڵ����
            uint16_t energy_buffer;            // ��������
            uint16_t max_power;                // ��������
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
