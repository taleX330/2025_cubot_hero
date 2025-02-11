#ifndef _PID_H_
#define _PID_H_

#include "stm32h7xx_hal.h"

/**
 * @brief µ¥»·PID
 *
 */
typedef struct
{
	float P;
	float I;
	float D;
	float delta;
	float delta_last;
	float delta_last_last;
	float p_part;
	float p_part_maxlimit;
	float i_part;
	float i_part_maxlimit;
	float i_part_detach_lower;
	float i_part_detach_upper;
	float i_delta_sum;
	float d_part;
	float d_part_maxlimit;
	float max_limit;
	float out;
} SinglePID_t;
/**
 * @brief Ë«»·PID
 *
 */
typedef struct
{
	struct
	{
		float shell_P;
		float shell_I;
		float shell_D;
		float shell_delta;
		float shell_delta_last;
		float shell_p_part;
		float shell_p_part_maxlimit;
		float shell_i_part;
		float shell_i_part_maxlimit;
		float shell_i_part_detach_lower;
		float shell_i_part_detach_upper;
		float shell_d_part;
		float shell_d_part_maxlimit;
		float shell_max_limit;
		float shell_out;
	} shell;
	struct
	{
		float core_P;
		float core_I;
		float core_D;
		float core_delta;
		float core_delta_last;
		float core_p_part;
		float core_p_part_maxlimit;
		float core_i_part;
		float core_i_part_maxlimit;
		float core_i_part_detach_lower;
		float core_i_part_detach_upper;
		float core_d_part;
		float core_d_part_maxlimit;
		float core_max_limit;
		float core_out;
	} core;
} DualPID_t;

void BasePID_Init_All(void);
float One_Pid_Ctrl(float target, float feedback, SinglePID_t *PID);
float Double_Pid_Ctrl(float shell_target, float shell_feedback, float core_feedback, DualPID_t *PID);

#endif
