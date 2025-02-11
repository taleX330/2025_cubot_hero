#include "filter.h"

/**
 * @brief 
 * @note LPF_noneÊÇ²»¿ªÂË²¨Æ÷
 */
LowPassFilter_t LPF_none = {
	.filter_coefficient = 1,
	.last_output = 0,
};
LowPassFilter_t LPF_pitch_speed = {
	.filter_coefficient = 1.0f,
	.last_output = 0,
};
LowPassFilter_t LPF_yaw_speed = {
	.filter_coefficient = 1.5f,
	.last_output = 0,
};
LowPassFilter_t LPF_pitch_vision = {
	.filter_coefficient = 1.0f,
	.last_output = 0,
};
LowPassFilter_t LPF_yaw_vision = {
	.filter_coefficient = 1.0f,
	.last_output = 0,
};
LowPassFilter_t LPF_pitch_mpu = {
	.filter_coefficient = 0.2f,
	.last_output = 0,
};
LowPassFilter_t LPF_yaw_mpu = {
	.filter_coefficient = 0.15f,
	.last_output = 0,
};
LowPassFilter_t LPF_Fric_top = {
	.filter_coefficient = 0.30f,
	.last_output = 0,
};
LowPassFilter_t LPF_Fric_left = {
	.filter_coefficient = 0.30f,
	.last_output = 0,
};
LowPassFilter_t LPF_Fric_right = {
	.filter_coefficient = 0.30f,
	.last_output = 0,
};
LowPassFilter_t LPF_Load = {
	.filter_coefficient = 0.60f,
	.last_output = 0,
};

/**
 * @brief Ò»½×µÍÍ¨ÂË²¨Æ÷
 *
 * @param sampling
 * @param LPF
 * @return float
 */
float LPFilter(float sampling, LowPassFilter_t *LPF)
{
	// Ò»½×µÍÍ¨ÂË²¨Æ÷£ºp(n) = c¡¤q(n) + (1 - c)¡¤p(n - 1)
	(*LPF).sampling = sampling;

	(*LPF).output = (*LPF).filter_coefficient * (*LPF).sampling + (1 - (*LPF).filter_coefficient) * (*LPF).last_output;

	(*LPF).last_output = (*LPF).output;

	return (*LPF).output;
};
