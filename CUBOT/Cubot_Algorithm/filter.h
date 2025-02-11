#ifndef __FILTER_H__
#define __FILTER_H__

#include "stm32h7xx_hal.h"

typedef struct
{
    float filter_coefficient;
    float last_output;
    float output;
    float sampling;
} LowPassFilter_t;

float LPFilter(float sampling, LowPassFilter_t *LPF);

extern LowPassFilter_t LPF_pitch_speed;
extern LowPassFilter_t LPF_pitch_vision;
extern LowPassFilter_t LPF_yaw_speed;
extern LowPassFilter_t LPF_yaw_vision;
extern LowPassFilter_t LPF_none;
extern LowPassFilter_t LPF_pitch_mpu;
extern LowPassFilter_t LPF_yaw_mpu;
extern LowPassFilter_t LPF_Fric_top;
extern LowPassFilter_t LPF_Fric_left;
extern LowPassFilter_t LPF_Fric_right;
extern LowPassFilter_t LPF_Load;

#endif
