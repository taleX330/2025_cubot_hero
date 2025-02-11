#ifndef _DRIVER_TIMER_H_
#define _DRIVER_TIMER_H_

#include "stm32h7xx_hal.h"
#include "tim.h"

typedef void (*TIM_ElapsedCallback)(void);

typedef struct
{
    TIM_HandleTypeDef *Handle;        //< 定时器句柄
    int32_t clock_time;               //< 任务定时器的计数变量
    int32_t vision_fps;               //< 自瞄数据接收帧率
    int32_t vision_loss_target_cnt;   //< 自瞄瞄不到装甲板的时间
    TIM_ElapsedCallback ElapCallback; //< 定时器溢出中断
    uint32_t UI_time;
} TIM_Object_t;

void TIMx_Init(TIM_HandleTypeDef *h_tim, TIM_ElapsedCallback callback);
void TIM_Open(TIM_Object_t *tim);
void TIM14_Callback(void);

extern TIM_Object_t tim14;
;

#endif
