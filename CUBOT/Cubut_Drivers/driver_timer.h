#ifndef _DRIVER_TIMER_H_
#define _DRIVER_TIMER_H_

#include "stm32h7xx_hal.h"
#include "tim.h"

typedef void (*TIM_ElapsedCallback)(void);

typedef struct
{
    TIM_HandleTypeDef *Handle;        //< ��ʱ�����
    int32_t clock_time;               //< ����ʱ���ļ�������
    int32_t vision_fps;               //< �������ݽ���֡��
    int32_t vision_loss_target_cnt;   //< �����鲻��װ�װ��ʱ��
    TIM_ElapsedCallback ElapCallback; //< ��ʱ������ж�
    uint32_t UI_time;
} TIM_Object_t;

void TIMx_Init(TIM_HandleTypeDef *h_tim, TIM_ElapsedCallback callback);
void TIM_Open(TIM_Object_t *tim);
void TIM14_Callback(void);

extern TIM_Object_t tim14;
;

#endif
