/**
 **********************************************************************************
 * @file        driver_timer.c
 * @brief       �����㣬CAN���������ļ�
 * @details     ��Ҫ����CAN��������ʼ����ָ���жϻص���������ͬCAN_ID�µ������շ�����
 * @date        2024-07-25
 * @version     V1.1
 * @copyright   Copyright (c) 2024-2124  �й���ҵ��ѧCUBOTս��
 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * @par �޸���־:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2024-03-11   <td>1.0         <td>ZengYulin   <td>������ʼ�汾
 * <tr><td>2024-07-25   <td>1.1         <td>EmberLuo    <td>
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    ��� driver_timer.h

    1. ����

 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
 */

#include "tim.h"
#include "driver_timer.h"
#include "driver_can.h"
#include "user_lib.h"

TIM_Object_t tim14 = {
    .clock_time             = 0,
    .UI_time                = 0,
    .vision_fps             = 0,
    .vision_loss_target_cnt = 0,
};

/**
 * @brief ��ʼ����ʱ�����û��ص�
 */
void TIMx_Init(TIM_HandleTypeDef *h_tim, TIM_ElapsedCallback callback)
{
    if (h_tim->Instance == TIM14) 
	{
        tim14.Handle       = h_tim;
        tim14.ElapCallback = callback;
    }
}

/**
 * @brief     ������ʱ������ж�
 */
void TIM_Open(TIM_Object_t *tim)
{
    HAL_TIM_Base_Start_IT(tim->Handle);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  /* USER CODE BEGIN Callback 1 */
	if(htim==(&htim14))
		tim14.ElapCallback();
  /* USER CODE END Callback 1 */
}

