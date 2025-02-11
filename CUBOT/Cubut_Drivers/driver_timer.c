/**
 **********************************************************************************
 * @file        driver_timer.c
 * @brief       驱动层，CAN外设配置文件
 * @details     主要包括CAN过滤器初始化，指定中断回调函数，不同CAN_ID下的数据收发函数
 * @date        2024-07-25
 * @version     V1.1
 * @copyright   Copyright (c) 2024-2124  中国矿业大学CUBOT战队
 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
 * @par 修改日志:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2024-03-11   <td>1.0         <td>ZengYulin   <td>创建初始版本
 * <tr><td>2024-07-25   <td>1.1         <td>EmberLuo    <td>
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    添加 driver_timer.h

    1. 创建

 **********************************************************************************
 * @attention
 * 硬件平台: STM32H750VBT \n
 * SDK版本：-++++
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
 * @brief 初始化定时器的用户回调
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
 * @brief     开启定时器溢出中断
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

