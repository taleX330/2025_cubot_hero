#ifndef _DRIVER_COUNTER_H_
#define _DRIVER_COUNTER_H_

#include "stm32h7xx_hal.h"
/**
 ******************************************************************************
 * @file	bsp_dwt.h
 * @author  Wang Hongxi
 * @author  modified by NeoZng
 * @version V1.2.0
 * @date    2022/3/8
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

/**
 * @brief �ú����ڼ�������ִ��ʱ��,��λΪ��/s,����ֵΪfloat����
 *        ������Ҫ����һ��float���͵ı���,���ڴ洢ʱ����
 *        ����õ���ʱ����ͬʱ����ͨ��RTT��ӡ����־�ն�,��Ҳ���Խ����dt������ӵ��鿴
 */
#define TIME_ELAPSE(dt, code)                    \
    do                                           \
    {                                            \
        float tstart = DWT_GetTimeline_s();      \
        code;                                    \
        dt = DWT_GetTimeline_s() - tstart;       \
        LOGINFO("[DWT] " #dt " = %f s\r\n", dt); \
    } while (0)

/**
 * @brief ��ʼ��DWT,�������ΪCPUƵ��,��λMHz
 *
 * @param CPU_Freq_mHz c��Ϊ168MHz,A��Ϊ180MHz
 */
void DWT_Init(uint32_t CPU_Freq_mHz);

/**
 * @brief ��ȡ���ε���֮���ʱ����,��λΪ��/s
 *
 * @param cnt_last ��һ�ε��õ�ʱ���
 * @return float ʱ����,��λΪ��/s
 */
float DWT_GetDeltaT(uint32_t *cnt_last);

/**
 * @brief ��ȡ���ε���֮���ʱ����,��λΪ��/s,�߾���
 *
 * @param cnt_last ��һ�ε��õ�ʱ���
 * @return double ʱ����,��λΪ��/s
 */
double DWT_GetDeltaT64(uint32_t *cnt_last);

/**
 * @brief ��ȡ��ǰʱ��,��λΪ��/s,����ʼ�����ʱ��
 *
 * @return float ʱ����
 */
float DWT_GetTimeline_s(void);

/**
 * @brief ��ȡ��ǰʱ��,��λΪ����/ms,����ʼ�����ʱ��
 *
 * @return float
 */
float DWT_GetTimeline_ms(void);

/**
 * @brief ��ȡ��ǰʱ��,��λΪ΢��/us,����ʼ�����ʱ��
 *
 * @return uint64_t
 */
uint64_t DWT_GetTimeline_us(void);

/**
 * @brief DWT��ʱ����,��λΪ��/s
 * @attention �ú��������ж��Ƿ�����Ӱ��,�������ٽ����͹ر��ж�ʱʹ��
 * @note ��ֹ��__disable_irq()��__enable_irq()֮��ʹ��HAL_Delay()����,Ӧʹ�ñ�����
 *
 * @param Delay ��ʱʱ��,��λΪ��/s
 */
void DWT_Delay_s(float Delay);

/**
 * @brief DWT����ʱ���ắ��,�ᱻ����timeline��������
 * @attention �����ʱ�䲻����timeline����,����Ҫ�ֶ����øú�������ʱ����,����CYCCNT�����ʱ��ʱ���᲻׼ȷ
 */
void DWT_SysTimeUpdate(void);

#endif
