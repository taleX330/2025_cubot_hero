/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @author  annotation and modification by NeoZeng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention INS����ĳ�ʼ����Ҫ����ʵʱϵͳ!Ӧ����applicationӵ��ʵ��,�����
 *            Ӧ�ò���ó�ʼ������.
 *
 ******************************************************************************
 */
#ifndef _INS_TASK_H_
#define _INS_TASK_H_

#include "quaternions_EKF.h"
#include <stdint.h>

#define X               0
#define Y               1
#define Z               2

#define INS_TASK_PERIOD 1

typedef struct _IMU_InitData_t
{
    float accel[3];
    float gyro[3];
    float temp_when_cali;
    float temperature;
    float accel_scale;
    float gyro_offset[3];
    float g_norm;
    uint8_t (*Init)(void);
    void (*Read)(struct _IMU_InitData_t *);
} IMU_InitData_t;

typedef struct
{
    float accel[3];        // ���ٶ�
    float gyro[3];         // ���ٶ�
    float roll;            // ��ת��? \phi
    float pitch;           // �����Ǧ� \theta
    float yaw;             // ƫ���Ǧ� \psi
    float yaw_total_angle; // ���ս���õ��ĽǶ�,�Լ�yawת�����ܽǶ�(�����Ȧ����)
    float q[4];            // ��Ԫ��
} Attitude_t;

typedef struct
{
    float MotionAccel_b[3]; // ����������ٶ�
    float MotionAccel_n[3]; // ����ϵ���ٶ�
    float AccelLPF;         // ���ٶȵ�ͨ�˲�ϵ��
    // bodyframe�ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];
    // ���ٶ��ڻ���ϵ��XY����ļн�
    float atanxz;        // ���ٶ��ڻ���ϵ��X��ļн�
    float atanyz;        // ���ٶ��ڻ���ϵ��Y��ļн�
    Attitude_t attitude; // ��̬����
    uint8_t init_done;   // ��ʼ���ɹ���־λ
} INS_t;

/* ����������װ���Ĳ��� */
typedef struct
{
    uint8_t flag;
    float scale[3];
    float yaw;
    float pitch;
    float roll;
} IMU_CorrectParam_t;

/**
 * @brief IMU��ʼ��
 * 
 */
typedef uint8_t (*IMU_Init)(void);

/**
 * @brief IMU��ȡ
 * 
 */
typedef void (*IMU_Read)(IMU_InitData_t *);

/**
 * @brief ��ʼ���ߵ�����ϵͳ
 * 
 * @return uint8_t 
 */
uint8_t INS_Init(IMU_InitData_t * imu_data);


Attitude_t *INS_GetAttitude(IMU_InitData_t * imu_data);

/**
 * @brief ����ϵ������ϵ�ı任����
 *
 * @param vecBF body frame
 * @param vecEF earth frame
 * @param q
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);

/**
 * @brief ����ϵת��������ϵ
 *
 * @param vecEF
 * @param vecBF
 * @param q
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

void INS_Task(void);

extern Attitude_t *INS_attitude;
extern  IMU_InitData_t *IMU_data;
#endif
