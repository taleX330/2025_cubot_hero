/**
 ******************************************************************************
 * @file    QuaternionEKF.h
 * @author  Wang Hongxi
 * @version V1.2.0
 * @date    2022/3/8
 * @brief   attitude update with gyro bias estimate and chi-square test
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _QUAT_EKF_H_
#define _QUAT_EKF_H_

#include "stm32h7xx_hal.h"
#include "kalman_filter.h"

/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

typedef struct
{
    uint8_t Initialized;
    KalmanFilter_t IMU_QuaternionEKF;
    uint8_t ConvergeFlag;
    uint8_t StableFlag;
    uint64_t ErrorCount;
    uint64_t UpdateCount;

    float q[4];        // ��Ԫ������ֵ
    float GyroBias[3]; // ��������ƫ����ֵ

    float gyro[3];
    float accel[3];

    float OrientationCosine[3];

    float accLPFcoef;
    float gyro_norm;
    float accl_norm;
    float AdaptiveGainScale;

    float roll;
    float pitch;
    float yaw;

    float yaw_total_angle;

    float Q1; // ��Ԫ�����¹�������
    float Q2; // ��������ƫ��������
    float R;  // ���ٶȼ���������

    float dt; // ��̬��������
    mat ChiSquare;
    float ChiSquare_Data[1];      // ���������⺯��
    float ChiSquareTestThreshold; // ����������ֵ
    float lambda;                 // ��������

    int16_t YawRoundCount;

    float YawAngleLast;
} QEKF_INS_t;

extern QEKF_INS_t QEKF_INS;
extern float chiSquare;
extern float ChiSquareTestThreshold;
void IMU_QuaternionEKF_Init(float *init_quaternion, float process_noise1, float process_noise2, float measure_noise, float lambda, float lpf);
void IMU_QuaternionEKF_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
