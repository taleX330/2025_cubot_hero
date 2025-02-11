/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins.h"
#include "user_lib.h"
#include "bmi088.h"
#include "mpu6050.h"
#include "fast_math_functions.h"
Attitude_t *INS_attitude;
IMU_InitData_t *IMU_data;
static INS_t ins                           = {0};
static IMU_CorrectParam_t IMU_paramCorrect = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// ���ڻ�ȡ���β���֮���ʱ����
static uint32_t INS_DWT_Count = 0;

static void IMU_Param_Correction(IMU_CorrectParam_t *param, float gyro[3], float accel[3]);

// ʹ�ü��ٶȼƵ����ݳ�ʼ��Roll��Pitch,��Yaw��0,�������Ա����ڳ�ʼʱ�����̬�������
/**
 * @brief
 *
 * @param init_q4
 */
static void InitQuaternion(float *init_q4, IMU_InitData_t *imu_data)
{
    float acc_init[3]     = {0};
    float gravity_norm[3] = {0, 0, 1}; // ����ϵ�������ٶ�ʸ��,��һ����Ϊ(0,0,1)
    float axis_rot[3]     = {0};       // ��ת��
    // ��ȡ100�μ��ٶȼ�����,ȡƽ��ֵ��Ϊ��ʼֵ
    for (uint8_t i = 0; i < 100; ++i) {
        imu_data->Read(imu_data);
        acc_init[X] += imu_data->accel[X];
        acc_init[Y] += imu_data->accel[Y];
        acc_init[Z] += imu_data->accel[Z];
        DWT_Delay_s(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // ����ԭʼ���ٶ�ʸ���͵���ϵ�������ٶ�ʸ���ļн�
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // ��ǹ�ʽ,������Ϊ0(û��z�����)
}
// ����ʼ�������Ͷ�ȡ��������ṹ��
static void IMU_FuncInit(void)
{
	bmi088.bmi088_Data.Init   = BMI088_Init;
    bmi088.bmi088_Data.Read   = BMI088_Read;
    mpu6050.mpu6050_Data.Init = MPU6050_Init;
    mpu6050.mpu6050_Data.Read = MPU6050_Read;
}
/**
 * @brief ��ʼ���ߵ�����ϵͳ
 *
 * @return uint8_t
 */
uint8_t INS_Init(IMU_InitData_t *imu_data)
{
    IMU_FuncInit();
	IMU_data = imu_data;
    if (ins.init_done)
        return ins.init_done;
    else {
        while (imu_data->Init() != 0) ;
        IMU_paramCorrect.scale[X] = 1;
        IMU_paramCorrect.scale[Y] = 1;
        IMU_paramCorrect.scale[Z] = 1;
        IMU_paramCorrect.yaw      = 0;
        IMU_paramCorrect.pitch    = 0;
        IMU_paramCorrect.roll     = 0;
        IMU_paramCorrect.flag     = 1;

        float init_quaternion[4] = {0};
        InitQuaternion(init_quaternion, imu_data);
        IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
        // imu heat init

        // noise of accel is relatively big and of high freq,thus lpf is used
        ins.AccelLPF = 0.0085;
        DWT_GetDeltaT(&INS_DWT_Count);
        ins.init_done = 1;
		
        return ins.init_done;
    }
}
/**
 * @brief INS����
 * @attention �˺�������ʵʱϵͳ��,��1kHzƵ�����С�p.s. osDelay(1);
 *
 * @return Attitude_t*
 */
Attitude_t *INS_GetAttitude(IMU_InitData_t *imu_data)
{
    /* ע����1kHz��Ƶ�����д����� */
    static Attitude_t insAttitude = {0};
    static float dt = 0, t = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    imu_data->Read(imu_data);

    ins.attitude.accel[X] = imu_data->accel[X];
    ins.attitude.accel[Y] = imu_data->accel[Y];
    ins.attitude.accel[Z] = imu_data->accel[Z];
    ins.attitude.gyro[X]  = imu_data->gyro[X];
    ins.attitude.gyro[Y]  = imu_data->gyro[Y];
    ins.attitude.gyro[Z]  = imu_data->gyro[Z];

    // demo function,����������װ���,���Բ���,��ʱû��
    IMU_Param_Correction(&IMU_paramCorrect, ins.attitude.gyro, ins.attitude.accel);

    // �����������ٶ�ʸ����bϵ��XY����ļн�,������������չ,��ʱû��
    ins.atanxz = -atan2f(ins.attitude.accel[X], ins.attitude.accel[Z]) * 180 / PI;
    ins.atanyz = atan2f(ins.attitude.accel[Y], ins.attitude.accel[Z]) * 180 / PI;

    // ���ĺ���,EKF������Ԫ��
    IMU_QuaternionEKF_Update(ins.attitude.gyro[X], ins.attitude.gyro[Y], ins.attitude.gyro[Z], ins.attitude.accel[X], ins.attitude.accel[Y], ins.attitude.accel[Z], dt);

    memcpy(ins.attitude.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // ����ϵ������ת������������ϵ��ѡȡ����ϵΪ����ϵ
    BodyFrameToEarthFrame(xb, ins.xn, ins.attitude.q);
    BodyFrameToEarthFrame(yb, ins.yn, ins.attitude.q);
    BodyFrameToEarthFrame(zb, ins.zn, ins.attitude.q);

    // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, ins.attitude.q);
    for (uint8_t i = 0; i < 3; ++i) // ͬ����һ����ͨ�˲�
    {
        ins.MotionAccel_b[i] = (ins.attitude.accel[i] - gravity_b[i]) * dt / (ins.AccelLPF + dt) + ins.MotionAccel_b[i] * ins.AccelLPF / (ins.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(ins.MotionAccel_b, ins.MotionAccel_n, ins.attitude.q); // ת���ص���ϵn

    ins.attitude.yaw             = QEKF_INS.yaw;
    ins.attitude.pitch           = QEKF_INS.pitch;
    ins.attitude.roll            = QEKF_INS.roll;
    ins.attitude.yaw_total_angle = QEKF_INS.yaw_total_angle;
    insAttitude                  = ins.attitude;

    return &insAttitude;
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.��������IMU��װ��������������,�������������̨��İ�װƫ��
 *
 * @param param IMU����
 * @param gyro  ���ٶ�
 * @param accel ���ٶ�
 */
static void IMU_Param_Correction(IMU_CorrectParam_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->yaw - lastYawOffset) > 0.001f ||
        fabsf(param->pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->roll - lastRollOffset) > 0.001f || param->flag) {
        cosYaw   = arm_cos_f32(param->yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->pitch / 57.295779513f);
        cosRoll  = arm_cos_f32(param->roll / 57.295779513f);
        sinYaw   = arm_sin_f32(param->yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->pitch / 57.295779513f);
        sinRoll  = arm_sin_f32(param->roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11        = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12        = cosPitch * sinYaw;
        c_13        = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21        = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22        = cosYaw * cosPitch;
        c_23        = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31        = -cosPitch * sinRoll;
        c_32        = sinPitch;
        c_33        = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset   = param->yaw;
    lastPitchOffset = param->pitch;
    lastRollOffset  = param->roll;
}

uint32_t INS_HighWaterMark;
void INS_Task(void)
{
    while (1) {
//        INS_attitude = INS_GetAttitude(IMU_data);
//#if INCLUDE_uxTaskGetStackHighWaterMark
//        INS_HighWaterMark = uxTaskGetStackHighWaterMark(NULL);
//#endif
//        osDelay(1);
    }
}
