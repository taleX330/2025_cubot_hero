#ifndef _BMI088_H_ // ��ֹ�ظ�����
#define _BMI088_H_

#include "driver_spi.h"
#include "driver_counter.h"
//#include "pid.h"
#include "ins.h"
#include "bmi088_regNdef.h"

// ���ֶ�У׼���޸�����
#define BMI088_GxOFFSET (-0.00418030564f)
#define BMI088_GyOFFSET (-0.0052329856f)
#define BMI088_GzOFFSET (-0.000287621311f)
#define BMI088_gNORM    10.2522392f

#define BMI088_USE_SPI

typedef struct {
    SPI_Slave_t bmi088Accel;
    SPI_Slave_t bmi088Gyro;
    IMU_InitData_t bmi088_Data;
} BMI088_t;

/**
 * @brief ��ʼ��BMI088,�������ӵ�SPI����handle,�Լ��Ƿ�������߱궨
 *
 * @param bmi088_SPI handle
 * @param calibrate  1Ϊ�������߱궨,0ʹ����������
 * @return uint8_t   �ɹ��򷵻�BMI088_NO_ERROR
 */
extern uint8_t BMI088_Init(void);

/**
 * @brief ���ټƳ�ʼ��
 *
 * @return uint8_t
 */
extern uint8_t BMI088_AccInit(void);

/**
 * @brief �����ǳ�ʼ��
 *
 * @return uint8_t
 */
extern uint8_t BMI088_GyroInit(void);

/**
 * @brief ��ȡһ��BMI088������,����gyro��accel
 *
 * @param bmi088_data ����BMI088ʵ��(�ṹ��)
 */
extern void BMI088_Read(IMU_InitData_t *bmi088_data);

extern BMI088_t bmi088;

#if defined(BMI088_USE_SPI)

#elif defined(BMI088_USE_IIC)

#endif

#endif
