/**
 **********************************************************************************
 * @file  		driver_iic.c
 * @brief    	�弶֧�ְ������ڹ����������ļ����û��ص��ض���
 * @details  	��Ҫ�����������ڹ��������ṩ���ڳ�ʼ�����û��ص��ض���
 * @author      RyanJiao  any question please send mail to 1095981200@qq.com
 * @date        2022-2-20
 * @version     V1.1
 * @copyright   Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * @par �޸���־:
 * <table>
 * <tr><th>Date        	<th>Version  	<th>Author    	<th>Description
 * <tr><td>2021-8-12  	<td>1.0      	<td>RyanJiao  	<td>������ʼ�汾
 * </table>
 *
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
#include "driver_iic.h"

/**
 * @brief  д�Ĵ����������ṩ���ϲ�Ľӿ�
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len,
                              unsigned char *data_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    status                   = HAL_I2C_Mem_Write(&hi2c2, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);

    /* ���ͨѶ״̬ */
    if (status != HAL_OK) {
        /* ���߳����� */
        I2C_Error(&hi2c2);
    }
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
    }
    /* ���SENSOR�Ƿ����������һ�ζ�д���� */
    while (HAL_I2C_IsDeviceReady(&hi2c2, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);

    /* �ȴ�������� */
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
    }

    return 0;
}

/**
 * @brief  ���Ĵ����������ṩ���ϲ�Ľӿ�
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                             unsigned char reg_addr,
                             unsigned short len,
                             unsigned char *data_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    status                   = HAL_I2C_Mem_Read(&hi2c2, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);
    /* ���ͨѶ״̬ */
    if (status != HAL_OK) {
        /* ���߳����� */
        I2C_Error(&hi2c2);
    }
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
    }
    /* ���SENSOR�Ƿ����������һ�ζ�д���� */
    while (HAL_I2C_IsDeviceReady(&hi2c2, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT);
    /* �ȴ�������� */
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY) {
    }

    return status;
}

void I2C_Error(I2C_HandleTypeDef *hi2c)
{
    /* �ָ�I2C�Ĵ���ΪĬ��ֵ */
    HAL_I2C_DeInit(hi2c);
    /* ���³�ʼ��I2C���� */
    HAL_I2C_Init(hi2c);
}
