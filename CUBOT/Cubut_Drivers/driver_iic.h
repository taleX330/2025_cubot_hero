#ifndef _DRIVER_IIC_H_
#define _DRIVER_IIC_H_

#include "stm32h7xx_hal.h"
#include "i2c.h"

#define I2Cx_FLAG_TIMEOUT             ((uint32_t) 1000)

int Sensors_I2C_WriteRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len,unsigned char* data_ptr);
int Sensors_I2C_ReadRegister(unsigned char slave_addr,unsigned char reg_addr,unsigned short len,unsigned char* data_ptr);
void I2C_Error(I2C_HandleTypeDef *hi2c);

#endif
