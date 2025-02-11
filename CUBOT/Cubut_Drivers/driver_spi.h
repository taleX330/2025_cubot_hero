#ifndef _DRIVER_SPI_H_
#define _DRIVER_SPI_H_

#include "gpio.h"
#include "spi.h"
#include <stdlib.h>
#include <stdint.h>

enum SPI_LIST {
    DEVICE_SPI1,
    DEVICE_SPI2,
    DEVICE_SPI3,
    DEVICE_SPI4,
    DEVICE_SPI5,
    DEVICE_SPI6,
    SPI_DEVICE_NUM,
};
/**
 * @brief   SPI����ģʽö��
 */
typedef enum {
    SPI_BLOCK_MODE, // Ĭ��ʹ������ģʽ
    SPI_IT_MODE,
    SPI_DMA_MODE,
} SPI_TXRX_MODE_e;

/**
 * @brief SPIƬѡ�źŶ�Ӧ��GPIO���ź�
 */
typedef struct
{
    GPIO_TypeDef *gpiox; // Ƭѡ�źŶ�Ӧ��GPIO,��GPIOA,GPIOB�ȵ�
    uint16_t cs_pin;     // Ƭѡ�źŶ�Ӧ�����ź�,GPIO_PIN_1,GPIO_PIN_2�ȵ�
} SPI_Chip_Select_t;

/**
 * @brief   SPI���ջ�����
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} SPI_RxBuffer_t;

/**
 * @brief	SPI���ͻ�����
 */
typedef struct
{
    uint8_t *Data;
    uint16_t Size;
} SPI_TxBuffer_t;

/**
 * @brief SPI�ӻ��ṹ��
 */
typedef struct _SPI_Slave_t {
    SPI_HandleTypeDef *spiHandler;                // SPI����handle
    SPI_TXRX_MODE_e spi_work_mode;                // ���乤��ģʽ
    SPI_RxBuffer_t rxBuffer;                      // ���ջ������ṹ��
    SPI_TxBuffer_t txBuffer;                      // ���ͻ������ṹ��
    SPI_Chip_Select_t chipSelect;                 // ƬѡGPIO���Žṹ��
    void (*RxCallBackSPI)(struct _SPI_Slave_t *); // ���ջص�����
} SPI_Slave_t;

/**
 * @brief SPIʵ���ṹ��
 */
typedef struct _SPI_Instance_t {
    uint16_t reg_slaves_num;       // �Ѿ�ע���Ƭѡ�ӻ���
    uint16_t total_slaves_num;     // �ܵĴӻ���
    SPI_HandleTypeDef *spiHandler; // SPI����handle
    SPI_Slave_t *slavesInstance;   // SPI�ӻ��豸ʵ���ṹ��ָ�룬���ڶ��岻��������
} SPI_Instance_t;

/**
 * @brief   SPI�ص�����
 */
typedef void (*SPI_RxCallback)(SPI_Slave_t *);

/**
 * @brief 
 * 
 * @param h_spi 
 * @param slave 
 * @param rxCallback 
 * @return uint8_t 
 */
uint8_t SPI_SlaveInit(SPI_HandleTypeDef *h_spi, SPI_Slave_t *slave, SPI_RxCallback rxCallback);

/**
 * @brief ��SPI�ӻ�ע�ᵽSPI���豸
 * 
 * @param master ����SPI�˿�
 * @param slave �ӻ��豸
 */
void SPI_RegisterSlave(SPI_Instance_t *master, SPI_Slave_t *slave);

/**
 * @brief ����SPIģʽ
 *
 * @param spi_ins   SPI�ӻ��ṹ��ָ��
 * @param spi_mode  ����ģʽ,��������ģʽ(block),�ж�ģʽ(IT),DMAģʽ.���SPI_TXRX_MODE_e�Ķ���
 */
void SPI_SetMode(SPI_Slave_t *spi_ins, SPI_TXRX_MODE_e spi_mode);

/**
 * @brief ͨ��spi���Ӧ�ӻ���������
 * 
 * @param spi_ins spiʵ��ָ��
 * @param ptr_data Ҫ���͵�����
 * @param len �����͵����ݳ���
 */
void SPI_Transmit(SPI_Slave_t *spi_ins, uint8_t *ptr_data, uint8_t len);

/**
 * @brief ͨ��spi�Ӵӻ���ȡ����
 * @attention �ر�ע��:�뱣֤ptr_data�ڻص�����������֮ǰ��Ȼ����������,��������֮�����Ϊ��δ�����!!!
 * 
 * @param spi_ins spiʵ��ָ��
 * @param ptr_data ��������buffer���׵�ַ
 * @param len �����յĳ���
 */
void SPI_Recv(SPI_Slave_t *spi_ins, uint8_t *ptr_data, uint8_t len);

/**
 * @brief ͨ��spi������λ�Ĵ���ͬʱ�շ�����
 * @todo  ������������ģʽ�µ�timeout����
 * @attention �ر�ע��:�뱣֤ptr_data_rx�ڻص�����������֮ǰ��Ȼ����������,��������֮�����Ϊ��δ�����!!!
 * 
 * @param spi_ins spiʵ��ָ��
 * @param ptr_data_rx �������ݵ�ַ
 * @param ptr_data_tx �������ݵ�ַ
 * @param len ����&���͵ĳ���
 */
void SPI_TransRecv(SPI_Slave_t *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);

extern SPI_Instance_t spi[6];

#endif
