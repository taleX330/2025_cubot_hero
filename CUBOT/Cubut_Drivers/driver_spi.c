/**
 **********************************************************************************
 * @file        driver_spi.c
 * @brief       �����㣬����SPI�ӻ��ĳ�ʼ�����������̣������ο�HNU�Ŀ�Դ�Դ���������޸�
 * @details
 * @date        2024-07-26
 * @version     V1.0
 * @copyright   Copyright CUMT-CUBOT (c) 2024
 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * @par �޸���־:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2024-07-26   <td>1.0         <td>EmberLuo    <td>������ʼ�汾
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                            How to use this driver
 ==============================================================================

    1. Include the "driver_spi.h" header file in your project.
        �������Ŀ�а���"driver_spi.h"ͷ�ļ���
    2. Create an instance of SPI_Slave_t for each SPI slave device you want to communicate with.
        ΪҪ��֮ͨ�ŵ�ÿ��SPI���豸����һ��SPI_Slave_tʵ����
    3. Initialize each SPI slave using the SPI_Slavelnit() function, providing the necessary parameters.
        ʹ��SPI_Slavelnit()������ʼ��ÿ��SPI�������ṩ��Ҫ�Ĳ�������SPI������������úͽ��ջص�������
    4. Set the SPI mode for each slave using SPL_SetMode(), choosing between DMA, interrupt, or blocking modes.
        ʹ��SPL_SetMode()����ÿ���ӻ���SPIģʽ����DMA���жϻ�����ģʽ֮�����ѡ��
    5. Use the provided functions to communicate with the SPI slaves:SPI_Transmit() to send data SPI_Recv() to receive data SPI_TransRecv() for fullduplex communication.
        ʹ���ṩ�ĺ�����SPI��վͨ��:SPI_Transmit()��������SPI_Recv()��������SPI_TransRecv()����ȫ˫��ͨ��
    6. Implement the necessary callback functions to handle received data or transmission completion events.
        ʵ�ֱ�Ҫ�Ļص�������������յ������ݻ�������¼���

 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * if you had modified this file, please make sure your code does not have many
 * bugs, update the version NO., write dowm your name and the date, the most
 * important is make sure the users will have clear and definite understanding
 * through your new brief.
 **********************************************************************************
 * @exception   SPI_NSS������ģʽ��SPI_NSS_Hard��SPI_NSS_Soft��
 *              SPI_NSS_Hard��Ӳ���Զ���������Ƭѡ������������Զ�������ʽ����Ҫ�ߵģ�
 *              ȱ���ǵ�STM32Ϊ���豸ʱ��ͬһ��SPI����ֻ�ܽ�һ�����豸��
 *              SPI_NSS_Soft��������ƣ�GPIO����Ƭѡ�������ͣ��ŵ���һ��SPI������ԹҶ���豸��
 *              SPI_NSS_SoftҲ��ʹ�ý϶�ķ�ʽ, ��������ʱֻ�������ģʽ��
 * @todo        ���ǽ��ӻ���������
 **********************************************************************************
 */

#include "driver_spi.h"

SPI_Instance_t spi[6] = {0};

static uint8_t SPI_SerialNumSelect(SPI_HandleTypeDef *h_spi);

uint8_t SPI_SlaveInit(SPI_HandleTypeDef *h_spi, SPI_Slave_t *slave, SPI_RxCallback rxCallback)
{
    uint8_t spi_case         = SPI_SerialNumSelect(h_spi);
    slave->spiHandler        = h_spi;
    slave->RxCallBackSPI     = rxCallback;
    spi[spi_case].spiHandler = h_spi;
    spi[spi_case].total_slaves_num++;
    if (spi[spi_case].slavesInstance == NULL) {
        spi[spi_case].slavesInstance = (SPI_Slave_t *)malloc(spi[spi_case].total_slaves_num * sizeof(SPI_Slave_t)); // ��̬�����ڴ�
    } else {
        spi[spi_case].slavesInstance = (SPI_Slave_t *)realloc(spi[spi_case].slavesInstance, spi[spi_case].total_slaves_num * sizeof(SPI_Slave_t)); // ��̬���·����ڴ�
    }
    if (spi[spi_case].slavesInstance == NULL) {
        return 1; // �ڴ����ʧ��
    }
    SPI_RegisterSlave(&spi[spi_case], slave);
    return 0;
}

/**
 * @brief ��SPI�ӻ�ע�ᵽSPI���豸
 *
 * @param master    SPIʵ���ṹ��
 * @param slave     SPI�ӻ��ṹ��
 */
void SPI_RegisterSlave(SPI_Instance_t *master, SPI_Slave_t *slave)
{
    // ���ôӻ��Ƿ��Ѿ�ע���
    for (uint16_t i = 0; i < master->reg_slaves_num; i++) {
        if (&master->slavesInstance[i] == slave) {
            // �ôӻ��Ѿ�ע�����������ע����Ϣ
            master->slavesInstance[i] = *slave;
            return;
        }
    }
    // ����Ƿ���δע��Ĵӻ�
    if (master->reg_slaves_num >= master->total_slaves_num) {
        return; // �Ѿ�û��δע��Ĵӻ�
    }
    // ���ӻ��ṹ�帴�Ƶ����豸SPIʵ���ṹ����
    master->slavesInstance[master->reg_slaves_num] = *slave;
    // ������ע��Ĵӻ���
    master->reg_slaves_num++;
}

/**
 * @brief ����SPIģʽ
 *
 * @param spi_ins   SPI�ӻ��ṹ��
 * @param spi_mode  SPIģʽö��
 */
void SPI_SetMode(SPI_Slave_t *spi_ins, SPI_TXRX_MODE_e spi_mode)
{
    if (spi_mode != SPI_DMA_MODE && spi_mode != SPI_IT_MODE && spi_mode != SPI_BLOCK_MODE)
        while (1); // error mode! ��鿴�Ƿ���ȷ����ģʽ�������ָ��Խ�絼��ģʽ���쳣�޸ĵ����
    spi_ins->spi_work_mode = spi_mode;
}

void SPI_Transmit(SPI_Slave_t *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // ����Ƭѡ,��ʼ����(ѡ�дӻ�)
    HAL_GPIO_WritePin(spi_ins->chipSelect.gpiox, spi_ins->chipSelect.cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            HAL_SPI_Transmit_DMA(spi_ins->spiHandler, ptr_data, len);
            break;
        case SPI_IT_MODE:
            HAL_SPI_Transmit_IT(spi_ins->spiHandler, ptr_data, len);
            break;
        case SPI_BLOCK_MODE:
            HAL_SPI_Transmit(spi_ins->spiHandler, ptr_data, len, 1000); // Ĭ��50ms��ʱ
            // ����ģʽ������ûص�����,������ɺ�ֱ������Ƭѡ����
            HAL_GPIO_WritePin(spi_ins->chipSelect.gpiox, spi_ins->chipSelect.cs_pin, GPIO_PIN_SET);
            break;
        default:
            while (1); // error mode! ��鿴�Ƿ���ȷ����ģʽ�������ָ��Խ�絼��ģʽ���쳣�޸ĵ����
    }
}

void SPI_Recv(SPI_Slave_t *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // �����Ժ�ص�ʹ��
    spi_ins->rxBuffer.Size = len;
    spi_ins->rxBuffer.Data = ptr_data;
    // ����Ƭѡ,��ʼ����
    HAL_GPIO_WritePin(spi_ins->chipSelect.gpiox, spi_ins->chipSelect.cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            HAL_SPI_Receive_DMA(spi_ins->spiHandler, ptr_data, len);
            break;
        case SPI_IT_MODE:
            HAL_SPI_Receive_IT(spi_ins->spiHandler, ptr_data, len);
            break;
        case SPI_BLOCK_MODE:
            HAL_SPI_Receive(spi_ins->spiHandler, ptr_data, len, 1000);
            // ����ģʽ������ûص�����,������ɺ�ֱ������Ƭѡ����
            HAL_GPIO_WritePin(spi_ins->chipSelect.gpiox, spi_ins->chipSelect.cs_pin, GPIO_PIN_SET);
            break;
        default:
            while (1); // error mode! ��鿴�Ƿ���ȷ����ģʽ�������ָ��Խ�絼��ģʽ���쳣�޸ĵ����
    }
}

void SPI_TransRecv(SPI_Slave_t *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    // �����Ժ�ص�ʹ��,�뱣֤ptr_data_rx�ڻص�����������֮ǰ��Ȼ����������,��������֮�����Ϊ��δ�����!!!
    spi_ins->rxBuffer.Size = len;
    spi_ins->rxBuffer.Data = ptr_data_rx;
    // ����Ƭѡ,��ʼ����
    HAL_GPIO_WritePin(spi_ins->chipSelect.gpiox, spi_ins->chipSelect.cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode) {
        case SPI_DMA_MODE:
            HAL_SPI_TransmitReceive_DMA(spi_ins->spiHandler, ptr_data_tx, ptr_data_rx, len);
            break;
        case SPI_IT_MODE:
            HAL_SPI_TransmitReceive_IT(spi_ins->spiHandler, ptr_data_tx, ptr_data_rx, len);
            break;
        case SPI_BLOCK_MODE:
            HAL_SPI_TransmitReceive(spi_ins->spiHandler, ptr_data_tx, ptr_data_rx, len, 1000); // Ĭ��50ms��ʱ
            // ����ģʽ������ûص�����,������ɺ�ֱ������Ƭѡ����
            HAL_GPIO_WritePin(spi_ins->chipSelect.gpiox, spi_ins->chipSelect.cs_pin, GPIO_PIN_SET);
            break;
        default:
            while (1); // error mode! ��鿴�Ƿ���ȷ����ģʽ�������ָ��Խ�絼��ģʽ���쳣�޸ĵ����
    }
}

/**
 * @brief ��SPI�������,������ô˻ص�����,���Խ���Э�������������������ݴ����
 *
 * @param h_spi spi handle
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *h_spi)
{
    uint8_t spi_case = SPI_SerialNumSelect(h_spi);
    for (size_t i = 0; i < spi[spi_case].total_slaves_num; i++) {
        // ����ǵ�ǰspiӲ��������complete,��cs_pinΪ�͵�ƽ(˵�����ڴ���),���Ե��ûص�����
        if (HAL_GPIO_ReadPin(spi[spi_case].slavesInstance[i].chipSelect.gpiox, spi[spi_case].slavesInstance[i].chipSelect.cs_pin) == GPIO_PIN_RESET) {
            // ������Ƭѡ,��������,���ж��Ƿ��лص�����,���������ûص�����
            HAL_GPIO_WritePin(spi[spi_case].slavesInstance[i].chipSelect.gpiox, spi[spi_case].slavesInstance[i].chipSelect.cs_pin, GPIO_PIN_SET);
            // @todo �������holdonģʽ,���û����о�����ʱ�ͷ�Ƭѡ,���������������
            if (spi[spi_case].slavesInstance[i].RxCallBackSPI != NULL) // �ص�������Ϊ��, ����ûص�����
                spi[spi_case].slavesInstance[i].RxCallBackSPI(&spi[spi_case].slavesInstance[i]);
            return;
        }
    }
}

/**
 * @brief   ���Ƕ�HAL���__weak��������д,����ʹ��IT��DMAģʽ,�ڴ������ʱ����ô˺�����
 * @details ��ȫ˫��ģʽ��ʹ�� HAL_SPI_TransmitReceive_IT()��HAL_SPI_TransmitReceive_DMA()��������SPI����ʱ,
 *          Ӧ��ʵ��HAL_SPI_TxRxCpltCallback()������������¼�, ��Ϊ�����������, HAL_SPI_RxCpltCallback()���ᱻ���á�
 *          ��RxCpltCallback���ý�������,����ֻ����ʽ�Ϸ�װһ��,�����ظ�д��
 * @param h_spi spi handle
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *h_spi)
{
    HAL_SPI_RxCpltCallback(h_spi); // ֱ�ӵ��ý�����ɵĻص�����
}

/**
 * @brief ͨ���������SPI���
 *
 * @param h_spi
 * @return uint8_t
 */
uint8_t SPI_SerialNumSelect(SPI_HandleTypeDef *h_spi)
{
    uint8_t spi_case = 0;
    if (h_spi->Instance == SPI1) {
        spi_case = DEVICE_SPI1;
    } else if (h_spi->Instance == SPI2) {
        spi_case = DEVICE_SPI2;
    } else if (h_spi->Instance == SPI3) {
        spi_case = DEVICE_SPI3;
    } else if (h_spi->Instance == SPI4) {
        spi_case = DEVICE_SPI4;
    } else if (h_spi->Instance == SPI5) {
        spi_case = DEVICE_SPI5;
    } else if (h_spi->Instance == SPI6) {
        spi_case = DEVICE_SPI6;
    }
    return spi_case;
}
