/**
 **********************************************************************************
 * @file       	rm_motor.c
 * @brief       �豸�㣬������ƴ��롣�豸������������������ݽṹ�������豸����ɹ���������¶�o�û�
 * @details     ͨ������CAN��������մ���ͷ��͵�����������
 * @date        2024-07-10
 * @version     V1.1
 * @copyright   Copyright (c) 2021-2121  �й���ҵ��ѧCUBOTս��
 **********************************************************************************
 * @attention
 * Ӳ��ƽ̨: STM32H750VBT \n
 * SDK�汾��-++++
 * @par �޸���־:
 * <table>
 * <tr><th>Date         <th>Version     <th>Author      <th>Description
 * <tr><td>2021-10-10   <td>1.0         <td>RyanJiao    <td>����շ�������д
 * <tr><td>2024-04-12   <td>1.1         <td>EmberLuo    <td>ɾ���ļ��е������ֵ���˲�����������Ȧ�ǶȻ���
 * <tr><td>2024-06-04   <td>1.1         <td>EmberLuo    <td>����driver_can�ļ���������������ΪCAN_Instance_t����
 * </table>
 *
 **********************************************************************************
 ==============================================================================
                        How to use this module
 ==============================================================================

    ���driver_can.h

    1. ����Motor�ṹ�壬��Ϊ���ʵ����

    2. ����MotorInit()����ʼ�������̬���ݡ�

    3. ������CANʱע���CANx_rxCallBack�ص����ж�ID�����MotorRxCallback()�����յ����̬���ݡ�

    4. ����PID������������������OutputCurrent��

    5. ��������Ӧ������MotorFillData()��д��Ӧ����ID�µĴ���������

    6. ����������д��Ϻ����MotorCanOutput()���Ͷ�ӦCAN�豸���ض�����ID��CAN����

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
#include "rm_motor.h"
#include "user_lib.h"
/**
 * @brief ��Ӧ�󽮵����ͬ����ID��CAN���ݷ��ͻ�����
 */
CAN_TxBuffer_t txBuffer0x200forCAN1 =   
    {
    .txHeader.Identifier = 0x200,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x1FFforCAN1 = 
    {
    .txHeader.Identifier = 0x1FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x2FFforCAN1 =   
    {
    .txHeader.Identifier = 0x2FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x200forCAN2 = 
    {
    .txHeader.Identifier = 0x200,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x1FFforCAN2 =   
    {
    .txHeader.Identifier = 0x1FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
CAN_TxBuffer_t txBuffer0x2FFforCAN2 = 
    {
    .txHeader.Identifier = 0x2FF,
    .txHeader.DataLength = FDCAN_DLC_BYTES_8
    };
	
/**
 * @brief  ���������㺯�����������̶�ת��Ϊ�Ƕ�
 */
static void MotorEcdtoAngle(Motor_t *motor)
{
	(motor->treatedData).last_ecd = (motor->treatedData).ecd;
    if ((motor->param).reduction_ratio == 1) 
    {
        if ((&motor->param)->ecd_offset < ((&motor->param)->ecd_range / 2)) 
        {
            if ((motor->treatedData.treated_ecd) > (&motor->param)->ecd_offset + (&motor->param)->ecd_range / 2)
                motor->treatedData.treated_ecd = (motor->treatedData.treated_ecd) - (&motor->param)->ecd_range;
        } 
        else 
        {
            if ((motor->treatedData.treated_ecd) < (&motor->param)->ecd_offset - (&motor->param)->ecd_range / 2)
                motor->treatedData.treated_ecd = (motor->treatedData.treated_ecd) + (&motor->param)->ecd_range;
        }
        /*�������������ض������λ��ת��Ϊ+-180��ĽǶ�*/
        (&motor->treatedData)->angle = K_ECD_TO_ANGLE * ((motor->treatedData.treated_ecd) - (&motor->param)->ecd_offset);
    } 
	else 
	{
        /*���ٱȲ�Ϊ1��ʱ��Ŀǰ��֧�ּ��ٱ�Ϊ2��3��36�����*/
        motor->treatedData.treated_ecd = motor->rawData.raw_ecd - (&motor->param)->ecd_offset;
        if ((motor->treatedData).treated_ecd < 0) 
            motor->treatedData.treated_ecd += (motor->param).ecd_range;
        if (((motor->treatedData).last_ecd > ((&motor->param)->ecd_range - 4000)) && ((motor->treatedData).treated_ecd < 4000)) 
        {
            (&motor->treatedData)->round_cnt++;
            if ((motor->treatedData.round_cnt) == (motor->param).reduction_ratio)
			{
                (&motor->treatedData)->round_cnt = 0;
				(&motor->treatedData)->axis_round_cnt++;
			}
        } 
		else if (((motor->treatedData).last_ecd >= 0) && ((motor->treatedData).last_ecd < 4000) && ((motor->treatedData).treated_ecd > ((&motor->param)->ecd_range - 4000)))
        {
            (&motor->treatedData)->round_cnt--;
            if ((motor->treatedData.round_cnt) == -1)
                (&motor->treatedData)->round_cnt = (motor->param).reduction_ratio - 1;
        }
		(motor->treatedData).ecd = (motor->treatedData).treated_ecd;
        for (uint16_t i = 0; i < (motor->param).reduction_ratio; i++) 
		{
            if ((motor->treatedData.round_cnt) == i) 
			{
				(&motor->treatedData)->total_ecd = (motor->treatedData).treated_ecd + i * ((motor->param).ecd_range) + i - 1;
                if ((&motor->treatedData)->total_ecd > (((motor->param).reduction_ratio) * ((&motor->param)->ecd_range / 2))) 
				{
					motor->treatedData.treated_ecd   = ((&motor->treatedData)->total_ecd - ((motor->param).reduction_ratio) * ((motor->param).ecd_range));
					(&motor->treatedData)->angle = K_ECD_TO_ANGLE * motor->treatedData.treated_ecd / ((motor->param).reduction_ratio);
				}
				else 
				{
					motor->treatedData.treated_ecd  = (&motor->treatedData)->total_ecd;
					(&motor->treatedData)->angle = K_ECD_TO_ANGLE * motor->treatedData.treated_ecd / ((motor->param).reduction_ratio);
				}
            }
        }
		/*δ���*/
		if((&motor->treatedData)->angle > 170 && (&motor->treatedData)->last_angle < -170 && (&motor->treatedData)->round_cnt == (((motor->param).reduction_ratio+1) / 2))
			(&motor->treatedData)->axis_round_cnt++;
		(&motor->treatedData)->axis_total_angle = (&motor->treatedData)->angle + (&motor->treatedData)->round_cnt * 360.0f / (motor->param).reduction_ratio + (&motor->treatedData)->axis_round_cnt * 360;
    }
}

/**
 * @brief  �������޷���
 */
static void MotorOutputLimit(Motor_t *motor)
{
    if ((&motor->treatedData)->motor_output > motor->param.current_limit)
        (&motor->treatedData)->motor_output = motor->param.current_limit;
    else if ((&motor->treatedData)->motor_output < (-motor->param.current_limit))
        (&motor->treatedData)->motor_output = (-motor->param.current_limit);
}

/**
 * @brief  ���C610��C620����Ŀ���ID������������������CAN���ͻ������ĺ���ָ�롣
 */
static uint8_t CAN_fill_3508_2006_data(CAN_Instance_t can, TreatedData_t motorData, uint16_t id)
{
    if (can.canHandler == &hfdcan1) 
    {
        if (id >= 0x201 && id <= 0x204) 
        {
            txBuffer0x200forCAN1.data[(id - 0x201) * 2]     = motorData.motor_output >> 8;
            txBuffer0x200forCAN1.data[(id - 0x201) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x205 && id <= 0x208) 
        {
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        }
    } 
    else if (can.canHandler == &hfdcan2) 
    {
        if (id >= 0x201 && id <= 0x204) 
        {
            txBuffer0x200forCAN2.data[(id - 0x201) * 2]     = motorData.motor_output >> 8;
            txBuffer0x200forCAN2.data[(id - 0x201) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x205 && id <= 0x208) 
        {
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x209 && id <= 0x20B) 
        {
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2]     = motorData.motor_output >> 8;
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2 + 1] = motorData.motor_output & 0xff;
        }
    }
    return 0;
}

/**
 * @brief  ���GM6020����Ŀ���ID������������������CAN���ͻ������ĺ���ָ�롣
 */
static uint8_t CAN_fill_6020_data(CAN_Instance_t can, TreatedData_t motorData, uint16_t id)
{
    if (can.canHandler == &hfdcan1) 
    {
        if (id >= 0x205 && id <= 0x208) {
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN1.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        } else if (id >= 0x209 && id <= 0x20B) {
            txBuffer0x2FFforCAN1.data[(id - 0x209) * 2]     = motorData.motor_output >> 8;
            txBuffer0x2FFforCAN1.data[(id - 0x209) * 2 + 1] = motorData.motor_output & 0xff;
        }
    } 
    else if (can.canHandler == &hfdcan2) 
    {
        if (id >= 0x205 && id <= 0x208) 
        {
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2]     = motorData.motor_output >> 8;
            txBuffer0x1FFforCAN2.data[(id - 0x205) * 2 + 1] = motorData.motor_output & 0xff;
        } 
        else if (id >= 0x209 && id <= 0x20B) 
        {
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2]     = motorData.motor_output >> 8;
            txBuffer0x2FFforCAN2.data[(id - 0x209) * 2 + 1] = motorData.motor_output & 0xff;
        }
    }
    return 0;
}

/**
 * @brief  ������ݸ��»ص�������ֻ��motor.c�ļ��ڵ��á����󽮵���������ĸ�ʽ��ͬ��
 */
static uint8_t CAN_update_data(RawData_t *raw, TreatedData_t *treated, CAN_RxBuffer_t bufferRx)
{

    raw->raw_ecd         = bufferRx.data[0] << 8 | bufferRx.data[1];
    raw->speed_rpm       = bufferRx.data[2] << 8 | bufferRx.data[3];
    raw->torque_current  = bufferRx.data[4] << 8 | bufferRx.data[5];
    raw->temperature     = bufferRx.data[6];
    treated->treated_ecd = raw->raw_ecd;

    return 0;
}

/**
 * @brief ע�����豸��CAN�豸������
 */
void CAN_RegisterMotor(CAN_Instance_t *canx, Motor_t *motor)
{
    list_add(&motor->list, (&canx->devicesList));
}

/**
 * @brief ������ṹ���CAN�豸����ɾ��
 */
void CAN_DeleteMotor(Motor_t *motor)
{
    list_del(&(motor->list)); //< �ж�������������ɾ�����豸
}

/**
 * @brief �����ʼ�������þ�̬������������������λ��������ͣ����ٱȺ�id
 *
 * @param motor 		��Ҫ��ʼ���ĵ��
 * @param ecdOffset 	��������λ
 * @param type 			�������
 * @param gearRatio 	���ٱȣ�Ŀǰֻ֧��ת���������֮��Ϊ2:1��3:1�����
 * @param canx 			ʹ�õ���CAN1����CAN2
 * @param id 			CAN_ID
 */
void MotorInit(Motor_t *motor, uint16_t ecdOffset, motor_type type, uint16_t gearRatio, CanNumber canx, uint16_t id)
{
    (&motor->param)->ecd_offset      = ecdOffset;
    (&motor->param)->motor_type      = type;
    (&motor->param)->can_id          = id;
    (&motor->param)->reduction_ratio = gearRatio;
    (&motor->param)->can_number      = canx;

    if (canx == CAN1)
        CAN_RegisterMotor(&can1, motor);
    else if (canx == CAN2)
        CAN_RegisterMotor(&can2, motor);

    switch (type) 
    {
        case Motor3508: 
        {
            (&motor->param)->current_limit = CURRENT_LIMIT_FOR_3508;
            (&motor->param)->ecd_range     = ECD_RANGE_FOR_3508;
            motor->MotorUpdate             = CAN_update_data;
            motor->FillMotorData           = CAN_fill_3508_2006_data;
            break;
        }
        case Motor6020: 
        {
            (&motor->param)->current_limit = VOLTAGE_LIMIT_FOR_6020;
            (&motor->param)->ecd_range     = ECD_RANGE_FOR_6020;
            motor->MotorUpdate             = CAN_update_data;
            motor->FillMotorData           = CAN_fill_6020_data;
            break;
        }
        case Motor2006: 
        {
            (&motor->param)->current_limit = CURRENT_LIMIT_FOR_2006;
            (&motor->param)->ecd_range     = ECD_RANGE_FOR_2006;
            motor->MotorUpdate             = CAN_update_data;
            motor->FillMotorData           = CAN_fill_3508_2006_data;
            break;
        }
        default:;
    }
}

/**
 * @brief  ����canID���豸������Ѱ�Ҷ�Ӧ�ĵ��
 */
static Motor_t *MotorFind(uint16_t canid, CAN_Instance_t canx)
{
    Motor_t *motor = NULL;
    list_t *node   = NULL;

    for (node = canx.devicesList.next; node != (canx.devicesList.prev->next); node = node->next) //< ��ѭ���������һȦ
    {
        motor = list_entry(node, Motor_t, list); //< ��������ͷ�����ڽ�㡢��Ƕ������Ľṹ�����͡���Ƕ������Ľṹ������������������ƣ����ɷ���Ƕ��ͷ�����ڽ��Ľṹ��
        if (motor->param.can_id == canid) {
            motor->online_cnt = 0; //< ������ߣ���������
            return motor;
        }
    }
    return NULL;
}

/**
 * @brief  ������ջص�ҵ���߼�, ���µ����̬���ݣ����б������Ƕȱ任
 */
void MotorRxCallback(CAN_Instance_t *canObject)
{
    uint32_t id;
    Motor_t *temp_motor = NULL;

    id         = canObject->rxBuffer.rxHeader.Identifier;
    temp_motor = MotorFind(id, *canObject);
    if (temp_motor != NULL) 
    {
        temp_motor->MotorUpdate(&temp_motor->rawData, &temp_motor->treatedData, canObject->rxBuffer);
        MotorEcdtoAngle(temp_motor);
    }
}

/**
 * @brief ��õ���ṹ���е�ID
 */
uint16_t MotorReturnID(Motor_t motor)
{
    return motor.param.can_id;
}

/**
 * @brief  ��treatedData.motor_output�޷������뷢�ͻ������ȴ����͡�
 */
void MotorFillData(Motor_t *motor, int32_t output)
{
    motor->treatedData.motor_output = output;
    MotorOutputLimit(motor);
    if (motor->param.can_number == CAN1)
        motor->FillMotorData(can1, motor->treatedData, motor->param.can_id);
    else if (motor->param.can_number == CAN2)
        motor->FillMotorData(can2, motor->treatedData, motor->param.can_id);
}

/**
 * @brief  ���ض�ID��CAN_TxBuffer_t���ͳ�ȥ��
 */
uint16_t MotorCanOutput(CAN_Instance_t can, int16_t IDforTxBuffer)
{
    switch (IDforTxBuffer) 
    {
        case 0x200: 
        {
            if (can.canHandler == &hfdcan1)
                CAN_Send(&can, &txBuffer0x200forCAN1);
            else if (can.canHandler == &hfdcan2)
                CAN_Send(&can, &txBuffer0x200forCAN2);
            break;
        }
        case 0x1ff: 
        {
            if (can.canHandler == &hfdcan1)
                CAN_Send(&can, &txBuffer0x1FFforCAN1);
            else if (can.canHandler == &hfdcan2)
                CAN_Send(&can, &txBuffer0x1FFforCAN2);
            break;
        }
        case 0x2ff: 
        {
            if (can.canHandler == &hfdcan1)
                CAN_Send(&can, &txBuffer0x2FFforCAN1);
            else if (can.canHandler == &hfdcan2)
                CAN_Send(&can, &txBuffer0x2FFforCAN2);
            break;
        }
        default:;
    }
	
	
    return 0;
}

/**
 * @brief  �_��늙C��ʼ��
 */
void DMiaoInit(DMiao_t *damiao, CAN_Instance_t can, uint16_t id, DMode_t mode)
{
	damiao->id   = id;
	damiao->mode = mode;
	
	if(damiao->id < 5)
	{
		damiao->txBufferforInit.txHeader.Identifier          = 0x200;
		damiao->txBufferforMitMode.txHeader.Identifier       = 0x200;
		damiao->txBufferforPositionSpeed.txHeader.Identifier = 0x200;
		damiao->txBufferforSpeed.txHeader.Identifier         = 0x200;
		damiao->txBufferforCurrent.txHeader.Identifier       = 0x200;
	}
	else
	{
		damiao->txBufferforInit.txHeader.Identifier          = 0x1FF;
		damiao->txBufferforMitMode.txHeader.Identifier       = 0x1FF;
		damiao->txBufferforPositionSpeed.txHeader.Identifier = 0x1FF;
		damiao->txBufferforSpeed.txHeader.Identifier         = 0x1FF;
		damiao->txBufferforCurrent.txHeader.Identifier       = 0x1FF;
	}
	
	damiao->txBufferforInit.txHeader.DataLength = FDCAN_DLC_BYTES_8;	
	damiao->bufferRx.rxHeader.DataLength        = FDCAN_DLC_BYTES_8;
	
	damiao->txBufferforInit.data[0] = 0xFF;
	damiao->txBufferforInit.data[1] = 0xFF;
	damiao->txBufferforInit.data[2] = 0xFF;
	damiao->txBufferforInit.data[3] = 0xFF;
	damiao->txBufferforInit.data[4] = 0xFF;
	damiao->txBufferforInit.data[5] = 0xFF;
	damiao->txBufferforInit.data[6] = 0xFF;
	damiao->txBufferforInit.data[7] = 0xFC;
	
	CAN_Send(&can, &damiao->txBufferforInit);
}

/**
 * @brief  �_��늙C����ģʽ
 */
void DMiaoMitControl(DMiao_t *damiao, float position, float speed, float kp, float kd, float torque)
{
	static uint16_t i_position, i_speed, i_kp, i_kd, i_torque;
	
	i_position = float_to_uint(position, damiao->MitMode.position_min, damiao->MitMode.position_max, 16);
	i_speed    = float_to_uint(speed, damiao->MitMode.speed_min, damiao->MitMode.speed_max, 12);
	i_kp       = float_to_uint(kp, damiao->MitMode.kp_min, damiao->MitMode.kp_max, 12);
	i_kd       = float_to_uint(kd, damiao->MitMode.kd_min, damiao->MitMode.kd_max, 12);
	i_torque   = float_to_uint(torque, damiao->MitMode.torque_min, damiao->MitMode.torque_max, 12);
	
	damiao->txBufferforMitMode.txHeader.DataLength = FDCAN_DLC_BYTES_8;
	
	damiao->txBufferforMitMode.data[0] = (i_position >> 8);
	damiao->txBufferforMitMode.data[1] = i_position;
	damiao->txBufferforMitMode.data[2] = (i_speed >> 4);
	damiao->txBufferforMitMode.data[3] = ((i_speed&0xF)<<4)|(i_kp>>8);
	damiao->txBufferforMitMode.data[4] = i_kp;
	damiao->txBufferforMitMode.data[5] = (i_kd >> 4);
	damiao->txBufferforMitMode.data[6] = ((i_kd&0xF)<<4)|(i_torque>>8);
	damiao->txBufferforMitMode.data[7] = i_torque;
}

/**
 * @brief  �_��늙Cλ���ٶ�ģʽ
 */
void DMiaoPositionSpeedControl(DMiao_t *damiao, float position, float speed)
{
	static uint8_t *i_position, *i_speed;
	
	i_position=(uint8_t*)&position;
	i_speed=(uint8_t*)&speed;
	
	damiao->txBufferforPositionSpeed.txHeader.DataLength = FDCAN_DLC_BYTES_8;
	
	damiao->txBufferforPositionSpeed.data[0] = *i_position;
	damiao->txBufferforPositionSpeed.data[1] = *(i_position+1);
	damiao->txBufferforPositionSpeed.data[2] = *(i_position+2);
	damiao->txBufferforPositionSpeed.data[3] = *(i_position+3);
	damiao->txBufferforPositionSpeed.data[4] = *i_speed;
	damiao->txBufferforPositionSpeed.data[5] = *(i_speed+1);
	damiao->txBufferforPositionSpeed.data[6] = *(i_speed+2);
	damiao->txBufferforPositionSpeed.data[7] = *(i_speed+3);
}

/**
 * @brief  �_��늙C�ٶ�ģʽ
 */
void DMiaoSpeedControl(DMiao_t *damiao, float speed)
{
	static uint8_t *i_speed;
	
	i_speed=(uint8_t*)&speed;
	
	damiao->txBufferforSpeed.txHeader.DataLength = FDCAN_DLC_BYTES_4;
	
	damiao->txBufferforSpeed.data[0] = *i_speed;
	damiao->txBufferforSpeed.data[1] = *(i_speed+1);
	damiao->txBufferforSpeed.data[2] = *(i_speed+2);
	damiao->txBufferforSpeed.data[3] = *(i_speed+3);
}

/**
 * @brief  �_��늙C���ģʽ
 */
void DMiaoCurrentControl(DMiao_t *damiao, int16_t current)
{
	
	damiao->txBufferforSpeed.txHeader.DataLength = FDCAN_DLC_BYTES_2;
	
	damiao->txBufferforCurrent.data[0] = (current >> 8);
	damiao->txBufferforCurrent.data[1] = current;
}
/**
 * @brief  �_��늙Cݔ������
 */
void DMiao_CanOutput(CAN_Instance_t can, DMiao_t *damiao)
{
	switch(damiao->mode)
	{
		case MIT: 
		{
			CAN_Send(&can, &damiao->txBufferforMitMode);
			break;
		}
		case POSITIONSPEED: 
		{
			CAN_Send(&can, &damiao->txBufferforPositionSpeed);
			break;
		}
		case SPEED: 
		{
			CAN_Send(&can, &damiao->txBufferforSpeed);
			break;
		}
		case CURRENT:
		{
			CAN_Send(&can, &damiao->txBufferforCurrent);
			break;
		}
		default:;
	}
}
/**
 * @brief  �_��늙Cݔ������
 */
void DMiao_CanUpdata(DMiao_t *damiao)
{
	switch(damiao->mode)
	{
		static uint16_t i_position, i_speed, i_torque;
		case MIT:
		case POSITIONSPEED: 
		case SPEED:
		{	
			if(damiao->bufferRx.rxHeader.Identifier == 0x200 + damiao->id)
			{
				i_position = (damiao->bufferRx.data[1]<<8)|damiao->bufferRx.data[2];
				i_speed    = (damiao->bufferRx.data[3]<<4)|(damiao->bufferRx.data[4]>>4);
				i_torque   = ((damiao->bufferRx.data[4]&0xF)<<8)|damiao->bufferRx.data[5];
				damiao->temperature = (float)(damiao->bufferRx.data[6]);
				
				damiao->angle       = uint_to_float(i_position, damiao->MitMode.position_min, damiao->MitMode.position_max, 16); // (-12.5,12.5)
				damiao->speed_rpm   = uint_to_float(i_speed, damiao->MitMode.speed_min, damiao->MitMode.speed_max, 12);// (-45.0,45.0)
				damiao->torque      = uint_to_float(i_torque, damiao->MitMode.torque_min, damiao->MitMode.torque_max, 12); // (-18.0,18.0)
			}
			break;
		}
		case CURRENT:	
		{
			if(damiao->bufferRx.rxHeader.Identifier == 0x200 + damiao->id)
			{
				damiao->angle          = ((damiao->bufferRx.data[0] << 8)|(damiao->bufferRx.data[1]));
				damiao->angle          = damiao->angle / 8192.0f * 360.0f;
				damiao->speed_rpm      = ((damiao->bufferRx.data[2] << 8)|(damiao->bufferRx.data[3]));
				damiao->torque_current = (damiao->bufferRx.data[4] << 8)|(damiao->bufferRx.data[5]);
				damiao->torque_current = damiao->torque_current / (16384.0f/20.0f);
				damiao->temperature    = (float)(damiao->bufferRx.data[6]);
			}
			break;
		}
		default:;
	}
}
