#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "driver_can.h"

#define K_ECD_TO_ANGLE         0.043945f //< �Ƕ�ת���������̶ȵ�ϵ����360/8192
#define ECD_RANGE_FOR_3508     8191      //< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_3508 16000     //< ���Ƶ�����ΧΪ����16384
#define ECD_RANGE_FOR_6020     8191      //< �������̶�ֵΪ0-8191
#define VOLTAGE_LIMIT_FOR_6020 29000     //< ���Ƶ�ѹ��ΧΪ����30000
#define ECD_RANGE_FOR_2006     8191      //< �������̶�ֵΪ0-8191
#define CURRENT_LIMIT_FOR_2006 9900      //< ���Ƶ�����ΧΪ����10000
/** 
 * @brief  ���������࣬���ڷ��ͺ�����ѡ��
 * @note   GM6020�ķ�������IDΪ 0x205-0x20B ֮��
 * @note   M3508�ķ�������IDΪ 0x201-0x208 ֮��
 */
typedef enum {
    Motor3508 = 0x00U,
    Motor6020 = 0x01U,
    Motor2006 = 0x02U
} motor_type;

/**
 * @brief ������صĳ�ʼ���ݣ���CAN�жϻص�����
 */
typedef struct
{
    int16_t speed_rpm;      //< ÿ������תȦ��
    int16_t torque_current; //< ʵ��ת�ص���
    uint8_t temperature;    //< �¶�
    int16_t raw_ecd;        //< ԭʼ����������
} RawData_t;

/**
 * @brief  �����ĵ����̬���ݣ���������в��������ݣ���CAN�жϻص�����
 */
typedef struct
{
    int32_t ecd;              //< ��ǰ����������ֵ����ֵ
    int32_t last_ecd;         //< ��һʱ�̱���������ֵ����ֵ
	int32_t treated_ecd;
    float angle;              //< �����ı������Ƕ�
	float last_angle;		  //< ��һʱ�����ı�������
	int32_t angle_demarcate;
    int16_t angle_speed;      //< �����ı��������ٶ�
    int32_t round_cnt;        //< �ۼ�ת��Ȧ��
	int16_t axis_round_cnt;   //< �ۼ������ת��Ȧ��
    int32_t total_ecd;        //< �������ۼ�����ֵ
    float total_angle;        //< �ۼ���ת�Ƕ�
	float axis_total_angle;   //< �ۼ��������ת�Ƕ�
    int32_t motor_output;     //< ����������ֵ��ͨ��Ϊ���Ƶ������ѹ
    int16_t filter_speed_rpm; //< ����ƽ���˲������ת��
} TreatedData_t;

/**
 * @brief   ����������ڳ�ʼ��������ȷ��
 */
typedef struct
{
    uint8_t can_number;       //< �����ʹ�õ�CAN�˿ں�
    uint16_t can_id;          //< ���ID
    uint8_t motor_type;       //< �������
    uint16_t ecd_offset;      //< �����ʼ���
    uint16_t ecd_range;       //< �������ֶ�ֵ
    uint16_t reduction_ratio; //< ���ٱȣ����ת��ת����Ȧ��������һȦ��
    int16_t current_limit;    //< ����ܳ��ܵ�������
} MotorParam_t;

/**
 * @brief     ������Ĵ�������������CAN���ͻ�����
 * @param[in] motorData     �����̬���ݽṹ�壬ֻ��Ҫ���ͣ����޸����ݣ�����Ҫ����ָ�롢
 * @param[in] id            �����ʶ��ID
 */
typedef uint8_t (*CAN_FillMotorData)(CAN_Instance_t can, TreatedData_t motorData, uint16_t id); //< ������ͻص�

/**
 * @brief  ɸѡCAN���ݣ������µ����̬���ݵĻص�����
 */
typedef uint8_t (*Motor_DataUpdate)(RawData_t *raw, TreatedData_t *treated, CAN_RxBuffer_t bufferRx); //< ������ͻص�

/**
 * @brief  ��������ݺͲ������Լ�������ͬ���֮����������ĳ�Ա����
 */
typedef struct
{
    int16_t online_cnt; //< ������߼�����
    list_t list;                     //< ����ָ�룬����ָ���Լ��ĳ�ʼ������ͨ��ע�ắ����չ��ѭ������
    RawData_t rawData;               //< �����ʼ��̬���ݣ������и���
    TreatedData_t treatedData;       //< ������������ݣ������и���
    MotorParam_t param;              //< ����������ڳ�ʼ��ʱ����
    Motor_DataUpdate MotorUpdate;    //< ���µ���������ݵĺ���ָ��
    CAN_FillMotorData FillMotorData; //< �Բ�ͬ����ID��CAN���ͻ�����������������ݵĺ���ָ��
} Motor_t;
/**
 * @brief  �_��늙Cģʽ
 */
typedef enum 
{
	MIT,
	POSITIONSPEED,
	SPEED,
	CURRENT
}DMode_t;
	
/**
 * @brief  �_��늙C�Y���w
 */
typedef struct
{
	uint16_t id;
	uint16_t online_cnt;
	int16_t torque_current;
    uint8_t temperature;
	float angle;
	float torque;
	int16_t speed_rpm;
	int32_t motor_output;
	CAN_TxBuffer_t txBufferforInit;
	CAN_TxBuffer_t txBufferforMitMode;
	CAN_TxBuffer_t txBufferforPositionSpeed;
	CAN_TxBuffer_t txBufferforSpeed;
	CAN_RxBuffer_t bufferRx;
	CAN_TxBuffer_t txBufferforCurrent;
	DMode_t mode;
	struct
	{
		float position_min;
		float speed_min;
		float kp_min;
		float kd_min;
		float torque_min;
		float position_max;
		float speed_max;
		float kp_max;
		float kd_max;
		float torque_max;
	}MitMode;
}DMiao_t;

void MotorInit(Motor_t *motor, uint16_t ecdOffset, motor_type type, uint16_t gearRatio, CanNumber canx, uint16_t id);
void MotorRxCallback(CAN_Instance_t *canObject);
uint16_t MotorReturnID(Motor_t motor);
void MotorFillData(Motor_t *motor, int32_t output);
uint16_t MotorCanOutput(CAN_Instance_t can, int16_t IDforTxBuffer);

void DMiaoInit(DMiao_t *damiao, CAN_Instance_t can, uint16_t id, DMode_t mode);

void DMiaoMitControl(DMiao_t *damiao, float position, float speed, float kp, float kd, float torque);
void DMiaoPositionSpeedControl(DMiao_t *damiao, float position, float speed);
void DMiaoSpeedControl(DMiao_t *damiao, float speed);
void DMiaoCurrentControl(DMiao_t *damiao, int16_t current);

void DMiao_CanOutput(CAN_Instance_t can, DMiao_t *damiao);

void DMiao_CanUpdata(DMiao_t *damiao);

#endif
