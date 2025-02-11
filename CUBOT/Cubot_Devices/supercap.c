#include "Supercap.h"
//#include "referee_task.h"
//#include "chassis_task.h"
Supercap_t heroSupercap;
CAN_TxBuffer_t txBuffer0x6FFforCAN1 = {
    .txHeader = {.Identifier = 0x6ff,
                 .DataLength = FDCAN_DLC_BYTES_8}};

uint8_t SupercapRxCallBack(CAN_RxBuffer_t bufferRx, Supercap_t *cap)
{
    if (bufferRx.rxHeader.Identifier == 0x601) {
        cap->capState.cap_voltage   = ((bufferRx.data[0] << 8) + bufferRx.data[1]) * 0.01f;
        cap->capState.motor_current = ((bufferRx.data[2] << 8) + bufferRx.data[3]) * 0.01f;
        cap->capState.motor_power   = ((bufferRx.data[4] << 8) + bufferRx.data[5]) * 0.01f;
        cap->online_cnt             = 0;
    }
    return 0;
}

void SupercapControl(CAN_Instance_t can, Supercap_t *cap)
{

//    if (rc_Ctrl.is_online == 1) {
//        cap->capState.voltage_flag = 1;

//        if (cap->capState.cap_voltage < 5.0f) {
//            cap->capState.voltage_flag = 0;
//        }
//        if (cap->capState.consume_low_power == 0 && cap->capState.voltage_flag == 1) {
//            txBuffer0x6FFforCAN1.data[0] = 1;
//            cap->capState.Supercap_Flag  = 1;
//        } else {
//            txBuffer0x6FFforCAN1.data[0] = 0;
//            cap->capState.Supercap_Flag  = 0;
//        }
//        if (cap->capState.Supercap_Charge_mode == 0)
//            cap->capState.Supercap_Charge = 0;
//        else if (cap->capState.Supercap_Charge_mode == 1)
//            cap->capState.Supercap_Charge = 1;
//    }
//    txBuffer0x6FFforCAN1.data[1] = referee2022.game_robot_status.chassis_power_limit;
//    txBuffer0x6FFforCAN1.data[2] = referee2022.power_heat_data.chassis_power;
//    txBuffer0x6FFforCAN1.data[3] = 0;
//    txBuffer0x6FFforCAN1.data[4] = 0;
//    txBuffer0x6FFforCAN1.data[5] = 0;
//    txBuffer0x6FFforCAN1.data[6] = 0;
//    txBuffer0x6FFforCAN1.data[7] = 0;

//    CAN_Send(&can, &txBuffer0x6FFforCAN1);

//    if ((heroChassis.power.target_require_power_sum) < 50)
//        cap->capState.consume_low_power_cnt++;
//    else
//        cap->capState.consume_low_power_cnt = 0;
//    if (cap->capState.consume_low_power_cnt >= 100) {
//        cap->capState.consume_low_power     = 1;
//        cap->capState.consume_low_power_cnt = 100;
//    } else
//        cap->capState.consume_low_power = 0;
}
