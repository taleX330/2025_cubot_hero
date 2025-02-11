#include "driver_timer.h"
#include "driver_can.h"
#include "user_lib.h"
#include "ano_vofa.h"
#include "mpu6050.h"
#include "bmi088.h"
#include "rm_motor.h"
#include "Supercap.h"
#include "brain.h"
#include "hiwonder.h"
#include "referee_task.h"
#include "online_ctrl_task.h"
#include "chassis_task.h"
#include "holder_task.h"
#include "remote_ctrl_task.h"
#include "shoot_task.h"

/**
 * @brief  初始化任務，只跑一次
 */
void HardwareConfig(void)
{
	DWT_Init(480);
	
	DR16Init(&rc_Ctrl);
	BasePID_Init_All();
	Ladrc_Init_all();
	
 	INS_Init(&mpu6050.mpu6050_Data); 
//	INS_Init(&bmi088.bmi088_Data);

	UARTx_Init(&huart1, DR16_Callback);
	UARTx_Init(&huart2, Brain_Callback);
	UARTx_Init(&huart3, Referee_Callback);
	UARTx_Init(&huart4, Servo_Callback);
	UARTx_Init(&huart5, vofa_Callback);

	CANx_Init(&hfdcan1, CAN1_rxCallBack);
	CANx_Init(&hfdcan2, CAN2_rxCallBack);
	CAN_Open(&can1);
	CAN_Open(&can2);
	
	MecanumChassisInit(&heroChassis);
	HolderInit(&heroHolder);
	ShootInit(&heroShoot); 

	UART_Receive_DMA(&uart1, &uart1_buffer);
	UART_Receive_DMA(&uart2, &uart2_rxbuffer);
	UART_Receive_DMA(&uart3, &uart3_buffer);
	UART_Receive_DMA(&uart4, &uart4_buffer);

	TIMx_Init(&htim14, TIM14_Callback);
	TIM_Open(&tim14);

	ServoMove(1, 0, 300, &huart4);//舵機復位
	ServoMove(2, 0, 300, &huart4);//舵機復位
}

//< TIM14的触发频率在CubeMX中被配置为1000Hz
/**
 * @brief  定时器溢出中断回调
 * @note   代碼主任務
 */
void TIM14_Callback(void)
{
    tim14.clock_time++;
	
	// 接收上位機的幀率
	if ((tim14.clock_time % 1000) == 0) 
		tim14.vision_fps = 0;
	
	OnlineCtrl_Task();
	RemoteCtrl_Task();
	Chassis_Task();
	Holder_Task();
	Shoot_Task();
	Referee_Task();
					
    if (tim14.clock_time % 4 == 0) 
	{
        // 电机缓冲区发送
        MotorCanOutput(can1, 0x200);
        MotorCanOutput(can2, 0x2ff);
    } 
	else if (tim14.clock_time % 4 == 1) 
	{
        // 电机缓冲区发送
        MotorCanOutput(can1, 0x1ff);
        MotorCanOutput(can2, 0x200);
    } 
	else if (tim14.clock_time % 4 == 2) 
	{
        // 电机缓冲区发送
        MotorCanOutput(can1, 0x2ff);
        MotorCanOutput(can2, 0x1ff);
    } 
	else if (tim14.clock_time % 4 == 3) 
        SupercapControl(can2,&heroSupercap);// 超电指令发送
	

    /*注意匿名上位机和VOFA+不要同时用*/
//
			/*匿名上位机*/
//  ANO_Send_Up_Computer(&huart5,0, 0, 0, 0, 0, 0);
//
//			/*VOFA+*/
//  UsartDmaPrintf(&huart7, "%f,%f,%f,%f,%f,%f,%f\r\n",
//                   INS_attitude->roll,
//                   INS_attitude->pitch,
//                   INS_attitude->yaw,
//                   INS_attitude->q[0],
//                   INS_attitude->q[1],
//                   INS_attitude->q[2],
//                   INS_attitude->q[3]);
//
//	UsartDmaPrintf(&huart7, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",-(heroShoot.booster.top.m3508.rawData.speed_rpm), 
//											-(heroShoot.booster.left.m3508.rawData.speed_rpm),
//											heroShoot.booster.right.m3508.rawData.speed_rpm,
//											heroShoot.loader.m3508.rawData.speed_rpm,
//											heroShoot.booster.top.m3508.rawData.temperature,
//											heroShoot.booster.left.m3508.rawData.temperature,
//											heroShoot.booster.right.m3508.rawData.temperature,
//											heroShoot.booster.top.m3508.treatedData.motor_output,
//											heroShoot.booster.left.m3508.treatedData.motor_output,
//											-(heroShoot.booster.right.m3508.treatedData.motor_output),
//											heroShoot.booster.top.m3508.rawData.torque_current,
//											heroShoot.booster.left.m3508.rawData.torque_current,
//											-(heroShoot.booster.right.m3508.rawData.torque_current)); 





}

