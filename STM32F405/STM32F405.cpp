#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "./Inc/basic/usart.h"
#include "./Inc/basic/can.h"
#include "./Inc/basic/tim.h"
#include "./Inc/basic/led.h"
#include "./Inc/basic/sysclk.h"
#include "./Inc/basic/delay.h"
#include "./Inc/device/imu.h"
#include "./Inc/device/DJImotor.h"
#include "./Inc/device/DMmotor.h"
#include "./Inc/device/LKmotor.h"
#include "./Inc/device/RC.h"
#include "./Inc/device/judgement.h"
#include "./Inc/user/label.h"
#include "./Inc/user/control.h"
#include "./Inc/user/LQR.h"
#include "./Inc/user/taskslist.h"
#include "./Inc/user/kalmanFilter.h"
#include "./Inc/device/supercap.h"
#include "./Inc/device/nuc.h"

CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
LED led1, led2;
TIM timer;
IMU imuChassis, imuPantile;
DELAY delay;
RC rc;
TASK task;
CONTROL ctrl;
PARAMETER para;
Judgement judgement;
SUPERCAP supercap;
KALMANFILTER speedKalmanFilter, IMUKalmanFilter;
NUC nuc;

DJIMOTOR DJI_can2_motor[DJI_CAN2_MOTOR_NUM] = {
	//,PID(15.f, 2.f, 3.49e-4f)),
	DJIMOTOR(M3508,SPD, shooterM, ID1, PID(11.f, 0.f, 15.f)),
	DJIMOTOR(M3508,SPD, shooterM, ID2, PID(11.f, 0.f, 15.f)),
	//DJIMOTOR(M3508,SPD, shooterM, ID1, PID(0.f, 0.f, 0.f)),
	//DJIMOTOR(M3508,SPD, shooterM, ID2, PID(0.f, 0.f, 0.f)),
	DJIMOTOR(M2006,ACE, supplyM, ID3, PID(50.f, 0.1f, 0.f),PID(1.f,0.f,0.f)),
	//DJIMOTOR(M6020,POS, pantileM, ID5, PID(300.0f, 0.02f, 200.f,0.2f),PID(0.25f,0.f,6.0f)),
	//DJIMOTOR(M6020,POS, pantileM, ID6, PID(70.0f, 0.f, 120.f,0.12f),PID(0.9f,0.f,4.f,0.2)),
	//DJIMOTOR(M6020,POS2, pantileM, ID5, PID(200.0f, 0.008f, 70.f,0.8f),PID(0.49f,0.f,0.3f),PID(100.f,0.015f,50.f,0.4f),PID(14.f,0.f,1.f,0.36f)),
	DJIMOTOR(M6020,POS, pantileM, ID4,PID(100.f,0.015f,50.f,0.4f),PID(14.f,0.f,1.f,0.36f),PID(180.f,0.f,1.f,120.f),PID(12.f,0.f,0.f,0.f)),
	DJIMOTOR(M6020,POS, pantileM, ID6, PID(90.0f, 0.f, 0.f,0.2f),PID(0.6f,0.f,0.f,0.4),PID(10.f,0.f,0.f,0.f),PID(1.f,0.f,0.f,0.f)),

};


LKMOTOR LK_can1_motor[LK_CAN1_MOTOR_NUM] = {
	LKMOTOR(LK9015,chassisM,1,&can1),
	LKMOTOR(LK9015,chassisM,2,&can1)
};


DMMOTOR DM_can1_motor[DM_CAN1_MOTOR_NUM] = {
	DMMOTOR(DM_ID1,jointM,L_F,&can1),
	DMMOTOR(DM_ID2,jointM,L_B,&can1),
	DMMOTOR(DM_ID3,jointM,R_F,&can1),
	DMMOTOR(DM_ID4,jointM,R_B,&can1),
};

//LQR lqr(-18.673 , { -1.5846 }, { -14.8739 }, { -5.217 }, { 40.657 }, { 4.6412 },
//	{ 20.7033 }, { 3.25456 }, { 34.7245 }, { 17.4749 }, { 151.85 }, { 5.278 });//

//LQR lqr(-18.673, { -1.5846 }, { -10.8739 }, { -3.217 }, { 17.657 }, { 1.6412 },
//	{ 148.7033 }, { 6.25456 }, { 34.7245 }, { 17.4749 }, { 151.85 }, { 5.278 });//

//LQR lqr(-10, -1.07, -14.56, -2.5, 17.0, 1.57
//	,
//	15.648, 2.3629, 9.6956, 7.8948, 60.052, 2.91112);//
LQR /*lqr(-11, -1.2, -10.56, -2.5, 14.0, 2.05,*/
	lqr(0, 0, 0, 0, 11.0, 2.75,
	15.0, 2.36, 0, 0, 55, 2.90);//15.648, 2.3629, 9.6956, 7.8948, 60.052, 2.91112

//LQR lqr(-22.673, { -1.9846 }, { -16.8739 }, { -4.217 }, { 32.657 }, { 1.6412 },
//	{ 148.7033 }, { 6.25456 }, { 34.7245 }, { 17.4749 }, { 151.85 }, { 5.278 });//

//LQR lqr(-42.54, -4.2085, -15.5403, -12.683, 18.881, 1.0648,
//	{ 20.7033 }, { 3.25456 }, { 34.7245 }, { 17.4749 }, { 151.85 }, { 5.278 });//

//LQR lqr({ -17.711 }, { -1.8334 }, { -20.6595 }, { -7.1263 }, { 19.351 }, { 3.054 },
//	{ 11.9844 }, { 10.43923 }, { 6.7236 }, { 5.8095 }, { 130.05 }, { 13.5125 });//加入滤波修改参数

int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init();
	can1.Init(CAN1);
	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();
	imuChassis.Init(&uart1, USART1, 115200, CH040, NULL);
	imuPantile.Init(&uart3, USART3, 115200, CH040, ctrl.PantileImuUpdate);
	judgement.Init(&uart4, 115200, UART4);
	supercap.Init(&uart6, USART6,115200);
	nuc.Init(&uart5, UART5, 115200);
	rc.Init(&uart2, USART2, 100000);
	para.Init();
	ctrl.Init(
		{ &DJI_can2_motor[0],&DJI_can2_motor[1],&DJI_can2_motor[2],&DJI_can2_motor[3],&DJI_can2_motor[4]
		/*&DJI_can1_motor[0], &DJI_can1_motor[1], &DJI_can1_motor[2], &DJI_can1_motor[3] */},
		{ &LK_can1_motor[1],&LK_can1_motor[0] },
		/*	{ &HT_can2_motor[0] }*/
		{ &DM_can1_motor[0],&DM_can1_motor[1],&DM_can1_motor[2],&DM_can1_motor[3] }
	);

	//for (size_t i = 0; i < DM_CAN1_MOTOR_NUM; i++)
	//{
	//	DM_can1_motor[i].Motor_Start(DM_can1_motor[i].ID);
	//}
	
	task.Init();
	for (;;)
	{

	}
}






