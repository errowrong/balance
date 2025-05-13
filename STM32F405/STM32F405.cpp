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
KALMANFILTER speedKalmanFilter, IMUKalmanFilter, kf;
NUC nuc;

DJIMOTOR DJI_can2_motor[DJI_CAN2_MOTOR_NUM] = {
	DJIMOTOR(M3508,SPD, shooterM, ID1, PID(11.f, 0.f, 15.f)),
	DJIMOTOR(M3508,SPD, shooterM, ID2, PID(11.f, 0.f, 15.f)),
	DJIMOTOR(M2006,ACE, supplyM, ID3, PID(50.f, 0.1f, 0.f),PID(1.f,0.f,0.f)),
	DJIMOTOR(M6020,POS, pantileM, ID4, PID(100.f,0.015f,50.f,0.4f),PID(14.f,0.f,1.f,0.36f),
										PID(180.f,0.f,1.f,120.f),PID(12.f,0.f,0.f,0.f)),
	DJIMOTOR(M6020,POS, pantileM, ID6, PID(90.0f, 0.f, 0.f,0.2f),PID(0.6f,0.f,0.f,0.4),
										PID(10.f,0.f,0.f,0.f),PID(1.f,0.f,0.f,0.f)),
};


LKMOTOR LK_can1_motor[LK_CAN1_MOTOR_NUM] = {
	LKMOTOR(LK9015,chassisM,1,&can1),
	LKMOTOR(LK9015,chassisM,2,&can1)
};


DMMOTOR DM_can1_motor[DM_CAN1_MOTOR_NUM] = {
	DMMOTOR(DM_ID1,jointM,L_F,&can2),
	DMMOTOR(DM_ID2,jointM,L_B,&can2),
	DMMOTOR(DM_ID3,jointM,R_F,&can2),
	DMMOTOR(DM_ID4,jointM,R_B,&can2),
};


LQR lqr(-10.27432, -1.2330, -2.54, -4.14223,    3.10474   , 0.9,
	17.809700, 2.927962, 1.9332, 3.2801, 1.28201474, 0.8706842);
//lqr(-1.109049, -0.015135, -2.5407, -3.7896, 12.6983, 2.5504,//-2.419878, -1.724319, 14.017059, 2.970487,   -6.2829   -0.8595   -0.7246   -1.2665   11.7271    1.7574   //-1.2829, -0.8595, -1.7246, -1.2665, 11.7271, 1.7574
//	1.809700, 0.927962, 0, 0, 0.28201474, 0.11706842);    //lzn0405
	/*lqr(-0.666494, -1.297038, -1.567055, -1.757274, 14.195513, 2.489823,
		0.809700, 0.927962, 0, 0, 1.2, 0.8); */ //lc0408  1.005550, 1.120846, 11.633119, 2.673494
	//lqr(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
//	lqr(-13.1106, -2.3251, -2.5407 ,-3.7896,   11.6983 ,   2.5504 , 
//		31.3871  ,  7.7783 ,   8.3878  , 11.7387 ,  26.4788  ,  3.8016
//	)
//
//
////lqr(42.232003, -0.212470, 3.764055, 1.325626, 13.753451, 5.483431,
//	-0.793516, 1.986789, 4.918880, 6.386397, -8.472855, -3.413230);

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
		{ &DJI_can2_motor[0],&DJI_can2_motor[1],&DJI_can2_motor[2],&DJI_can2_motor[3],&DJI_can2_motor[4]},
		{ &LK_can1_motor[1],&LK_can1_motor[0] },
		{ &DM_can1_motor[0],&DM_can1_motor[1],&DM_can1_motor[2],&DM_can1_motor[3] }
	);
	task.Init();
	
	for (;;)
	{

	}
}






