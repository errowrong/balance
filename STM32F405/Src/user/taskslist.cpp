#include "./Inc/basic/tim.h"
#include "./Inc/basic/delay.h"
#include "./Inc/basic/can.h"
#include "./Inc/basic/led.h"
#include "./Inc/basic/can.h"
#include "./Inc/device/DJImotor.h"
#include "./Inc/device/LKmotor.h"
#include "./Inc/device/DMMOTOR.h"
#include "./Inc/device/imu.h"
#include "./Inc/device/RC.h"
#include "./Inc/user/control.h"
#include "./Inc/user/taskslist.h"
#include "./Inc/user/LQR.h"
#include "./Inc/device/judgement.h"
#include "./Inc/device/supercap.h"
#include "./Inc/device/nuc.h"

void TASK::Init()
{

	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建LED1任务

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&(task.DecodeTask_Handler));

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlDecodeTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&(task.ControlTask_Handler));

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&(task.MotorTask_Handler));

	xTaskCreate((TaskFunction_t)CanTransimtTask,
		(const char*)"CanTransimtTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&(task.CanTxTask_Handler));

	xTaskCreate((TaskFunction_t)LedTask,
		(const char*)"LedTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&(task.LedTask_Handler));

	vTaskDelete(task.StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}

void MotorUpdateTask(void* pvParameters)
{
	while (1)
	{
		//if (task.counter % 5 == 0)
		//{
			//for (auto& motor : DIJ_can1_motor)motor.DIJmotor_Ontimer(can1.DJImotor_data, can1.DJImotor_temp_data);
			//for (auto& motor : DJI_can2_motor)motor.DJImotor_Ontimer(can2.DJImotor_data, can2.DJImotor_temp_data);
			for (auto& motor : LK_can1_motor)motor.LKmotorDecode(can1.LKmotor_data).LKmotorOntimer(can1.LKmotor_temp_data);
			//LK_can2_motor[0].LKmotorDecode(can2.LKmotor_data).LKmotorOntimer(can2.LKmotor_temp_data);
			/*LK_can1_motor->LKmotor_Ontimer(can1.LKmotor_temp_data);*/
			for (auto& motor : DM_can1_motor)motor.StateDecode(can2.DMmotor_data).DMmotorOntimer(motor, can2.DMmotor_temp_data[motor.ID - 0x01]);
			//for (auto& motor : LK_can1_motor)motor.LKmotor_Decode(can1.LKmotor_data);
			//LK_can1_motor[0].Single_Motor_Decode(can1, LK_can1_motor[0].ID, LK_can1_motor[0].mode);
			
		//}
		vTaskDelay(1);
	}
}

void CanTransimtTask(void* pvParameters)
{
	while (true)
	{
		
		if (task.counter > 1000) 
		{
			//can2
			DM_can1_motor[0].DMmotorTransmit(DM_can1_motor[0].ID);
			DM_can1_motor[1].DMmotorTransmit(DM_can1_motor[1].ID);
			DM_can1_motor[2].DMmotorTransmit(DM_can1_motor[2].ID);
			DM_can1_motor[3].DMmotorTransmit(DM_can1_motor[3].ID);
			//can1
			LK_can1_motor[0].LKmotorTransmit();
		}

		/*if (task.counter > 1000)
		{
			switch (task.counter % 5)
			{
			case 0:		
				DM_can1_motor[0].DMmotorTransmit(DM_can1_motor[0].ID);
				break;
			case 1:
				DM_can1_motor[1].DMmotorTransmit(DM_can1_motor[1].ID);
				break;
			case 2:
				DM_can1_motor[2].DMmotorTransmit(DM_can1_motor[2].ID);
				break;
			case 3:
				DM_can1_motor[3].DMmotorTransmit(DM_can1_motor[3].ID);
				break;
			case 4:
				LK_can1_motor[0].LKmotorTransmit();
				break;
			default:
				break;
			}
		}*/
		task.counter++;
		vTaskDelay(1);
	}
}

void DecodeTask(void* pvParameters)
{
	while (true)
	{
		if (task.counter % 5 == 0)
		{
			/*judgement.BuffData();
			judgement.GetData();*/
			//imuChassis.Decode();
			//imuPantile.Decode();
			rc.Decode();
			//nuc.Decode();
		}
		imuChassis.Decode();
		
		vTaskDelay(1);
	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		if (task.counter % 5 == 0)
		{
			//nuc.Encode(imuPantile.GetAnglePitch(), imuPantile.GetAngleYaw(), imuPantile.GetAngleRoll());
			rc.Update();
			ctrl.Update();
		};
		ctrl.Update();

		vTaskDelay(1);
	}
}

void LedTask(void* pvParameters)
{
	while (true)
	{
		
		vTaskDelay(1);
	}
}



