#ifndef __TASKLIST_H__
#define __TASKLIST_H__

#include "FreeRTOS.h"
#include "task.h"

/*
若在某个任务卡死，尝试增加堆栈大小
应尽量减少函数嵌套
*/

//任务优先级
#define START_TASK_PRIO		1
//任务堆栈大小	
#define START_STK_SIZE 		256  

#define CANTX_TASK_PRIO		3
#define CANTX_STK_SIZE		512

#define DECODE_TASK_PRIO		2
#define DECODE_STK_SIZE 		512 

#define CONTROL_TASK_PRIO		2
#define CONTROL_STK_SIZE 		512  

#define MOTOR_TASK_PRIO		2
#define MOTOR_STK_SIZE 		256  

#define LED_TASK_PRIO		2
#define LED_STK_SIZE 		256  

class TASK
{
public:
	void Init();
	TaskHandle_t CanTxTask_Handler;
	TaskHandle_t StartTask_Handler;
	TaskHandle_t DecodeTask_Handler;
	TaskHandle_t ControlTask_Handler;
	TaskHandle_t MotorTask_Handler;
	TaskHandle_t LedTask_Handler;
	int32_t counter{};

};

void start_task(void* pvParameters);
void CanTransimtTask(void* pvParameters);
void DecodeTask(void* pvParameters);
void ControlTask(void* pvParameters);
void MotorUpdateTask(void* pvParameters);
void LedTask(void* pvParameters);

#endif // !__TASKLIST_H__

extern TASK task;
