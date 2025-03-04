#ifndef __CAN_H__
#define __CAN_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "FreeRTOS.h"

class CAN
{
public:

	void Init(CAN_TypeDef* instance);
	void InitFilter();

	uint8_t DJImotor_data[12][8];
	uint8_t DJImotor_temp_data[16];
	uint8_t DMmotor_data[4][8];
	uint8_t DMmotor_temp_data[4][8];
	uint8_t LKmotor_data[2][8];
	uint8_t LKmotor_temp_data[8];

	
	HAL_StatusTypeDef canState;
	CAN_HandleTypeDef hcan;
	BaseType_t pd_Rx = false;

	HAL_StatusTypeDef Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len = 8);

private:

	CanTxMsgTypeDef	TxMessage;//发送结构体
	CanRxMsgTypeDef RxMessage;//接收结构体

};

extern CAN can1, can2;

#endif /* __CAN_H__ */



