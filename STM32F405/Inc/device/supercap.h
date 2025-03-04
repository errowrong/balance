#pragma once
#include "stm32f4xx_hal.h"
#include "./Inc/basic/usart.h"

class SUPERCAP
{
public:
	enum CAPSTATE { DISABLE, ENABLE, OUTPUT, CLOSE, REVERSE };
	CAPSTATE setstate;
	typedef struct
	{
		uint8_t frameHeader[2] = { 0x4a,0xa4 };
		uint8_t capState;
		// 0 disable
		// 1 enable
		// 2 open
		// 3 close
		float power;
		float capVoltage;
		float batteryVoltage;
	}CAP;

	CAP Cap;

	bool enableFlag;
	float powerLimit = 45.f ,testpower=60.f;
	bool openCAP;

	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	void Decode();
	void CAPControl(float powerLimit, CAPSTATE state);
	void USARTTransimt(CAP cap);

private:

	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};

extern SUPERCAP supercap;
