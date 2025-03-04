#include "./Inc/device/supercap.h"
#include "./Inc/device/imu.h"
#include "./Inc/user/label.h"


void SUPERCAP::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void SUPERCAP::Decode()
{
	pd_Rx = xQueueReceive(*queueHandler, rxData, NULL);
	if (rxData[0] == 0x4a && rxData[1] == 0xa4)
	{
		Cap.capState = (CAPSTATE)rxData[2];
		int16_t _power, _capVoltage, _batteryVoltage;

		_power = rxData[3] << 8 | rxData[4];
		_capVoltage = rxData[5] << 8 | rxData[6];
		_batteryVoltage = rxData[7] << 8 | rxData[8];

		Cap.power = uint_to_float(_power, 0, 250, 16);//电容输出功率
		Cap.capVoltage = uint_to_float(_capVoltage, 0, 50, 16);
		Cap.batteryVoltage = uint_to_float(_batteryVoltage, 0,50, 16);
	}
}


void SUPERCAP::CAPControl(float powerLimit, CAPSTATE state)
{
	if (powerLimit == 0)
	{
		powerLimit = testpower;
	}
	if (state == DISABLE)
	{
		enableFlag = false;
	}
	else if (state == OUTPUT)
	{
		Cap.power = powerLimit;
	}
	else if (state == CLOSE)
	{
		Cap.power = powerLimit;
	}

	Cap.capState = state;
	USARTTransimt(Cap);
}


void SUPERCAP::USARTTransimt(CAP cap)
{
	uint8_t data[5];
	int16_t _power;

	_power = float_to_uint(powerLimit, 0, 250, 16);

	data[0] = cap.frameHeader[0];
	data[1] = cap.frameHeader[1];
	data[2] = cap.capState;
	data[3] = _power >> 8;
	data[4] = _power;

	m_uart->UARTTransmit(data, 5);
}