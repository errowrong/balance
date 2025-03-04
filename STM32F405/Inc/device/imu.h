#ifndef __IMU_H__
#define __IMU_H__

#include "stm32f4xx.h"
#include "./Inc/basic/usart.h"
#include "./Inc/user/kalman.h"
#include <string.h>
#include <functional>

enum IMU_TYPE { IMU601 = 0, CH040, HI226 };

class IMU
{
public:
	
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type, std::function<void()>fun);
	void Decode();
	void SetCorrect();//设置校准角函数
	bool Check(uint8_t* pdata, uint8_t len, uint32_t com);
	float GetAngleYaw();
	float GetAnglePitch();
	float GetAngleRoll();
	float GetDeltaAngleYaw();
	float* GetAcceleration();
	float GetAngularVelocityPitch();
	float GetAngularVelocityYaw();
	float GetAngularVelocityRoll();

	std::function<void()> UserUpdate;//更新电机imuValue角度;

	int16_t getword(uint8_t HighBit, uint8_t LowBits);

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;
private:
	float ACC_FSR = 4.f, GYRO_FSR = 2000.f;
	typedef struct
	{
		float roll, yaw, pitch;
	}Angle, AngularVelocity;

	typedef struct
	{
		float x{}, y{}, z{};
	}Acceleration;

	Angle angle[2], correctAngle, delta;//correctAngle为yaw校准变量
	AngularVelocity angularvelocity;
	Acceleration acceleration;
	uint16_t crc, len;
	IMU_TYPE type;
	
	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};


static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imuChassis, imuPantile;

#endif // !__IMU_H__

