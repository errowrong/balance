#ifndef __DJIMOTOR_H__
#define __DJIMOTOR_H__

#include <cinttypes>
#include <cmath>
#include <cstring>
#include "./Inc/user/pid.h"
#include "./Inc/user/kalman.h"
#include "./Inc/user/label.h"

#define SQRTF(x) ((x)>0?sqrtf(x):-sqrtf(-x))			
#define T 1.e-3f
#define torqueConstant 0.3

enum { ID1 = 0x201, ID2, ID3, ID4, ID5, ID6, ID7, ID8 };

class DJIMOTOR
{

public:
	uint32_t ID;
	PID pid[4];
	FUCTION_MODE function;
	int32_t torqueCurrent{}, setCurrent{}, curSpeed{};
	float setSpeed{}, aceSpeed = 3400;
	int16_t maxSpeed{}, maxCurrent{};
	int32_t setAngle{}, angle[2]{}, deltaAngle{};
	float setTorque{};
	int32_t curTorque{};
	float curCircle{}, setCircle{};
	float setImuValue{}, imuValue{}, imuSpeedValue{};
	MOTOR_MODE mode{};

	bool autoAim{};
	bool output;
	DJIMOTOR(const MOTOR_TYPE type, const MOTOR_MODE mode, const FUCTION_MODE function, const uint32_t id, PID speed, PID position, PID autoSpd, PID autoPos);
	DJIMOTOR(const MOTOR_TYPE type, const MOTOR_MODE mode, const FUCTION_MODE function, const uint32_t id, PID speed, PID position);
	DJIMOTOR(const MOTOR_TYPE type, const MOTOR_MODE mode, const FUCTION_MODE function, const uint32_t id, PID speed);

	void SetCircle(float deltaangle);
	void DJImotor_Ontimer(uint8_t idata[][8], uint8_t* odata);
	void SetTorque(float settorque);
	void SetSpeed(float speed, float limit = 7000);
	void SetAngle(float setangle);
	void SetImuVaule(float setImuValue);
	int32_t GetSpeed();
	int32_t GetDeltaAngle();
	int32_t GetAngle();
	float GetAngularVelocity();

private:

	void getmax(const MOTOR_TYPE type);
	int16_t getword(const uint8_t high, const uint8_t low);
	float setrange(const float original, const float range);
	MOTOR_TYPE type;

};

//此处要根据实际不同can线上的电机数量进行更改
//extern DJIMOTOR DJI_can1_motor[DJI_CAN1_MOTOR_NUM];
extern DJIMOTOR DJI_can2_motor[DJI_CAN2_MOTOR_NUM];

#endif // !__DJIMOTOR_H__
