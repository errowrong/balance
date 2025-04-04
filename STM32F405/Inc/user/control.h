#ifndef __CONTORL_H__
#define __CONTROL_H__

#include <vector>
#include <cmath>
#include "stm32f4xx.h"
#include "./Inc/device/DJImotor.h"
#include "./Inc/device/DMmotor.h"
#include "./Inc/device/LKmotor.h"
#include "./Inc/device/imu.h"
#include "./Inc/user/label.h"

//#define TIME_STEP 5
#define ONEBULLETANGLE 36864

class CONTROL final
{
public:
	enum FIREMODE {
		CLOSE,
		ONEB,  // 单发
		THREEB,  // 三发
		FIRE, // 连发
		REVERSE
	};

	bool autoAimFlag = false;

	struct CHASSIS
	{
		LKMOTOR* chassisMotor[CHASSIS_MOTOR_NUM]{};

		bool sideFlag = false;
		float sideYaw{}, followYaw{};

		float moveSpeed{}, turnTorque{},movex{};

		void PowerUpdate();
		void SpeedEstInit();

	};
	struct PANTILE
	{
		DJIMOTOR* pantileMotor[PANTILE_MOTOR_NUM]{};

		PID pantilePid = { 0.45f,0.f,0.4f }, autoAimPid[2] = { {0.1f,0.f,0.f},{0.1f,0.f,0.f} };
		bool roteFlag;//切换至小陀螺标志

		enum TYPE { YAW, PITCH };


		float markPitch{};
		float markImuYaw{}, markImuPitch{}, initialImuYaw{}, intialImuPitch{}, correctYaw{};
		float pitchImuMax{}, pitchImuMin{};

		void Update();
	};
	struct SHOOTER
	{
		DJIMOTOR* shooterMotor[SHOOTER_MOTOR_NUM]{};
		DJIMOTOR* supplyMotor[SUPPLY_MOTOR_NUM]{};

		int32_t primary_speed, middle_speed, mh_speed, high_speed;
		bool openRub{};
		bool isFire{};
		FIREMODE fireMode;


		void Update();
	};

	DMMOTOR* jointMotor[2][2]{};

	void SetMoveSpd(float movex);
	void SetYaw(float deltaYaw);
	void SetL0(float setDeltaL0);
	void SetPantile(float deltaYaw, float deltaPitch);
	void SetShootRub(bool _openRub);
	void SetShootMode(FIREMODE _fireMode);
	void IsFire(bool _isFire);
	void KeepPantile(float imuYaw, float imuPitch);
	void PowerUpdata(float* maxSpeed);

	static void PantileImuUpdate();

	uint8_t GetFireMode();
	bool GetShootRub();

	void Init(std::vector<DJIMOTOR*> DIJmotor, std::vector<LKMOTOR*> LKmotor, std::vector<DMMOTOR*> DMmotor);

	void Update();
	CHASSIS chassis;
private:

	
	PANTILE pantile;
	SHOOTER shooter;

	float omega = 0.98f;
	float aimL0 = 0.13;
	float aimYaw;
	float aimPitch;
};


extern CONTROL ctrl;

#endif // !__CONTORL_H__