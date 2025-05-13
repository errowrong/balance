#ifndef __LKMOTOR_H__
#define __LKMOTOR_H__

#include "./Inc/basic/can.h"
#include "./Inc/user/pid.h"
#include "./Inc/user/kalman.h"
#include "./Inc/user/label.h"

#define KT 0.32//转矩常数为0.32

#define C_MIN -1900.f    //IQcontrol的限幅
#define C_MAX 1900.f
#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

#ifndef TEST_LKMOTOR
#define TEST_LKMOTOR 1
#endif // !TEST_LKMOTOR

#if TEST_LKMOTOR
	enum MODE {
		OFF = 0x00, r_pid = 0x30, r_acceleration = 0x33, r_encode = 0x90, r_multi_circle = 0x92,
		r_single_circle = 0x94, state1 = 0x9A, state2 = 0x9C,
		w_pid = 0x31, w_acceleration = 0x34, w_encode = 0x91, torque_control = 0xA1,
		speed_control = 0xA2, angle_control = 0xA6
	};
#endif // TEST_LKMOTOR


enum { P = 0, I };

class LKMOTOR
{
public:

	LKMOTOR(MOTOR_TYPE type, const FUCTION_MODE function, const uint32_t id, CAN* hcan);

	void LKmotorOntimer(uint8_t* odata);
	LKMOTOR& LKmotorDecode(uint8_t idata[][8]);
	void LKmotorTransmit();
	float GetAngularVelocity();

	float torque{}, set_t{};// 20250513lc:这里变量名与函数名重复 故更改

	int16_t Setrange(const int32_t original, const int32_t range);
	float GetTorque();
	float GetSpeed();
	float GetPosition();
	float SetTorque(float settorque);
	/*float T2C(float setTorque);*/
	int16_t T2C(float setTorque);

#if TEST_LKMOTOR	//测试电机
	MODE mode = speed_control;
	uint8_t position_pid[2], speed_pid[2], torque_pid[2];
	uint32_t acceleration{};
	uint16_t encoder{}, encoderRaw{}, encodeOffset{};
	uint8_t spinDirection{};
	uint16_t spin_speed = 10;
	int64_t multi_circle{};
	uint8_t errorState{};
	//00正常，01低压，08过温，09低压过温
	int8_t temperature{};
	uint16_t voltage{};
	void Single_Motor_Tx(CAN can, uint16_t ID, MODE mode, uint8_t* odata);
	void Single_Motor_Decode(CAN can, uint16_t ID, MODE mode);

	void Set_PID(uint8_t speed_P, uint8_t speed_I, uint8_t position_P,
		uint8_t position_I, uint8_t torque_P, uint8_t torque_I, uint8_t* odata);
	void Set_Acceleration(uint32_t acceleration, uint8_t* odata);
	void Set_Offset(uint32_t encodeOffset, uint8_t* odata);
	void Torque_Control(uint16_t torque, uint8_t* odata);
	void Speed_Control(uint32_t speed, uint8_t* odata);
	void Angle_Control(uint8_t spinDirection, uint16_t angle, uint16_t spin_speed, uint8_t* odata);

	void PID_Decode(CAN can, uint16_t ID);
	void Acceleration_Decode(CAN can, uint16_t ID);
	void Encode_Decode(CAN can, uint16_t ID);
	void Multi_Circle_Decode(CAN can, uint16_t ID);
	void Single_Circle_Decode(CAN can, uint16_t ID);
	void State1_Decode(CAN can, uint16_t ID);
	void State2_Decode(CAN can, uint16_t ID);
#endif // TEST_LKMOTOR

private:

	CAN* mcan;

	uint32_t ID;
	MOTOR_MODE motor_mode;
	MOTOR_TYPE type;
	FUCTION_MODE function;
	/*int16_t torque{}, setTorque{};*/
	
	int16_t curSpeed{}, setSpeed{};
	float speed_real_{};
	int32_t angle[2];
	int16_t deltaAngle, setAngle;
	int16_t current{}, setCurrent;
	float torqueCurrent;
	int8_t temperature{};

};

extern LKMOTOR LK_can1_motor[LK_CAN1_MOTOR_NUM];
//extern LKMOTOR LK_can2_motor[LK_CAN2_MOTOR_NUM];

#endif // !__LKMOTOR_H__
