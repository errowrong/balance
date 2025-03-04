#ifndef __DMMOTOR_H__
#define __DMMOTOR_H__

#include "./Inc/basic/can.h"
#include "./Inc/user/label.h"
#define P_MIN -12.5f    // Radians
#define P_MAX 12.5f    
#define V_MIN -45.0f    // Rad/s
#define V_MAX 45.0f
#define T_MIN -5.1f    //力矩
#define T_MAX 5.1f
#define KP_MIN 0.0f     // N-m/rad
#define KP_MAX 500.0f//以上这些必须同上位机一致

#define KD_MAX 5
#define KD_MIN 0

enum { DM_ID1 = 0x01, DM_ID2, DM_ID3, DM_ID4 };
enum  POSITION { L_F, L_B, R_F, R_B };

enum ERR { overVoltage = 8, underVoltage, overCurrent, mosOverT, motorOverT, missConnect, overLoad };

//关节电机的数据是问答模式，发送一次控制数据才返回一次当前数据
class DMMOTOR
{
public:
	uint32_t ID;
	FUCTION_MODE function;
	POSITION position;
	ERR error;

	DMMOTOR(const uint32_t ID, FUCTION_MODE function, POSITION position, CAN* hcan);
	DMMOTOR& StateDecode(uint8_t odata[][8]);
	void DMmotorOntimer(DMMOTOR pthis, uint8_t* odata);
	void MotorStart(uint32_t id);
	void DMmotorTransmit(uint32_t id);
	void SetSpeed(float setspeed, float kp, float kd);
	void SetPosition(float position, float kp, float kd);
	void SetTorque(float settorque, float kp = 0.f, float kd = 0.f);
	float GetPosition();
	float GetSpeed();
	float GetTorque();
	bool intial = false;
private:

	CAN* mcan;

	float kp{}, kd{};
	float angle[2]{}, setAngle{}, deltaAngle{};//认为是安装电机时，初始姿态的电机与连杆夹角；
	float pos, setPos;//转子位置，也是反馈回来的角度
	float curSpeed, setSpeed;
	float current, setCurrent;
	float torque, setTorque;

	/*float uint_to_float(int x_int, float x_min, float x_max, int bits)
	{
		/// converts unsigned int to float, given range and number of bits ///
		float span = x_max - x_min;
		float offset = x_min;
		return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;

	};

	int float_to_uint(float x, float x_min, float x_max, int bits)
	{
		float span = x_max - x_min;
		float offset = x_min;
		return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
	}*/
};

extern DMMOTOR DM_can1_motor[DM_CAN1_MOTOR_NUM];

#endif // !__DMMOTOR_H__
