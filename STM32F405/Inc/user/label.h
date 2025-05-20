#ifndef __TASK_H__
#define __TASK_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include <cstdlib>
#include <cstring>
#include <cmath>

#define g 9.81
#define MAX(x, y) (((x) >= (y)) ? (x) : (y))                                                  // 两个数的最大值
#define MIN(x, y) (((x) >= (y)) ? (y) : (x))                                                  // 两个数的最小值
#define Limit(x, max, min) ((x) > (MAX(min, max)) ? (MAX(min, max)) : ((x) < (MIN(min, max)) ? (MIN(min, max)) : (x)))

#define TIME_STEP 0.005

#define TEST_LKMOTOR 0

#define DJI_CAN1_MOTOR_NUM 0
#define DJI_CAN2_MOTOR_NUM 5
//#define LK_CAN1_MOTOR_NUM 1
//#define LK_CAN2_MOTOR_NUM 1
#define LK_CAN1_MOTOR_NUM 2
#define LK_CAN2_MOTOR_NUM 0
#define HT_CAN1_MOTOR_NUM 0
#define HT_CAN2_MOTOR_NUM 0
#define DM_CAN1_MOTOR_NUM 4
#define DM_CAN2_MOTOR_NUM 0

#define CHASSIS_MOTOR_NUM 2
#define JOINT_MOTOR_NUM 4
#define PANTILE_MOTOR_NUM 2
#define SHOOTER_MOTOR_NUM 2
#define SUPPLY_MOTOR_NUM 1

#define degreeToMechanical(a) ((a)*8192.f/360.f)
#define mechanicalToDegree(a) ((a)*360.f/8192.f)

#define wheelRadii 0.1035
#define PI 3.1415926

#define L1 0.15f
#define L2 0.288f
#define L3 0.288f
#define L4 0.15f
#define L5 0.150f
#define WIDENTH 0.38f




enum PID_MODE { speed = 0, position, autoSpd, autoPos };
enum MOTOR_MODE { SPD = 0, POS,POS2, ACE, TOR };
enum FUCTION_MODE { chassisM, jointM, pantileM, shooterM, supplyM };
enum { last = 0, now };
enum MOTOR_TYPE { M3508, M6020, M2006, M2310, LK9015, HT_03 };
enum { left, right };

class PARAMETER
{
public:

	int32_t pitchMax{}, pitchMin{}, initialMotorPitch{}, initialMotorYaw{};
	float pitchSpd{}, yawSpd{};
	float mg{};
	float initialL0{}, maxL0{}, minL0{}, L0Spd{};
	float supplySpd{}, shootSpd{}, moveSpd{}, turnSpd{}, pcMoveSpd{}, maxMoveSpeed{};
	float M{}, Mw{};
	float FN;

	void Init();

};

//浮点型转无符号整型
static __inline uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;

	return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

//将任意位的无符号数据转为浮点型
static __inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static __inline float Ramp(float setval, float curval, float RampSlope)
{
	if (fabs(setval - curval) > 0.01)
	{
		if ((setval - curval) >= 0)
		{
			curval += RampSlope;
			curval = fmin(curval, setval);
		}
		else
		{
			curval -= RampSlope;
			curval = fmax(curval, setval);
		}
	}
	return curval;
}


static __inline float GetDelta(float valueNow, float valueLast, float threshold)
{
	float delta = valueNow - valueLast;
	if (fabs(delta) > threshold / 2)
	{
		if (delta > 0)
		{
			delta -= threshold;
		}
		else if (delta < 0)
		{
			delta += threshold;
		}
	}
	return delta;
}

extern PARAMETER para;

#endif // !__TASK_H__

