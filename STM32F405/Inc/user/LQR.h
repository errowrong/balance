#ifndef __LQR_H__
#define __LQR_H__

#include "./Inc/device/DJImotor.h"
#include "./Inc/device/DMmotor.h"
#include "./Inc/device/LKmotor.h"
#include "./Inc/device/imu.h"
#include "./Inc/user/kalmanFilter.h"
#include "./Inc/user/label.h"
#include "./Inc/user/kalman.h"
#include <cmath>

class LQR
{
public:
	struct K//分别对应常数项,一次项，二次项，三次项
	{
		float K0{};
		float K1{};
		float K2{};

	};
	typedef struct STATE
	{
		float L0{};
		float yaw{};
		float roll{};
		float timestep;
		float last_time;
		float phi{};
		float dphi{};
		float theta[2]{};
		float dtheta{};
		float x{};
		float dx{};
		float gain = 0.005;
		float d2theta;
	};
	typedef struct LEGTORQUE
	{
		float Tp{};
		float thetaTp{};
	};
	typedef struct SENSOR
	{
		float jointSensor[2]{};
		float wheelSensor{};
		float feedbackT[2]{};
	};
	typedef struct DRIVERTORQUE
	{
		float T_drive{};
		float turnT_drive{};
	};
	typedef struct TORQUE
	{
		float F{};
		float rollTorque{};
		LEGTORQUE legTorque{};
		float T1{};
		float T4{};
		DRIVERTORQUE driverTorque{};
		float debugT1, debugT4;
	};
	typedef struct LEGPOSITION
	{
		float phi0{};
		float phi1{};
		float phi2{};
		float phi3{};
		float phi4{};
		float dphi0{};
		float dphi1{};
		float dphi4{};

		float L0;
		float dL0;
		float d2L0;
	};
	typedef struct JACOBI
	{
		float J[2][2];
	};
	typedef struct ACCELEROMETER
	{
		float x{};
		float y{};
		float z{};
	};
	typedef struct ANGLE
	{
		float roll{};
		float yaw{};
		float pitch{};
	};
	typedef struct pid
	{
		float kp{}, ki{}, kd{};
	};

	pid rollPid = { 10.f,0.f,0.5f };
	pid yawPid = { 0.0f,0.0f,0.f };
	float senstive1 = 1.f;
	struct JOINT
	{
		Kalman filter[2] = { {1.f,10000.f},{1.f,10000.f} };
		float leg_kp = 0;
		float leg_kd = 0;
		float leg_m = 0;
		TickType_t tick_present;
		TickType_t tick_last;
		float Time;
	// , rollPid = { 1.f,0.f,0.f };//旋转pid

		PID jumpPid[5] = { { 0.f, 0.f, 0.f},{ 0.f, 0.f, 0.f},{ 0.f, 0.f, 0.f},{ 0.f, 0.f, 0.f},{ 0.f, 0.f, 0.f} };

		float P{}, FN;
		float rollTorque{};

		JACOBI T2F, F2T;
		STATE error, aim, present;
		SENSOR sensorLast, sensorNow;
		TORQUE aimTorque, feedbackTorque;
		LEGPOSITION legposition;

		bool leftOrRight = false;
		bool offGround = false;//当支持力小于某个值时，认为离地s
		bool xFlag = false;
		
		void Unforce();//失力模式
		void UpdateAim(float speed, float setL, float aimYaw, float aimPitch);
		void UpdateState(float roll, float yaw, float pitch, float m_dphi1, float m_dphi4, float m_dx, float mo_dx, float m_dphi);
		void UpdateTorque(bool offFround);
		
		void LegSolution(float phi1, float phi4, float dphi1, float dphi4);//五连杆正运动学解算
		TORQUE ForwardKinetic(float thetaError, float dthetaError, float xError, float dxError, float phiError, float dphiError, bool offGround);
		TORQUE InverseKinetic();
		JACOBI T2FJacobian(float phi0, float phi1, float phi2, float phi3, float phi4);//五连杆正动力学解算
		JACOBI F2TJacobian(float phi0, float phi1, float phi2, float phi3, float phi4);
		bool OffGroundDection();
	};
	enum  LEGSTATE {
		NORMALL = 0, //正常状态
		STRETCH, //伸腿中
		SHRINK, //缩腿中
		KICK,	//踢腿中
		FALL, //自由落体
		LAND, //着陆
	};
	enum { front, behind };
	enum MODE { UNFORCE, RESET, NORMAL, FOLLOW, SIDEWAYS, ROTATE, ENMERGE };

	float bodyVelocity, bodyVelocityHat{};

	static void SpeedKalmanFilter();

	LQR(float K11, float K12, float K13, float K14, float K15, float K16,
		float K21, float K22, float K23, float K24, float K25, float K26);

	void ModeUpdate(DMMOTOR* jointMotor[][2], LKMOTOR* chassisMotor[], IMU* imuChassis, float* aimL0, float moveSpd, float aimYaw, float aimPitch);
	void UpdateSensor(DMMOTOR* jointMotor[][2], LKMOTOR* chassisMotor, IMU* imuChassis);
	void SpeedCalc();//机体速度解算
	void SetMode(MODE setMode);
	MODE GetMode();
	float GetYaw();

	float AccelerationSolution(float roll, float pitch, float yaw);
	void Jump();
	void SHUTDOWN();
	float Torque_Calcute(float theta, float dtheta, float x, float dx, float phi, float dphi, FUCTION_MODE funcition);

	JOINT joint[2];
	bool initialFlag = false;
	bool isOffGround = false;
	bool jump = false;

private:
	float k11, k12, k13, k14, k15, k16, k21, k22, k23, k24, k25, k26;

	float bodyAccelerometer;
	ACCELEROMETER weight, tempAccelerometer;
	Kalman accelerometerFilter = { 1.f,10000.f };
	LEGSTATE legState;
	MODE mode[2];
	ANGLE imu;
	ACCELEROMETER accelerometer;

	PID thetaPid = { 10.f,0.f,20.f };
	bool legFlag = false;
	float kp = 50;
	float kd = 10;
	float m = 0.2;

	
};

extern LQR lqr;
#endif