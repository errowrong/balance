#include "./Inc/basic/tim.h"
#include "./Inc/device/RC.h"
#include "./Inc/device/imu.h"
#include "./Inc/device/DJImotor.h"
#include "./Inc/user/LQR.h"
#include "./Inc/user/control.h"
#include "./Inc/user/QuaternionEKF.h"
#include "./Inc/device/LKmotor.h"


LQR::LQR(float K11, float K12, float K13, float K14, float K15, float K16,
	float K21, float K22, float K23, float K24, float K25, float K26) :
	k11(K11), k12(K12), k13(K13), k14(K14), k15(K15), k16(K16),
	k21(K21), k22(K22), k23(K23), k24(K24), k25(K25), k26(K26) 
{
	joint[left].aim.L0 = para.initialL0;
	joint[right].aim.L0 = para.initialL0;

	std::vector<float> P{ 100000, 0,
					0,100000 };
	std::vector<float> F{ 1.f, TIME_STEP,
				 0, 1.f, };
	std::vector<float> Q { 25,0,
							0,0.01 };
	std::vector<float> R{ 800, TIME_STEP * TIME_STEP,
		TIME_STEP * TIME_STEP ,0.01};
	std::vector<float> H{ 1, 0,
						0,1	};
	std::vector<float> Z{ 0.1, 0,
							0, 0.1 };
	std::vector<float> measurement_reference{ 0.3,0.3 };
	std::vector<float> state_min_variance { 0.1,0.1 };
	std::vector<float> measurement_degree { 1,1 };
	std::vector<float> mat_R_diagonal_elements{ 0.2,0.2};
	speedKalmanFilter.TaskInit( P, F, Q,R,H,Z,
		measurement_reference, state_min_variance, measurement_degree, mat_R_diagonal_elements, 
		2, 0, 2, 0, SpeedKalmanFilter);
};

void LQR::SpeedKalmanFilter()
{
	static float Q[4]= { 1000,0,0,1 },R[4]= { 5000, TIME_STEP * TIME_STEP,
		TIME_STEP * TIME_STEP,1 };
	memcpy(speedKalmanFilter.kf.Q.pData, Q, 16);
	memcpy(speedKalmanFilter.kf.R.pData, R, 16);
	float acY, weightY, temp;

	acY = imuChassis.GetAcceleration()[1];
	weightY = cosf(imuChassis.GetAngleRoll() * PI / 180.f) * sinf(imuChassis.GetAnglePitch() * PI / 180.f);

	speedKalmanFilter.kf.MeasuredVector[0] = lqr.bodyVelocity;
	speedKalmanFilter.kf.MeasuredVector[1] = (acY - weightY) * g;
	temp = (acY - weightY) * g;

	// 提取估计值
	lqr.bodyVelocityHat = speedKalmanFilter.kf.FilteredValue[0];
}

void LQR::ModeUpdate(DMMOTOR* jointMotor[][2], LKMOTOR* chassisMotor[], IMU* _imuChassis, float* aimL0, float moveSpd, float aimYaw, float aimPitch)
{
	
	UpdateSensor(jointMotor, *chassisMotor, _imuChassis);
	AccelerationSolution(imu.roll, imu.pitch, imu.yaw);  //离地检测

	float dphi1[2], dphi4[2];

	dphi1[left] = jointMotor[left][behind]->GetSpeed();
	dphi1[right] = -jointMotor[right][behind]->GetSpeed();
	dphi4[left] = jointMotor[left][front]->GetSpeed();
	dphi4[right] = -jointMotor[right][front]->GetSpeed();

	float dx[2];
	dx[left] = (-1.f) * chassisMotor[left]->GetAngularVelocity() * wheelRadii;
	dx[right] = chassisMotor[right]->GetAngularVelocity() * wheelRadii;

	float dphi = imuChassis.GetAngularVelocityPitch() * PI / 180.f;
	
	SpeedCalc(); //机体速度解算
	SpeedKalmanFilter();
	//bodyVelocity = AccelerationSolution(imu.roll, imu.pitch, imu.yaw);
	
	//if (initialFlag)
	//{
	//	//initialFlag = true;
	//speedKalmanFilter.TaskUpdate();
	
	joint[left].UpdateState(imu.roll, imu.yaw, imu.pitch, dphi1[left], dphi4[left], bodyVelocityHat, dx[left], dphi);//速度融合
	joint[right].UpdateState(imu.roll, imu.yaw, imu.pitch, dphi1[right], dphi4[right], bodyVelocityHat, dx[right], dphi);
	//}
	//else
	//{
		//joint[left].UpdateState(imu.roll, imu.yaw, imu.pitch, dphi1[left], dphi4[right], dx[left], dx[right], dphi);//非速度融合
		//joint[right].UpdateState(imu.roll, imu.yaw, imu.pitch, dphi1[right], dphi4[right], dx[right], dx[left], dphi);
	//}

	float deltaTheta = joint[right].present.theta[now] - joint[left].present.theta[now];
	float thetaTp = thetaPid.Position(deltaTheta);

	joint[left].aimTorque.legTorque.thetaTp = -thetaTp;
	joint[right].aimTorque.legTorque.thetaTp = thetaTp;    //theta补偿

	joint[left].UpdateAim(moveSpd, *aimL0, aimYaw, aimPitch);
	joint[right].UpdateAim(moveSpd, *aimL0, aimYaw, aimPitch);

	if (!legFlag && mode[now] != MODE::ENMERGE)
	{
		if (fabs(joint[left].present.phi) < 0.05 && fabs(joint[right].present.phi) < 0.05
			&& fabs(joint[left].present.dphi) < 0.05 && fabs(joint[right].present.dphi) < 0.05)
			//当前行速度不为0且双腿倾角均小于阈值时解锁
		{
			legFlag = true;
		}
	}
	if (mode[now] == UNFORCE)
	{

		joint[left].Unforce();
		joint[right].Unforce();
		legFlag = false;
		*aimL0 = para.initialL0;

		chassisMotor[left]->SetTorque(0);
		chassisMotor[right]->SetTorque(0);
		jointMotor[left][front]->SetTorque(0.f);
		jointMotor[left][behind]->SetTorque(0.f);
		jointMotor[right][front]->SetTorque(0.f);
		jointMotor[right][behind]->SetTorque(0.f);
	}
	if (mode[last] == UNFORCE && mode[now] == RESET)
	{
		jointMotor[left][front]->intial = false;
		jointMotor[left][behind]->intial = false;
		jointMotor[right][front]->intial = false;
		jointMotor[right][behind]->intial = false;
	}
	if (!legFlag)
	{
		jointMotor[left][front]->SetTorque(0.f);
		jointMotor[left][behind]->SetTorque(0.f);
		jointMotor[right][front]->SetTorque(0.f);
		jointMotor[right][behind]->SetTorque(0.f);//4 -22 1 22 2 23 3 23
	}
	else
	{
		
		jointMotor[left][front]->SetTorque(0.05*joint[left].aimTorque.T4);
		jointMotor[left][behind]->SetTorque(0.05*joint[left].aimTorque.T1);
		jointMotor[right][front]->SetTorque(0.05*joint[right].aimTorque.T4);
		jointMotor[right][behind]->SetTorque(0.05*joint[right].aimTorque.T1);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   	}


	/*LK_can1_motor[0].set_t = (1.f)* chassisMotor[left]->SetTorque(joint[left].aimTorque.driverTorque.T_drive);
	LK_can1_motor[1].set_t = (-1.f) * chassisMotor[right]->SetTorque(joint[right].aimTorque.driverTorque.T_drive);*/

	if (legFlag)
	{
		initialFlag = true;
		isOffGround = true;
	}
	mode[last] = mode[now];
}

void LQR::SpeedCalc()
{
	static float dxM[2]{};
	dxM[left] = (-1.f) * ctrl.chassis.chassisMotor[left]->GetAngularVelocity();
	dxM[right] = ctrl.chassis.chassisMotor[right]->GetAngularVelocity();

	static float angularVelocity[2]{}, wheelVelocity[2]{};

	angularVelocity[left] = dxM[left] + joint[left].legposition.dphi0 - imuChassis.GetAngularVelocityPitch() * PI / 180.f;  //imuChassis.GetAngularVelocityPitch() * PI / 180.f这一部分需要改一下。
	angularVelocity[right] = dxM[right] + joint[right].legposition.dphi0 - imuChassis.GetAngularVelocityPitch() * PI / 180.f;
	//统一从机体左方向看，phi0和pitch方向相反

	wheelVelocity[left] = angularVelocity[left] * wheelRadii +
		joint[left].legposition.L0 * joint[left].present.dtheta * arm_cos_f32(joint[left].present.theta[now]) +
		joint[left].legposition.dL0 * arm_sin_f32(joint[left].present.theta[now]);

	wheelVelocity[right] = angularVelocity[right] * wheelRadii +
		joint[right].legposition.L0 * joint[right].present.dtheta * arm_cos_f32(joint[right].present.theta[now]) +
		joint[right].legposition.dL0 * arm_sin_f32(joint[right].present.theta[right]);

	bodyVelocity = (wheelVelocity[left] + wheelVelocity[right]) / 2;
}

void LQR::JOINT::Unforce()
{
	aim.L0 = para.initialL0;
	aimTorque.driverTorque.T_drive = 0;
	aimTorque.driverTorque.turnT_drive = 0;
	aimTorque.legTorque.Tp = 0;
	aimTorque.legTorque.thetaTp = 0;
	aimTorque.T1 = 0;
	aimTorque.T4 = 0;
	aimTorque.F = 0;
	lqr.joint[left].aim.x = 0;
	lqr.joint[left].present.x = 0;
	lqr.joint[right].aim.x = 0;
	lqr.joint[right].present.x = 0;
	lqr.legFlag = false;
	lqr.isOffGround = false;
	offGround = false;

}

void LQR::JOINT::UpdateState(float roll, float yaw, float pitch, float m_dphi1, float m_dphi4, float m_dx, float mo_dx, float m_dphi)
{
	present.roll = roll;
	present.yaw = yaw;

	present.phi = pitch;

	double dL0Last = legposition.dL0;
	double dthetaLast = present.dtheta;
	
	tick_present = xTaskGetTickCount();
	Time = (tick_present - tick_last) * portTICK_PERIOD_MS /1000.f;
	

	present.theta[last] = present.theta[now];

	/*根据实物更新*/
	/*注意前腿是phi4，后腿是phi1，从phi4->phi1方向为正方向*/
	/*................................................*/

	legposition.phi1 = sensorNow.jointSensor[behind];// 与模型对应
	legposition.phi4 = sensorNow.jointSensor[front];

	legposition.dphi1 = m_dphi1;
	legposition.dphi4 = m_dphi4;
	present.dphi = m_dphi;

	LegSolution(legposition.phi1, legposition.phi4, legposition.dphi1, legposition.dphi4);
	//legposition.d2L0 = (legposition.dL0 - dL0Last) / (0.001f+0.008f)+ legposition.d2L0*0.008f/ (0.001f+0.008f);
	const float alpha = 0.001f;  // 原小常数
	const float beta = 0.008f;    // 原系数
	const float denominator = alpha + beta;

	// 使用更大的分母值（如0.1f），保持比例关系
	const float new_denominator = 0.1f;
	legposition.d2L0 = (roundf(legposition.dL0 * 100.0f) / 100.0f - roundf(dL0Last * 100.0f) / 100.0f) / new_denominator
		+ legposition.d2L0 * (beta / denominator) * (denominator / new_denominator);
	 
	/*................................................*/
	present.dx = (m_dx+mo_dx)/2;

	//present.x += (m_dx + mo_dx) * (TIME_STEP) / 2;//差速积分算速度
	if (lqr.mode[now] != SIDEWAYS || fabs(aim.yaw - present.yaw) < 0.04)
	{
		//present.timestep = (timer.counter - present.last_time) / 1000.0f;
		present.x += present.dx * Time;//速度融合计算
	}
	T2FJacobian(legposition.phi0, legposition.phi1, legposition.phi2, legposition.phi3, legposition.phi4);

	present.theta[now] = -(PI / 2 - legposition.phi0 + present.phi); //
	present.dtheta = legposition.dphi0 - m_dphi;

	present.d2theta = (present.dtheta - dthetaLast) / (Time);
	
	tick_last = tick_present;

}


void LQR::JOINT::UpdateAim(float speed, float setL, float aimYaw, float aimPitch)
{
	aim.yaw = aimYaw;

	error.yaw = aim.yaw - present.yaw;
	if (fabs(error.yaw) > PI)
	{
		if (error.yaw > PI)
		{
			error.yaw -= 2 * PI;
		}
		else if (error.yaw <= -PI)
		{
			error.yaw += 2 * PI;
		}
	}

	error.yaw = Limit(error.yaw, 0.8, -0.8);  ///

	error.roll = aim.roll - present.roll;
	if (fabs(error.roll) < 0.01)
	{
		error.roll = 0;
	}

	float gain;
	gain = 0.25 * error.roll;// -lqr.rollPid.kd * imuChassis.GetAngularVelocityRoll() * PI / 180.f;
	/*if (!leftOrRight)
	{*/
		aimTorque.driverTorque.turnT_drive = lqr.yawPid.kp * error.yaw -
			lqr.yawPid.kd * imuChassis.GetAngularVelocityYaw() * PI / 180.f;
		/*aimTorque.rollTorque = -lqr.rollPid.kp * error.roll +
			lqr.rollPid.kd * imuChassis.GetAngularVelocityRoll() * PI / 180.f;*/

		
		//aim.L0 = Ramp(setL + gain, aim.L0, temp);
		aim.L0 = setL;// + 0.25 * error.roll;
	
		/*}
	else
	{
		aimTorque.driverTorque.turnT_drive = yawPid.kp * error.yaw -
			yawPid.kd * imuChassis.GetAngularVelocityYaw() * PI / 180.f;
		aimTorque.rollTorque = lqr.rollPid.kp * error.roll -
			lqr.rollPid.kd * imuChassis.GetAngularVelocityRoll() * PI / 180.f;

		aim.L0 = setL - 0.25 * error.roll;

	}*/

	aimTorque.driverTorque.turnT_drive = Limit(aimTorque.driverTorque.turnT_drive, 5.f, -5.f);

	//aim.L0 = setL;
	aim.dx = Ramp(speed, aim.dx, 0.021f);//速度期望斜坡跟踪
	//aim.x = Ramp(speed, aim.dx, 0.021f);
	error.dx = aim.dx - present.dx;
	
	//if (fabs(aim.dx) < 0.02f && xFlag)//单片机计算精度问题
	//{
	//	aim.dx = 0;
	//	xFlag = false;
	//}
	//else if (aim.dx != 0)
	//{
	//	aim.x = present.x;
	//	xFlag = true;
	//}
	aim.x += (speed*aim.gain);
	//aim.L0 = setL;
	error.x = aim.x - present.x;
	aim.L0 = Limit(aim.L0, para.maxL0, para.minL0);

	aim.phi = aimPitch;


	float* tempAim, * tempPresent, * tempError;
	tempAim = &aim.L0;
	tempPresent = &present.L0;
	tempError = &error.L0;
	for (int i = 0; i < sizeof(STATE) / 4; i++)
	{
		tempError[i] = tempAim[i] - tempPresent[i];
	}

	if (lqr.mode[now] == MODE::ENMERGE)
	{
		error.theta[now] = 0;
		error.dtheta = 0;
		error.phi = 0;
		error.dphi = 0;
		error.x = 0;
		error.dx = 0;
		//error.dx = Limit(error.dx, 0.01, -0.01);
		lqr.legFlag = false;
	}
	error.dx = Limit(error.dx, 2, -2);
	//error.x = 0;
	error.x = Limit(error.x, 1, -1);
	error.dphi = Limit(error.dphi, 1.5f, -1.5f);
	error.dtheta = Limit(error.dtheta, 1.5f, -1.5f);
	InverseKinetic();
	//if (lqr.mode[now] != MODE::SIDEWAYS && lqr.mode[now] != MODE::ENMERGE)
	//{
		//if (lqr.isOffGround)
		//{
	//		offGround = OffGroundDection();
	//	}
	//}
	//lqr.Jump();
	//OffGroundDection();
	ForwardKinetic(error.theta[now], error.dtheta, error.x, error.dx, error.phi, error.dphi, offGround);
	//ForwardKinetic(0, 0, error.x,0,0, 0, ctrl.offGround);
}

LQR::TORQUE LQR::JOINT::ForwardKinetic(float thetaError, float dthetaError, float xError, float dxError, float phiError, float dphiError, bool offFround)
{
	leg_kp = lqr.kp;
	leg_kd = lqr.kd;
	leg_m = lqr.m;

	//if (offFround)
	//{
	//	present.x = aim.x;//离地时，默认不累加位移量
	//	aimTorque.legTorque.Tp = -(lqr.senstive1 * lqr.Torque_Calcute(0, 0, 0, 0, \
	//		0, 0, FUCTION_MODE::jointM));

	//	aimTorque.F = 0;
	//	aimTorque.driverTorque.T_drive = 0;
	//	aimTorque.driverTorque.turnT_drive = 0;
	//}
	//else
	//{
	if (!leftOrRight)
	{//left
		aimTorque.legTorque.Tp = (lqr.Torque_Calcute(thetaError, dthetaError, xError, dxError, \
			phiError, dphiError, FUCTION_MODE::jointM));
		aimTorque.F = -(-leg_kp * error.L0 + leg_kd * legposition.dL0 - leg_m * g * arm_cos_f32(present.theta[0]));
		aimTorque.driverTorque.T_drive = lqr.Torque_Calcute(thetaError, dthetaError, xError, dxError, \
			phiError, dphiError, FUCTION_MODE::chassisM);

		//aimTorque.F += aimTorque.rollTorque;//横滚角补偿
		//aimTorque.legTorque.Tp += aimTorque.legTorque.thetaTp;//劈叉补偿
		// 
		aimTorque.driverTorque.T_drive -= aimTorque.driverTorque.turnT_drive;//转向补偿

		aimTorque.T1 = (T2F.J[0][0] * aimTorque.legTorque.Tp + T2F.J[0][1] * aimTorque.F);
		aimTorque.T4 = (T2F.J[1][0] * aimTorque.legTorque.Tp + T2F.J[1][1] * aimTorque.F);
		aimTorque.T1 = Limit(aimTorque.T1, T_MAX, T_MIN);
		aimTorque.T4 = Limit(aimTorque.T4, T_MAX, T_MIN);
		aimTorque.debugT1 = aimTorque.T1;
		aimTorque.debugT4 = aimTorque.T4;
	}
	else
	{//right
		aimTorque.legTorque.Tp = -(lqr.Torque_Calcute(thetaError, dthetaError, xError, dxError, \
			phiError, dphiError, FUCTION_MODE::jointM));
		aimTorque.F = (-leg_kp * error.L0 + leg_kd * legposition.dL0 - leg_m * g); //* arm_cos_f32(present.theta[0]));
		aimTorque.driverTorque.T_drive = lqr.Torque_Calcute(thetaError,  dthetaError, xError, dxError, \
			phiError, dphiError, FUCTION_MODE::chassisM);

		//aimTorque.F += aimTorque.rollTorque;//横滚角补偿
		//aimTorque.legTorque.Tp += aimTorque.legTorque.thetaTp;//劈叉补偿
		aimTorque.driverTorque.T_drive += aimTorque.driverTorque.turnT_drive;//转向补偿

		aimTorque.T1 = (T2F.J[0][0] * aimTorque.legTorque.Tp + T2F.J[0][1] * aimTorque.F);
		aimTorque.T4 = (T2F.J[1][0] * aimTorque.legTorque.Tp + T2F.J[1][1] * aimTorque.F);
		aimTorque.T1 = Limit(aimTorque.T1, T_MAX, T_MIN);
		aimTorque.T4 = Limit(aimTorque.T4, T_MAX, T_MIN);
		aimTorque.debugT1 = aimTorque.T1;
		aimTorque.debugT4 = aimTorque.T4;
	}
		

		

	//}	

	/*aimTorque.F = -m;*/
	//已知：F<0,轮腿向下支撑；F>0,轮腿向上缩；对于L0,由于error=aim-present
	//present<aim，需要伸腿，error>0,则F<0， (-kp*error.L0)<0,(kd * legposition.dL0)>0阻尼作用
	//present>aim,需要缩腿，error<0,则F>0， (-kp*error.L0)>0,(kd * legposition.dL0)<0阻尼作用

	//aimTorque.F = Limit(aimTorque.F, 50, -50);

	

	return aimTorque;
}

LQR::TORQUE LQR::JOINT::InverseKinetic()
{
	F2TJacobian(legposition.phi0, legposition.phi1, legposition.phi2, legposition.phi3, legposition.phi4);

	feedbackTorque.T1 = sensorNow.feedbackT[behind];
	feedbackTorque.T4 = sensorNow.feedbackT[front];

	feedbackTorque.legTorque.Tp = (F2T.J[0][0] * feedbackTorque.T1 + F2T.J[0][1] * feedbackTorque.T4);
	feedbackTorque.F = (F2T.J[1][0] * feedbackTorque.T1 + F2T.J[1][1] * feedbackTorque.T4);

	return feedbackTorque;
}

void LQR::JOINT::LegSolution(float phi1, float phi4, float dphi1, float dphi4)
{
	float s_phi1 = arm_sin_f32(phi1), c_phi1 = arm_cos_f32(phi1);
	float s_phi4 = arm_sin_f32(phi4), c_phi4 = arm_cos_f32(phi4);

	float a1 = L1 * s_phi1 - L4 * s_phi4;
	float omega1 = a1 * a1;
	float omega2 = L5 - L1 * c_phi1 + L4 * c_phi4;
	float a2 = omega1 + omega2 * omega2;
	float a3 = L2 * omega2;
	float a4 = L2 * L2 - L3 * L3;
	float a5 = L2 * L2 * omega1;
	float a6 = sqrt(4 * a5 - pow(a2 + a4, 2) + 4 * pow(a3, 2));
	float x1 = 2 * L2 * a1 - a6;
	float x2 = a2 + 2 * a3 + a4;
	legposition.phi2 = -2 * atan2(2 * L2 * a1 - a6, a2 + 2 * a3 + a4);

	float s_phi2 = sinf(legposition.phi2), c_phi2 = cosf(legposition.phi2);


	float b1 = a1 + L2 * s_phi2;
	float b2 = -omega2 + L2 * c_phi2;
	legposition.phi3 = atan2(b1, b2);

	float s_phi3 = sinf(legposition.phi3), c_phi3 = cosf(legposition.phi3);

	float c1 = L1 * s_phi1 + L2 * s_phi2;
	float c2 = L1 * c_phi1 - L5 / 2 + L2 * c_phi2;
	legposition.phi0 = atan2(c1, c2);

	float d1 = dphi1 * L1 * c_phi1 - dphi4 * L4 * c_phi4;
	float d2 = dphi1 * L1 * s_phi1 - dphi4 * L4 * s_phi4;
	float d3 = dphi1 * L1 * s_phi1;
	float d4 = dphi1 * L1 * c_phi1;
	float d5 = sinf(legposition.phi2 - legposition.phi3);
	float delta1 = s_phi3 * d1 - c_phi3 * d2;
	float delta2 = L1 * c_phi1 - L5 / 2 + L2 * c_phi2;
	float delta3 = L1 * s_phi1 + L2 * s_phi2;
	float e1 = delta2 * delta2;
	float e2 = delta3 * delta3;

	present.L0 = sqrt(e1 + e2);

	float alpha1 = sinf(legposition.phi3 - phi4);
	float alpha2 = sinf(legposition.phi2 - legposition.phi3);
	float alpha3 = sinf(phi1 - legposition.phi2);

	legposition.dphi0 = -(L1 * dphi1 * cosf(legposition.phi0 - legposition.phi3) * alpha3 +
		L4 * dphi4 * cosf(legposition.phi0 - legposition.phi2) * alpha1) /
		(legposition.L0 * alpha2);

	legposition.dL0 = -(L1 * dphi1 * sinf(legposition.phi0 - legposition.phi3) * alpha3 +
		L4 * dphi4 * sinf(legposition.phi0 - legposition.phi2) * alpha1) /
		alpha2;


	legposition.L0 = present.L0;

}

LQR::JACOBI LQR::JOINT::T2FJacobian(float phi0, float phi1, float phi2, float phi3, float phi4)
{
	float omega1 = sin(phi2 - phi3);
	float omega2 = sin(phi3 - phi4);
	float omega3 = sin(phi1 - phi2);
	float a11 = cos(phi0 - phi3);
	float a12 = sin(phi0 - phi3);
	float a21 = cos(phi0 - phi2);
	float a22 = sin(phi0 - phi2);

	T2F.J[0][0] = -L1 * a11 * omega3 / (present.L0 * omega1);
	T2F.J[0][1] = -L1 * a12 * omega3 / omega1;
	T2F.J[1][0] = -L4 * a21 * omega2 / (present.L0 * omega1);
	T2F.J[1][1] = -L4 * a22 * omega2 / omega1;

	return T2F;
}

LQR::JACOBI LQR::JOINT::F2TJacobian(float phi0, float phi1, float phi2, float phi3, float phi4)
{
	float omega1 = L4 * sin(phi3 - phi4);
	float omega2 = L1 * sin(phi1 - phi2);
	float a11 = cos(phi0 - phi3);
	float a12 = sin(phi0 - phi3);
	float a21 = cos(phi0 - phi2);
	float a22 = sin(phi0 - phi2);

	F2T.J[0][0] = present.L0 * a22 / omega2;
	F2T.J[0][1] = -present.L0 * a12 / omega1;
	F2T.J[1][0] = -a21 / omega2;
	F2T.J[1][1] = a11 / omega1;

	return F2T;
}

bool LQR::JOINT::OffGroundDection()
{
	double wheelAccelerometer = lqr.bodyAccelerometer - filter[0].Filter(legposition.d2L0) * cosf(present.theta[now]) +
		2 * legposition.dL0 * present.dtheta * sinf(present.theta[now]) + present.L0 * filter[1].Filter(present.d2theta) * sinf(present.theta[now]) +
		present.L0 * present.dtheta * present.dtheta * cosf(present.theta[now]);

	P = -feedbackTorque.F * cos(present.theta[now]) + feedbackTorque.legTorque.Tp * sin(present.theta[now]) / present.L0;//F、Tp符号相反
	FN = P + para.Mw * g + para.Mw * wheelAccelerometer;
	if ((FN < para.FN ))
	{
		return true;
	}

	return false;
}

float LQR::AccelerationSolution(float roll, float pitch, float yaw)
{
	//pitch 与 roll 互换,pitch取负号
	//weight.x = sinf(pitch);
	//weight.y = cosf(pitch) * sinf(roll);
	//weight.z = cosf(pitch) * cosf(roll);

	weight.x = arm_sin_f32(roll);
	weight.y = arm_cos_f32(roll) * arm_sin_f32(pitch);
	weight.z = arm_cos_f32(roll) * arm_cos_f32(pitch);

	tempAccelerometer.x = (accelerometer.x + weight.x) *g;
	tempAccelerometer.y = (accelerometer.y - weight.y) *g;
	tempAccelerometer.z = (accelerometer.z - weight.z) *g;

	bodyAccelerometer = accelerometerFilter.Filter(tempAccelerometer.z * weight.z -
		tempAccelerometer.x * weight.x - \
		tempAccelerometer.y * weight.y);

	return bodyAccelerometer; //过滤掉重力加速后的数据，滤波不好,静态时数值波动。
}

void LQR::UpdateSensor(DMMOTOR* jointMotor[][2], LKMOTOR* chassisMotor, IMU* imuChassis)
{
	joint[left].sensorLast = joint[left].sensorNow;
	joint[right].sensorLast = joint[right].sensorNow;

	joint[left].sensorNow.jointSensor[front] = jointMotor[left][front]->GetPosition();
	joint[left].sensorNow.jointSensor[behind] = jointMotor[left][behind]->GetPosition();

	joint[right].sensorNow.jointSensor[front] = jointMotor[right][front]->GetPosition();
	joint[right].sensorNow.jointSensor[behind] = jointMotor[right][behind]->GetPosition();

	joint[left].sensorNow.feedbackT[front] = -jointMotor[left][front]->GetTorque();
	joint[left].sensorNow.feedbackT[behind] = -jointMotor[left][behind]->GetTorque();
	joint[right].sensorNow.feedbackT[front] = jointMotor[right][front]->GetTorque();
	joint[right].sensorNow.feedbackT[behind] = jointMotor[right][behind]->GetTorque();

	imu.yaw = imuChassis->GetAngleYaw() * PI / 180.f;
	imu.pitch = imuChassis->GetAnglePitch() * PI / 180.f;//注意pitch安装位置
	imu.roll = imuChassis->GetAngleRoll() * PI / 180.f;
	float* temp = imuChassis->GetAcceleration();
	accelerometer.x = temp[0];
	accelerometer.y = temp[1];
	accelerometer.z = temp[2];

}

float LQR::Torque_Calcute(float theta, float dtheta, float x, float dx, float phi, float dphi, FUCTION_MODE funcition)
{
	float torque{};
	//if (fabs(dphi) < 0.02)
	//{
	//	dphi = 0;
	//}
	//if (fabs(dtheta) < 0.05)
	//{
	//	dtheta = 0;
	//}
	if (funcition == chassisM)
	{
		torque = k11 * theta + k12 * dtheta + k13 * x + k14 * dx + k15 * phi + k16 * dphi;
	}
	else if (funcition == jointM)
	{
		torque = k21 * theta + k22 * dtheta + k23 * x + k24 * dx + k25 * phi + k26 * dphi;
	}
	return torque;
}
void LQR::SetMode(MODE setMode)
{
	mode[now] = setMode;
}

LQR::MODE LQR::GetMode()
{
	return mode[now];
}

float LQR::GetYaw()
{
	return (joint[left].present.yaw + joint[right].present.yaw) / 2;
}

void LQR::SHUTDOWN()
{
	ctrl.chassis.chassisMotor[left]->SetTorque(0);
	ctrl.chassis.chassisMotor[right]->SetTorque(0);
	ctrl.jointMotor[left][front]->SetTorque(0.f);
	ctrl.jointMotor[left][behind]->SetTorque(0.f);
	ctrl.jointMotor[right][front]->SetTorque(0.f);
	ctrl.jointMotor[right][behind]->SetTorque(0.f);
}
