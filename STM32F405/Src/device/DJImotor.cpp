#include "./Inc/device/DJImotor.h"
#include <cmath>

DJIMOTOR::DJIMOTOR(const MOTOR_TYPE type, const MOTOR_MODE mode, const FUCTION_MODE function, const uint32_t id, PID _speed, PID _position, PID _autoSpd, PID _autoPos)
	: ID(id)
	, type(type)
	, mode(mode)
	, function(function)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
	memcpy(&pid[autoSpd], &_autoSpd, sizeof(PID));
	memcpy(&pid[autoPos], &_autoPos, sizeof(PID));
}

DJIMOTOR::DJIMOTOR(const MOTOR_TYPE type, const MOTOR_MODE mode, const FUCTION_MODE function, const uint32_t id, PID _speed, PID _position)
	: ID(id)
	, type(type)
	, mode(mode)
	, function(function)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
	memcpy(&pid[position], &_position, sizeof(PID));
}

DJIMOTOR::DJIMOTOR(const MOTOR_TYPE type, const MOTOR_MODE mode, const FUCTION_MODE function, const uint32_t id, PID _speed)
	: ID(id)
	, type(type)
	, mode(mode)
	, function(function)
{
	getmax(type);
	memcpy(&pid[speed], &_speed, sizeof(PID));
}

void DJIMOTOR::SetCircle(float deltaAngle)
{
	if (mode == ACE)
	{
		setCircle += deltaAngle;
		setCircle = Limit(setCircle, 1e9, -1e9);
		if (setCircle >= 1e9 || setCircle <= -1e9)
		{
			setCircle = 0;
			curCircle = 0;
		}
	}
}

void DJIMOTOR::DJImotor_Ontimer(uint8_t idata[][8], uint8_t* odata)//定时器传输数据
{
	//电调反馈报文数据见《RoboMaster C620无刷电机调速器使用说明（中英日）V1.01》
	const uint32_t ID = this->ID - ID1;
	angle[now] = getword(idata[ID][0], idata[ID][1]);
	curSpeed = getword(idata[ID][2], idata[ID][3]);
	torqueCurrent = getword(idata[ID][4], idata[ID][5]);
	deltaAngle = GetDelta(angle[now], angle[last], 8192);

	if (output)
	{
		switch (mode)
		{
		case ACE:
		{
			curCircle += deltaAngle;

			//计算所转圈数
			int32_t error_curcirclr = setCircle - curCircle;
			if (abs(error_curcirclr) > 4096)
			{
				if (error_curcirclr > 0)
				{
					setSpeed = aceSpeed;
				}
				else if (error_curcirclr < 0)
				{
					setSpeed = -aceSpeed;
				}
				setCurrent = pid[speed].Delta(setSpeed - curSpeed, 1000);
			}
			else
			{
				if (abs(error_curcirclr) < 150)
				{
					setSpeed = curSpeed;
					setCurrent = 0;
				}
				else
				{
					setSpeed = pid[position].Position(error_curcirclr, 1000);
					setCurrent = pid[speed].Delta(setSpeed - curSpeed, 1000);
				}
			}
			if (curSpeed == 0 && setSpeed == 0)
			{
				if (fabs(setCircle - curCircle) < 150)
				{
					curCircle = 0;
					setCircle = 0;
				}
			}

			setCurrent = setrange(setCurrent, maxCurrent);
		}
		break;
		case POS:
		{
			const float error = GetDelta(setAngle, angle[now], 8192);
			if (std::fabs(error) < 12)
				setSpeed = 0;
			else
			{
				if (!autoAim)
				{
					setSpeed = pid[position].Position2(error, setAngle);
				}
				else
				{
					setSpeed = pid[autoPos].Position2(error, setAngle);
				}
			}
			setSpeed = setrange(setSpeed, maxSpeed);
			if (!autoAim)
			{
				setCurrent = pid[speed].Position(setSpeed - curSpeed, 5000);
			}
			else
			{
				setCurrent = pid[autoSpd].Position(setSpeed - curSpeed, 5000);
			}
			setCurrent = setrange(setCurrent, maxCurrent);
		}
		break;
		case POS2:
		{
			static float lastSet;
			float error = GetDelta(setImuValue, imuValue, 360);
			if (std::fabs(error) < 0.2)
				setSpeed = 0;
			else
			{
				if (!autoAim)
				{
					setSpeed = pid[position].Position(error);
				}
				else
				{
					setSpeed = pid[autoPos].Position3(error, GetDelta(setImuValue - lastSet, 0, 360.f));
				}

			}
			setSpeed = setrange(setSpeed, maxSpeed);
			if (!autoAim)
			{
				setCurrent = pid[speed].Position(setSpeed - imuSpeedValue, 5000);
			}
			else
			{
				setCurrent = pid[autoSpd].Position(setSpeed - imuSpeedValue, 5000);
			}
		
			setCurrent = setrange(setCurrent, maxCurrent);
			lastSet = setImuValue;
		}
		break;
		case SPD:
			setCurrent += pid[speed].Delta(setSpeed - curSpeed, 1000);
			setCurrent = setrange(setCurrent, maxCurrent);
			break;
		case TOR:
			curTorque = torqueCurrent * torqueConstant * 20 / 16384;
			setCurrent = setTorque * 16384 / (torqueConstant * 20);
			//setCurrent = Ramp(setTorque * 16384/(torqueConstant * 20 ), setCurrent, 10);
			//setCurrent = pid[position].Position(kalman.Filter(getdeltaa(setTorque - curTorque)));
			setCurrent = setrange(setCurrent, maxCurrent);
		}
	}
	else
	{
		setCurrent = 0;
	}
	angle[last] = angle[now];           //更新角度
	odata[ID * 2] = (setCurrent & 0xff00) >> 8;
	odata[ID * 2 + 1] = setCurrent & 0x00ff;      //向电调发送报文
}

void DJIMOTOR::SetImuVaule(float _setImuValue)
{
	setImuValue = _setImuValue;
}

void DJIMOTOR::SetTorque(float settorque)
{
	setTorque = Limit(settorque, 2.5, -2.5);
}

void DJIMOTOR::SetSpeed(float setspeed, float limit)
{
	setSpeed = Limit(setspeed, limit, -limit);
}

void DJIMOTOR::SetAngle(float setangle)
{
	setAngle = Limit(setangle, 8191, 0);
}

int32_t DJIMOTOR::GetSpeed()
{
	return curSpeed;
}

int32_t DJIMOTOR::GetDeltaAngle()
{
	return deltaAngle;
}

int32_t DJIMOTOR::GetAngle()
{
	return angle[now];
}

float DJIMOTOR::GetAngularVelocity()
{
	return curSpeed * 2 * PI / 60;
}

void DJIMOTOR::getmax(const MOTOR_TYPE type)
{
	switch (type)
	{
	case M3508:
		maxCurrent = 10000;
		maxSpeed = 6000;
		break;
	case M6020:
		maxCurrent = 30000;
		maxSpeed = 400;
		break;
	case M2006:
		maxCurrent = 10000;
		maxSpeed = 5000;
		break;
	default:;
	}

}


int16_t DJIMOTOR::getword(const uint8_t high, const uint8_t low)//高位低位数据拼接
{
	const int16_t word = high;
	return (word << 8) + low;
}

float DJIMOTOR::setrange(const float original, const float range)//
{
	return fmax(fmin(range, original), -range);
}
