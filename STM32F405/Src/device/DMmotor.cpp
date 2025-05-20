#include "./Inc/device/DMmotor.h"
#include "./Inc/basic/can.h"

DMMOTOR& DMMOTOR::StateDecode(uint8_t odata[][8])
{
	uint8_t id = this->ID - DM_ID1;
	error = (ERR)(odata[id][0] >> 4);

	int p_temp{}, v_temp{}, t_temp{};

	p_temp = (odata[id][1] << 8) | (odata[id][2]);
	v_temp = (odata[id][3] << 4) | (odata[id][4] >> 4);
	t_temp = ((odata[id][4] & 0x0F) << 8) | (odata[id][5]);

	pos = uint_to_float(p_temp, P_MIN, P_MAX, 16);
	curSpeed = uint_to_float(v_temp, V_MIN, V_MAX, 12);
	torque = uint_to_float(t_temp, T_MIN, T_MAX, 12);

	angle[last] = angle[now];

	switch (position)
	{
	case L_F:
		angle[now] = pos + 2.6+1.14-6.41;
		break;
	case L_B:
		angle[now] = pos + 2.5-1.07;
		break;
	case R_F:
		angle[now] = -pos - 0.50;
		break;
	case R_B:
		angle[now] = -pos + 2.50;
		break;
	default:
		break;
	}
}

void DMMOTOR::DMmotorOntimer(DMMOTOR pthis, uint8_t* odata)
{
	uint16_t p_temp{}, v_temp{}, t_temp{}, kp_temp{}, kd_temp{};

	p_temp = float_to_uint(pthis.setPos, P_MIN, P_MAX, 16);
	v_temp = float_to_uint(pthis.setSpeed, V_MIN, V_MAX, 12);
	t_temp = float_to_uint(pthis.setTorque, T_MIN, T_MAX, 12);
	kp_temp = float_to_uint(pthis.kp, KP_MIN, KP_MAX, 12);
	kd_temp = float_to_uint(pthis.kd, KD_MIN, KD_MAX, 12);

	odata[0] = p_temp >> 8;
	odata[1] = p_temp & 0xFF;
	odata[2] = v_temp >> 4;
	odata[3] = ((v_temp) & 0xF) << 4 | (kp_temp >> 8);
	odata[4] = kp_temp & 0xFF;
	odata[5] = kd_temp >> 4;
	odata[6] = ((kd_temp & 0xF) << 4) | (t_temp >> 8);
	odata[7] = t_temp & 0xFF;
}

DMMOTOR::DMMOTOR(const uint32_t ID, FUCTION_MODE function, POSITION position, CAN* hcan)
	:ID(ID), function(function), position(position), mcan(hcan)
{}

void DMMOTOR::DMmotorTransmit(uint32_t id)
{
	if (!intial)
	{
		MotorStart(id);
		intial = true;
	}
	else
	{
		mcan->Transmit(id, mcan->DMmotor_temp_data[id - 1], 8);
	}
}

void DMMOTOR::MotorStart(uint32_t id)/* 启动电机控制 */
{
	uint8_t buf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC };
	mcan->Transmit(id, buf, 8);
}

void DMMOTOR::SetSpeed(float setspeed, float _kp, float _kd)
{
	kd = _kd;
	kp = _kp;
	setSpeed = setspeed;
}
void DMMOTOR::SetPosition(float position, float _kp, float _kd)
{
	kd = _kd;
	kp = _kp;
	setPos = position;
}

void DMMOTOR::SetTorque(float settorque, float _kp, float _kd)
{
	kd = _kd;
	kp = _kp;
	setTorque = settorque;
}
float DMMOTOR::GetPosition()
{
	return angle[now];
}

float DMMOTOR::GetTorque()
{
	return torque;
}

float DMMOTOR::GetSpeed()
{
	return curSpeed;
}