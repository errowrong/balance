#include "./Inc/device/LKmotor.h"
#include "string.h"
#include <cmath>
#include <cstring>
int16_t ttorque = 0;
LKMOTOR::LKMOTOR(MOTOR_TYPE type, const FUCTION_MODE function, const uint32_t id, CAN* hcan)
	: ID(id)
	, type(type)
	, function(function)
	, mcan(hcan)
{
	;
}

void LKMOTOR::LKmotorOntimer(uint8_t odata[8])//广播发送
{
	int16_t temp;
	uint16_t id = 2 * (ID - 1);
	/* 限制输入的参数在定义的范围内 */

	temp = T2C(setTorque);
	temp = LIMIT_MIN_MAX(temp, C_MIN, C_MAX);
	
	ttorque = temp;
	/* 根据传输协议，把数据转换为CAN命令数据字段并存入输出缓冲区*/

	odata[id] = temp & 0x00ff;
	odata[id + 1] = (temp & 0xff00) >> 8;

}

LKMOTOR& LKMOTOR::LKmotorDecode(uint8_t idata[][8])
{
	uint8_t id = this->ID - 0x01;
	angle[last] = angle[now];
	angle[now] = (idata[id][7] << 8) | idata[id][6];
	curSpeed = (idata[id][5] << 8) | idata[id][4];
	speed_real_ = 0.15f * speed_real_ + 0.85f * curSpeed * PI / 180.f;
	current = (idata[id][3] << 8) | idata[id][2];//转换为实际电流
	deltaAngle = GetDelta(angle[now] ,angle[last], 32768);

	torqueCurrent = current * 33 / 2048;
	torque = torqueCurrent * KT;

	setCurrent = setTorque / KT;

}

void LKMOTOR::LKmotorTransmit()
{
	mcan->Transmit(0x280, mcan->LKmotor_temp_data, 8);
}
float LKMOTOR::GetTorque()
{
	return torque;
}

float LKMOTOR::GetAngularVelocity()
{
	return speed_real_;//返回值为rad/s。
}

float LKMOTOR::GetPosition()
{
	return angle[now];
}

float LKMOTOR::GetSpeed()
{
	return curSpeed;
}

float LKMOTOR::SetTorque(float settorque)
{
	setTorque = Limit(settorque, 7.5f, -7.5f);
}

int16_t LKMOTOR::T2C(float setTorque)
{
	//return (setTorque * 370 - 14 * setTorque * setTorque);
	return  setTorque / 0.00512f;
	
}



#if TEST_LKMOTOR
void LKMOTOR::Single_Motor_Tx(CAN can, uint16_t ID, MODE mode, uint8_t* odata)
{
	uint8_t data[8]{};
	if (odata != NULL)
	{
		memcpy(&data, odata, sizeof(uint8_t) * 8);
	}
	data[0] = mode;

	switch (mode)
	{
	case w_pid:
		Set_PID(speed_pid[0], speed_pid[1], position_pid[0], position_pid[1], torque_pid[0], torque_pid[1], data);
		break;
	case w_acceleration:
		Set_Acceleration(acceleration, data);
		break;
	case w_encode:
		Set_Offset(encodeOffset, data);
		break;
	case(torque_control):
		Torque_Control(torque, data);
		break;
	case(speed_control):
		Speed_Control(setspeed * 100, data);
		break;
	case(angle_control):
		Angle_Control(spinDirection, angle[now], spin_speed, data);
		break;
	default:
		break;
	}
	can.canState = can.Transmit(ID, data);

}

void LKMOTOR::Single_Motor_Decode(CAN can, uint16_t ID, MODE mode)
{
	ID = this->ID - 0x141;
	switch (mode)
	{
	case r_pid:
		PID_Decode(can, ID);
		break;
	case r_acceleration:
		Acceleration_Decode(can, ID);
		break;
	case r_encode:
		Encode_Decode(can, ID);
		break;
	case r_multi_circle:
		Multi_Circle_Decode(can, ID);
		break;
	case r_single_circle:
		Single_Circle_Decode(can, ID);
		break;
	case state1:
		State1_Decode(can, ID);
		break;
	case state2:
		State2_Decode(can, ID);
		break;
	case torque_control:
		State2_Decode(can, ID);
		break;
	case speed_control:
		State2_Decode(can, ID);
		break;
	case angle_control:
		State2_Decode(can, ID);
		break;
	default:
		break;
	}
}

void LKMOTOR::Set_PID(uint8_t speed_P, uint8_t speed_I, uint8_t position_P, uint8_t position_I, uint8_t torque_P, uint8_t torque_I, uint8_t* odata)
{
	odata[2] = position_P;
	odata[3] = position_I;
	odata[4] = speed_P;
	odata[5] = speed_I;
	odata[6] = torque_P;
	odata[7] = torque_I;
}

void LKMOTOR::Set_Acceleration(uint32_t acceleration, uint8_t* odata)
{
	odata[4] = *(uint8_t*)(&acceleration);
	odata[5] = *((uint8_t*)(&acceleration) + 1);
	odata[6] = *((uint8_t*)(&acceleration) + 2);
	odata[7] = *((uint8_t*)(&acceleration) + 3);
}

void LKMOTOR::Set_Offset(uint32_t encodeOffset, uint8_t* odata)
{
	odata[6] = *(uint8_t*)(&encodeOffset);
	odata[7] = *((uint8_t*)(&encodeOffset) + 1);
}

void LKMOTOR::Torque_Control(uint16_t torque, uint8_t* odata)
{
	odata[4] = *(uint8_t*)(&torque);
	odata[5] = *((uint8_t*)(&torque) + 1);
}

void LKMOTOR::Speed_Control(uint32_t speed, uint8_t* odata)
{
	odata[4] = *(uint8_t*)(&speed);
	odata[5] = *((uint8_t*)(&speed) + 1);
	odata[6] = *((uint8_t*)(&speed) + 2);
	odata[7] = *((uint8_t*)(&speed) + 3);
}

void LKMOTOR::Angle_Control(uint8_t spinDirection, uint16_t angle, uint16_t spin_speed, uint8_t* odata)
{
	odata[1] = spinDirection;
	odata[2] = *(uint8_t*)(&spin_speed);
	odata[3] = *((uint8_t*)(&spin_speed) + 1);
	odata[4] = *(uint8_t*)(&angle);
	odata[5] = *((uint8_t*)(&angle) + 1);
}

void LKMOTOR::PID_Decode(CAN can, uint16_t ID)
{
	position_pid[P] = can.LKmotor_data[ID][2];
	position_pid[I] = can.LKmotor_data[ID][3];
	speed_pid[P] = can.LKmotor_data[ID][4];
	speed_pid[I] = can.LKmotor_data[ID][5];
	torque_pid[P] = can.LKmotor_data[ID][6];
	torque_pid[I] = can.LKmotor_data[ID][7];
}

void LKMOTOR::Acceleration_Decode(CAN can, uint16_t ID)
{
	acceleration = can.LKmotor_data[ID][4] | can.LKmotor_data[ID][5] << 8 | can.LKmotor_data[ID][6] << 16 | can.LKmotor_data[ID][7] << 24;
}

void LKMOTOR::Encode_Decode(CAN can, uint16_t ID)
{
	encoder = can.LKmotor_data[ID][2] | can.LKmotor_data[ID][3] << 6;
	encoderRaw = can.LKmotor_data[ID][4] | can.LKmotor_data[ID][5] << 6;
	encodeOffset = can.LKmotor_data[ID][6] | can.LKmotor_data[ID][7] << 6;
}

void LKMOTOR::Multi_Circle_Decode(CAN can, uint16_t ID)
{
	multi_circle = can.LKmotor_data[ID][1] | can.LKmotor_data[ID][2] << 8 | can.LKmotor_data[ID][3] << 16 | can.LKmotor_data[ID][4] << 24
		| can.LKmotor_data[ID][5] << 32 | can.LKmotor_data[ID][6] << 40 | can.LKmotor_data[ID][7] << 48;
}

void LKMOTOR::Single_Circle_Decode(CAN can, uint16_t ID)
{
	curcircle = can.LKmotor_data[ID][4] | can.LKmotor_data[ID][5] << 8 | can.LKmotor_data[ID][6] << 16 | can.LKmotor_data[ID][7] << 24;;
}

void LKMOTOR::State1_Decode(CAN can, uint16_t ID)
{
	temperature = can.LKmotor_data[ID][1];
	voltage = can.LKmotor_data[ID][3] | can.LKmotor_data[ID][4] << 8;
	errorState = can.LKmotor_data[ID][7];
}

void LKMOTOR::State2_Decode(CAN can, uint16_t ID)
{
	temperature = can.LKmotor_data[ID][1];
	current = can.LKmotor_data[ID][2] | can.LKmotor_data[ID][3] << 8;
	curspeed = can.LKmotor_data[ID][4] | can.LKmotor_data[ID][5] << 8;
	encoder = can.LKmotor_data[ID][6] | can.LKmotor_data[ID][7] << 6;
}
#endif // TSET_LKMOTOR