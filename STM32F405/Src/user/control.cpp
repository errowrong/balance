#include "./Inc/basic/tim.h"
#include "./Inc/device/RC.h"
#include "./Inc/device/imu.h"
#include "./Inc/device/DJImotor.h"
#include "./Inc/user/LQR.h"
#include "./Inc/user/control.h"
#include "./Inc/device/judgement.h"
#include "./Inc/device/nuc.h"

void CONTROL::Init(std::vector<DJIMOTOR*> DIJmotor, std::vector<LKMOTOR*> LKmotor, std::vector<DMMOTOR*> DMmotor)
{
	int num1{}, num2{}, num3{}, num4{}, num5{};
	for (int i = 0; i < DIJmotor.size(); i++)
	{
		switch (DIJmotor[i]->function)
		{
		case(FUCTION_MODE::pantileM):
			pantile.pantileMotor[num2++] = DIJmotor[i];
			break;
		case(FUCTION_MODE::shooterM):
			shooter.shooterMotor[num3++] = DIJmotor[i];
			DIJmotor[i]->output = true;
			break;
		case(FUCTION_MODE::supplyM):
			shooter.supplyMotor[num4] = DIJmotor[i];
			shooter.supplyMotor[num4++]->aceSpeed = para.supplySpd;
			//shooter.supplyMotor[num4++]->output = true;
			break;
		default:
			break;
		}
	}
	for (int i = 0; i < LKmotor.size(); i++)
	{
		chassis.chassisMotor[num1++] = LKmotor[i];
	}

	int k = 0;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			jointMotor[i][j] = DMmotor[k++];
		}
	}

	pantile.markPitch = para.initialMotorPitch;
	pantile.markImuYaw = 0;

	lqr.joint[left].aim.L0 = para.initialL0;
	lqr.joint[right].aim.L0 = para.initialL0;

	lqr.joint[left].leftOrRight = false;
	lqr.joint[right].leftOrRight = true;
	lqr.joint[left].leg_kp = 40;
	lqr.joint[right].leg_kp = 40;
}

void CONTROL::Update()
{
	uint8_t mode = lqr.GetMode();
	float deltaYaw{};
	switch (mode)
	{
	case LQR::MODE::UNFORCE:
		//pantile.markYaw = pantile.pantileMotor[PANTILE::YAW]->GetAngle();
		pantile.markImuYaw = imuPantile.GetAngleYaw();
		pantile.pantileMotor[PANTILE::YAW]->output = false;

		pantile.markPitch = pantile.pantileMotor[PANTILE::PITCH]->GetAngle();
		pantile.pantileMotor[PANTILE::PITCH]->output = false;

		shooter.supplyMotor[0]->setCircle = shooter.supplyMotor[0]->curCircle;
		shooter.supplyMotor[0]->output = false;

		shooter.isFire = false;
		shooter.openRub = false;
		aimL0 = para.initialL0;
		aimYaw = imuChassis.GetAngleYaw() * PI / 180.f;
		chassis.moveSpeed = 0; 
		autoAimFlag = false;
		
		break;
	case LQR::MODE::RESET:
		pantile.markImuYaw = pantile.initialImuYaw;
		pantile.markPitch = para.initialMotorPitch;
		shooter.isFire = false;
		shooter.openRub = false;
		//aimL0 = Ramp(para.initialL0, aimL0, 0.001);
		//aimYaw = Ramp(para.initialMotorYaw, aimYaw, 0.001);
		break;
	case LQR::MODE::FOLLOW:
		deltaYaw = 1.4 * GetDelta(para.initialMotorYaw, pantile.pantileMotor[PANTILE::YAW]->GetAngle(), 8192);//计算云台误差角
		/*aimYaw = Ramp(aimYaw + deltaYaw, aimYaw, 0.1);*/
		
		chassis.followYaw = GetDelta(imuChassis.GetAngleYaw() * PI / 180.f - deltaYaw * PI / 8192.f, 0, 2 * PI);
		if (fabs(chassis.followYaw - aimYaw) > PI)//解决-PI与PI跳变
		{
			aimYaw = chassis.followYaw;
		}
		else
		{
			float step;
			if (fabs(chassis.followYaw - aimYaw) > PI / 2)
			{
				step = 0.015;
			}
			else
			{
				step = 0.03;
			}
			aimYaw = Ramp(chassis.followYaw, aimYaw, step);
		}
		break;
	case LQR::MODE::SIDEWAYS:

		chassis.sideYaw = GetDelta((GetDelta((imuPantile.GetAngleYaw() - pantile.correctYaw), 0, 360) + 90)* PI / 180.f , 0, 2 * PI);

			if (GetDelta(chassis.sideYaw - aimYaw, 0, 2 * PI) > 0.04)//侧面对敌
			{
				SetYaw(0.04);
			}
			else if (GetDelta(chassis.sideYaw - aimYaw, 0, 2 * PI) < -0.04)
			{
				SetYaw(-0.04);
			}

		break;
	case LQR::MODE::ROTATE:

		SetYaw(0.05);
		break;
	case LQR::MODE::NORMAL:
		deltaYaw = GetDelta(para.initialMotorYaw, pantile.pantileMotor[PANTILE::YAW]->GetAngle(), 8192);
		pantile.markImuYaw = GetDelta(imuPantile.GetAngleYaw() + deltaYaw * 360.f / 8192.f, 0, 360.f);
		break;
	case LQR::MODE::ENMERGE:
		pantile.markImuYaw = pantile.initialImuYaw;
		pantile.markPitch = para.initialMotorPitch;
		shooter.isFire = false;

		aimL0 = para.initialL0;
		aimYaw = imuChassis.GetAngleYaw() * PI / 180.f;
	default:
		break;
	}

	if (mode != LQR::MODE::SIDEWAYS && chassis.sideFlag)
	{
		chassis.sideFlag = false;
		chassis.sideYaw = 0;
	}

	if (mode != LQR::MODE::UNFORCE )
	{
		if (!pantile.pantileMotor[PANTILE::YAW]->output)
		{
			pantile.pantileMotor[PANTILE::YAW]->output = true;
		}
		if (!pantile.pantileMotor[PANTILE::PITCH]->output)
		{
			pantile.pantileMotor[PANTILE::PITCH]->output = true;
		}
		if (!shooter.supplyMotor[0]->output)
		{
			shooter.supplyMotor[0]->output = true;
		}
	}

	pantile.Update();
	shooter.Update();
	lqr.ModeUpdate(jointMotor, chassis.chassisMotor, &imuChassis, &aimL0, chassis.movex, aimYaw, aimPitch);

}

void CONTROL::PANTILE::Update()
{
	/*if(lqr.mode!=LQR::MODE::SIDEWAYS)*/
	if (fabs(pantileMotor[PANTILE::TYPE::YAW]->GetAngle() - para.initialMotorYaw) < 5.f)
	{
		initialImuYaw = imuPantile.GetAngleYaw();
		correctYaw = GetDelta(imuPantile.GetAngleYaw() - imuChassis.GetAngleYaw(), 0, 360.f);
	}

	//markPitch = Limit(markPitch, para.pitchMax, para.pitchMin);
	markImuYaw = GetDelta(markImuYaw, 0, 360.f);
	//pantileMotor[PANTILE::TYPE::YAW]->SetAngle(markYaw);
	pantileMotor[PANTILE::TYPE::YAW]->SetImuVaule(markImuYaw);
	pantileMotor[PANTILE::TYPE::PITCH]->SetAngle(markPitch);

}

void CONTROL::SHOOTER::Update()
{
	//单次拨弹和连续拨弹
	//now_bullet_speed = judgement.data.ext_shoot_data_t.bullet_speed;//当前射速
	//模式选择

	if (openRub)
	{
		shooterMotor[0]->SetSpeed(-para.shootSpd,7500);
		shooterMotor[1]->SetSpeed(para.shootSpd, 7500);
	}
	else
	{
		shooterMotor[0]->SetSpeed(0);
		shooterMotor[1]->SetSpeed(0);
	}

	switch (fireMode)
	{
	case CONTROL::CLOSE:
		supplyMotor[0]->setCircle = supplyMotor[0]->curCircle;
		break;
	case CONTROL::ONEB:
		if (isFire)
		{
			supplyMotor[0]->SetCircle(ONEBULLETANGLE);
			isFire = false;
		}
		break;
	case CONTROL::THREEB:
		if (isFire)
		{
			supplyMotor[0]->SetCircle(3 * ONEBULLETANGLE);
			isFire = false;
		}
		break;
	case CONTROL::FIRE:
		if (isFire)
		{
			supplyMotor[0]->SetCircle(10*ONEBULLETANGLE);
		}
		else
		{
			supplyMotor[0]->setCircle=supplyMotor[0]->curCircle;
		}
		break;
	case CONTROL::REVERSE:
		if (isFire)
		{
			supplyMotor[0]->SetCircle(-ONEBULLETANGLE);
		}
		else
		{
			supplyMotor[0]->setCircle = supplyMotor[0]->curCircle;
		}
		break;
	default:
		//supplyMotor[0]->setCircle = supplyMotor[0]->curCircle;
		break;
	}
}

void CONTROL::KeepPantile(float imuYaw,float imuPitch)
{	
	pantile.markImuYaw = imuYaw;
	pantile.markImuPitch = imuPitch;
}

void CONTROL::PantileImuUpdate()
{
	ctrl.pantile.pantileMotor[PANTILE::TYPE::YAW]->imuValue = imuPantile.GetAngleYaw();
	ctrl.pantile.pantileMotor[PANTILE::TYPE::YAW]->imuSpeedValue = imuPantile.GetAngularVelocityYaw();
}

void CONTROL::SetPantile(float deltaYaw, float deltaPitch)
{
	if (!autoAimFlag)
	{
		pantile.pantileMotor[PANTILE::YAW]->autoAim = false;
		pantile.pantileMotor[PANTILE::PITCH]->autoAim = false;
		pantile.markImuYaw = GetDelta((pantile.markImuYaw - 0.25f * deltaYaw), 0, 360);
	
	}
	else
	{
		pantile.pantileMotor[PANTILE::YAW]->autoAim = true;
		pantile.pantileMotor[PANTILE::PITCH]->autoAim = true;

		if (nuc.update)
		{
			pantile.markImuYaw = GetDelta(imuPantile.GetAngleYaw() + nuc.yaw_err, 0, 360);// pantile.autoAimPid[0].Delta(nuc.yaw_err), 0, 360);
			nuc.update = false;
		}
	}

	if (lqr.GetMode() != LQR::MODE::ROTATE && lqr.GetMode() != LQR::MODE::FOLLOW)
	{
		pantile.markPitch += deltaPitch;
		//pantile.correctYaw = GetDelta(imuChassis.GetAngleYaw() - imuPantile.GetAngleYaw(), 0, 360);
	}
	else
	{
		float tempImuPitch = pantile.markImuPitch;//防止pitch卡住

		pantile.markImuPitch = GetDelta(pantile.markImuPitch - 0.1f * deltaPitch, 0, 360);
		pantile.markPitch = para.initialMotorPitch - pantile.pantilePid.Position((pantile.markImuPitch - imuPantile.GetAnglePitch()) * 8192.f / 360.f);
		if (pantile.markPitch > para.pitchMax || pantile.markPitch < para.pitchMin)
		{
			pantile.markImuPitch = tempImuPitch;
		}
	}

	pantile.markPitch = Limit(pantile.markPitch, para.pitchMax, para.pitchMin);
	
}

void CONTROL::PowerUpdata(float* maxSpeed)
{
	static float maxSpeedDelta = 0.005, maxSpeedDelta1 = 0.01;
	if (judgement.online)
	{
		if (judgement.data.power_heat_data_t.chassis_power_buffer >= 60.f)
		{
			*maxSpeed += maxSpeedDelta;
		}
		else if (judgement.data.power_heat_data_t.chassis_power_buffer < 60.f && judgement.data.power_heat_data_t.chassis_power_buffer > 40.f)
		{
			*maxSpeed -= maxSpeedDelta;
			//aimYaw = 0.9 * aimYaw + 0.1 * lqr.GetYaw();
		}
		else
		{
			aimYaw = lqr.GetYaw();
			*maxSpeed -= maxSpeedDelta1;
		}

		*maxSpeed = Limit(*maxSpeed, para.maxMoveSpeed, 0);
	}
}

void CONTROL::SetShootRub(bool _openRub)
{
	shooter.openRub = _openRub;
}

//void CONTROL::SetMoveSpd(float moveSpd)
//{
//	moveSpd = Limit(moveSpd, -1.f, 1.f);
//	chassis.moveSpeed = moveSpd;
//	//chassis.moveSpeed = Ramp(moveSpd, chassis.moveSpeed, 0.05);
//}
void CONTROL::SetMoveSpd(float movex)
{
	movex = Limit(movex, -0.4f, 0.4f);
	chassis.movex = -movex;
	//chassis.moveSpeed = Ramp(moveSpd, chassis.moveSpeed, 0.05);
}

void CONTROL::SetYaw(float deltaYaw  )
{
	float tempYaw = imuChassis.GetAngleYaw() * PI / 180.f;
	aimYaw += deltaYaw;

	if (aimYaw > PI)
	{
		aimYaw -= 2 * PI;
	}
	else if (aimYaw < -PI)
	{
		aimYaw += 2 * PI;
	}

	if (fabs(aimYaw - tempYaw) < PI)//卡墙修正
	{
		aimYaw = omega * aimYaw + (1 - omega) * tempYaw;
	}
	
}

void CONTROL::SetL0(float setDeltaL0)
{
	aimL0 += setDeltaL0;
	aimL0 = Limit(aimL0, para.maxL0, para.minL0);
}

uint8_t CONTROL::GetFireMode()
{
	return shooter.fireMode;
}

void CONTROL::SetShootMode(FIREMODE _fireMode)
{
	shooter.fireMode = _fireMode;
}

bool CONTROL::GetShootRub()
{
	return shooter.openRub;
}

void CONTROL::IsFire(bool _isFire)
{
	shooter.isFire = _isFire;
}

