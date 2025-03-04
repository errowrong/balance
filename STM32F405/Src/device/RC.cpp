#include "./Inc/user/label.h"
#include "./Inc/device/RC.h"
#include "./Inc/user/control.h"
#include "./Inc/user/LQR.h"
#include "./Inc/device/judgement.h"
#include "./Inc/device/supercap.h"
#include "./Inc/device/supercap.h"

void RC::Decode()
{
	pd_Rx = xQueueReceive(*queueHandler, m_frame, NULL);
	if (pd_Rx)
	{
		if (m_uart->dataNum < 18)
		{
			rc.ch[0] = 0;
			rc.ch[1] = 0;
			rc.ch[2] = 0;
			rc.ch[3] = 0;
			rc.s[0] = 3;
			rc.s[1] = 1;
			return;
		}
		if ((m_frame[0] | m_frame[1] | m_frame[2] | m_frame[3] | m_frame[4] | m_frame[5]) == 0)
			return;

		rc.ch[0] = ((m_frame[0] | m_frame[1] << 8) & 0x07FF) - 1024;
		rc.ch[1] = ((m_frame[1] >> 3 | m_frame[2] << 5) & 0x07FF) - 1024;
		rc.ch[2] = ((m_frame[2] >> 6 | m_frame[3] << 2 | m_frame[4] << 10) & 0x07FF) - 1024;
		rc.ch[3] = ((m_frame[4] >> 1 | m_frame[5] << 7) & 0x07FF) - 1024;
		if (rc.ch[0] <= 8 && rc.ch[0] >= -8)rc.ch[0] = 0;
		if (rc.ch[1] <= 8 && rc.ch[1] >= -8)rc.ch[1] = 0;
		if (rc.ch[2] <= 8 && rc.ch[2] >= -8)rc.ch[2] = 0;
		if (rc.ch[3] <= 8 && rc.ch[3] >= -8)rc.ch[3] = 0;

		preRc = rc;
		prePc = pc;

		rc.s[0] = ((m_frame[5] >> 4) & 0x0C) >> 2;
		rc.s[1] = ((m_frame[5] >> 4) & 0x03);

		pc.x = m_frame[6] | (m_frame[7] << 8);
		pc.y = m_frame[8] | (m_frame[9] << 8);
		pc.z = m_frame[10] | (m_frame[11] << 8);
		pc.press_l = m_frame[12];
		pc.press_r = m_frame[13];

		uint16_t tempKey{};
		tempKey = m_frame[15] << 8 | m_frame[14];
		memcpy(&pc.key, &tempKey, sizeof(tempKey));

		for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
		{
			if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
				continue;
			// 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
			if ((pc.key.keys & j) && !(prePc.key.keys & j))
				keyCount[i]++;
		}
		if ((pc.press_l) && !(prePc.press_l))
		{
			mouseCount[0]++;
		}
		if ((pc.press_r) && !(prePc.press_r))
		{
			mouseCount[1]++;
		}
	}

}

void RC::OnRC()
{
	static float powerRcSpd{};

	if (judgement.data.robot_status_t.current_HP == 0||!judgement.data.robot_status_t.power_management_chassis_output)
	{
		lqr.SetMode(LQR::MODE::UNFORCE);
		keyCount[Key_Q] = 0;

		supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
		keyCount[Key_C] = 2;
	}

	if (rc.s[0] == UP && rc.s[1] == MID)
	{
		/*if (ShiftMode())
		{
			ctrl.KeepPantile(imuPantile.GetAngleYaw(), imuPantile.GetAnglePitch());
		}
		ctrl.SetShootRub(true);
		OnPC();
		return;*/
		lqr.SetMode(LQR::MODE::UNFORCE);
	}
	else if (rc.s[0] == DOWN && rc.s[1] == DOWN)//失力模式
	{
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
		}
		lqr.SetMode(LQR::MODE::UNFORCE);
	}
	else if (rc.s[0] == DOWN && rc.s[1] == MID)//复位模式
	{
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
		}
		lqr.SetMode(LQR::MODE::RESET);

		ctrl.SetMoveSpd(rc.ch[1] * para.moveSpd / 660.f);
		ctrl.SetYaw(-rc.ch[0] * para.turnSpd / 660.f);
		ctrl.SetL0(rc.ch[2] * para.L0Spd / 660.f);
	}
	else if (rc.s[0] == MID && rc.s[1] == DOWN)//复位模式
	{
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
		}
		lqr.SetMode(LQR::MODE::ENMERGE);

		ctrl.SetMoveSpd(rc.ch[1] * para.moveSpd / 660.f);
		ctrl.SetYaw(-rc.ch[0] * para.turnSpd / 660.f);
		ctrl.SetL0(rc.ch[2] * para.L0Spd / 660.f);

	}
	else if (rc.s[0] == MID && rc.s[1] == MID)//平衡模式
	{

		lqr.SetMode(LQR::MODE::NORMAL);
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
			powerRcSpd = para.moveSpd;
		}	

		ctrl.SetMoveSpd(rc.ch[1] * powerRcSpd / 660.f);
		ctrl.SetYaw(-rc.ch[0] * para.turnSpd / 660.f);
		ctrl.SetL0(rc.ch[2] * para.L0Spd / 660.f);
		//ctrl.SetPantile(rc.ch[2] * para.yawSpd, -rc.ch[3] * para.pitchSpd);*/
		ctrl.PowerUpdata(&powerRcSpd);
		
	}
	else if (rc.s[0] == UP && rc.s[1] == DOWN)//小陀螺
	{
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
			ctrl.KeepPantile(imuPantile.GetAngleYaw(), imuPantile.GetAnglePitch());
		}
 		lqr.SetMode(LQR::MODE::ROTATE);
		ctrl.SetPantile(rc.ch[2] * para.yawSpd ,rc.ch[3] * para.pitchSpd );
	}
	else if (rc.s[0] == MID && rc.s[1] == UP)//跟随模式
	{
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
			ctrl.KeepPantile(imuPantile.GetAngleYaw(), imuPantile.GetAnglePitch());
		}
		lqr.SetMode(LQR::MODE::FOLLOW);
		ctrl.SetMoveSpd(rc.ch[1] * para.moveSpd / 660.f);
		ctrl.SetPantile(rc.ch[0] * para.yawSpd, 0);

		if (fabs(rc.ch[2]) > 330.f)
		{
			ctrl.SetShootRub(true);
		}
		else
		{
			ctrl.SetShootRub(false);
		}
		if ((rc.ch[3]) > 330.f)
		{
			ctrl.IsFire(true);
			ctrl.SetShootMode(CONTROL::FIREMODE::FIRE);
		}
		else if (rc.ch[3] < -330.f)
		{
			ctrl.IsFire(true);
			ctrl.SetShootMode(CONTROL::FIREMODE::REVERSE);
		}
		else
		{
			ctrl.IsFire(false);
		}
		
	}
	else if (rc.s[0] == DOWN && rc.s[1] == UP)//侧面对敌
	{
		if (ShiftMode())
		{
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
			ctrl.SetShootRub(false);
		}
		lqr.SetMode(LQR::MODE::SIDEWAYS);

		ctrl.SetMoveSpd(rc.ch[1] * para.moveSpd / 660.f);
		ctrl.SetPantile(rc.ch[0] * para.yawSpd, 0);
		if (fabs(rc.ch[2]) > 330.f)
		{
			ctrl.SetShootRub(true);
		}
		else
		{
			ctrl.SetShootRub(false);
		}
		if ((rc.ch[3]) > 330.f)
		{
			ctrl.IsFire(true);
			ctrl.SetShootMode(CONTROL::FIREMODE::FIRE);
		}
		else if (rc.ch[3] < -330.f)
		{
			ctrl.IsFire(true);
			ctrl.SetShootMode(CONTROL::FIREMODE::REVERSE);
		}
		else
		{
			ctrl.IsFire(false);
		}
	}
	else if (rc.s[0] == UP && rc.s[1] == UP)
	{
		lqr.SetMode(LQR::MODE::UNFORCE);
	}

	if (preRc.s[0] == UP && preRc.s[1] == MID)
	{
		if (rc.s[0] != UP || rc.s[1] != MID)//退出PC端关闭摩擦轮
		{
			ctrl.SetShootRub(false);
			supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
		}
	}

	
}

void RC::OnPC()
{
	static float lastHp{};
	static float powerPcSpd, pcMoveSpd, pcSetYaw, pcSetPitch;
	static float deltaSpd = 0.015, deltaYaw = 0.04, deltaPitch = 0.1, deltaL0 = 0.001;
	if (lqr.GetMode() != LQR::MODE::ROTATE)
	{
		if (!pc.key.W && !pc.key.S)
		{
			deltaSpd = 0.1;
		}
		else
		{
			deltaSpd = 0.05;
		}

		if (pc.key.W || pc.key.S)
		{
			ctrl.PowerUpdata(&powerPcSpd);
		}
		else
		{
			powerPcSpd = para.pcMoveSpd;
		}

		pcMoveSpd = Ramp((pc.key.W - pc.key.S) * powerPcSpd, pcMoveSpd, deltaSpd);

		ctrl.SetMoveSpd(pcMoveSpd);
	}
	pcSetYaw = pcKalman[0].Filter(pc.x) * deltaYaw;
	pcSetPitch = pcKalman[1].Filter(pc.y) * deltaPitch;

	ctrl.SetPantile(pcSetYaw, pcSetPitch);
	if (pc.key.B)//刷新UIp
	{
		judgement.graphInit = false;
		judgement.count = 0;
		keyCount[Key_B] = 0;
	}
	
	if (pc.key.G && !pc.key.SHIFT)//调整腿长]
	{
		ctrl.SetL0(deltaL0);
	}
	else if (pc.key.G && pc.key.SHIFT)
	{
		ctrl.SetL0(-deltaL0);
	}

	if (pc.key.Z)
	{
		if (keyCount[Key_Z] != 0)
		{
			if (ctrl.GetShootRub())
			{
				ctrl.SetShootRub(false);
			}
			else
			{
				ctrl.SetShootRub(true);
			}
			keyCount[Key_Z] = 0;
		}
	}
	
	if (pc.press_r)
	{
		ctrl.autoAimFlag = true;
	}
	else
	{
		ctrl.autoAimFlag = false;
	}

	if (ctrl.GetShootRub())
	{
		switch (keyCount[Key_F] % 3)//发射模式
		{
		case 0:
			ctrl.SetShootMode(CONTROL::FIREMODE::ONEB);//单发模式
			if (mouseCount[0] != 0)
			{
				ctrl.IsFire(true);//消抖？
				mouseCount[0] = 0;
			}
			break;
		case 1:
			ctrl.SetShootMode(CONTROL::FIREMODE::THREEB);//三发模式
			if (mouseCount[0] != 0)
			{
				ctrl.IsFire(true);//消抖？
				mouseCount[0] = 0;
			}
			break;
		case 2:
			ctrl.SetShootMode(CONTROL::FIREMODE::FIRE);//连发模式
			ctrl.IsFire(pc.press_l);//消抖？
			break;
		default:
			break;
		}
	}
	else
	{
		ctrl.SetShootMode(CONTROL::FIREMODE::CLOSE);
	}

	if (pc.key.SHIFT && !pc.press_l)
	{
		ctrl.SetShootMode(CONTROL::FIREMODE::CLOSE);
	}
	else if (pc.key.SHIFT && pc.press_l)
	{
		ctrl.SetShootMode(CONTROL::FIREMODE::REVERSE);
	}


	if (lastHp == 0 && judgement.data.robot_status_t.current_HP != 0)
	{
		lqr.SetMode(LQR::MODE::RESET);
		keyCount[Key_Q] = 0;
	}
	else
	{
		switch (keyCount[Key_Q] % 3)//切换底盘模式
		{
		case 0:
			lqr.SetMode(LQR::MODE::FOLLOW);
			break;
		case 1:
			lqr.SetMode(LQR::MODE::SIDEWAYS);
			break;
		case 2:
			lqr.SetMode(LQR::MODE::RESET);
			break;
		default:
			break;
		}
	}

	switch (keyCount[Key_C] % 3)//切换超电模式
	{
	case 0:
		supercap.setstate = SUPERCAP::CAPSTATE::ENABLE;
		break;
	case 1:
		supercap.setstate = SUPERCAP::CAPSTATE::OUTPUT;
		break;
	case 2:
		supercap.setstate = SUPERCAP::CAPSTATE::CLOSE;
		break;
	default:
		break;
	}

	if (pc.key.CTRL)
	{
		keyCount[Key_Q] = 0;
		lqr.SetMode(LQR::MODE::ENMERGE);
	}


	lastHp = judgement.data.robot_status_t.current_HP;
	//if (pc.key.R)
	//{
	//	lqr.jump = true;
	//}
}

void RC::Update()
{
	OnRC();
	//OnPC();
}


void RC::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit();
	queueHandler = &huart->UartQueueHandler;
	m_uart = huart;
}

bool RC::ShiftMode()
{
	if (rc.s[0] != preRc.s[0] || rc.s[1] != preRc.s[1])
	{
		preRc = rc;
		return true;
	}
	return false;
}
