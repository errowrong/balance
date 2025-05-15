#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "./Inc/user/label.h"

void PARAMETER::Init()
{
	pitchMin = 3200, pitchMax = 4400, initialMotorPitch = 4020, initialMotorYaw = 3060;//µç»ú³õÊ¼½Ç
	pitchSpd = 0, yawSpd = 0, L0Spd = 0.005;
	mg = 11.5f;
	M = 2.1f;
	FN = 20, Mw = 2;
	shootSpd = 7000;
	initialL0 = 0.12, maxL0 = 0.25, minL0 = 0.12, L0Spd = 0.0005;
	supplySpd = 4300, moveSpd = 1.5f, turnSpd = 0.005, pcMoveSpd = 6.f, maxMoveSpeed = 10.f;
	pcMoveSpd = 2;
	yawSpd = 0.015, pitchSpd = 0.015;

}

