#include "./Inc/device/judgement.h"
#include "./Inc/user/label.h"
#include <cstdarg>
#include "imu.h"
#include "./Inc/user/control.h"
#include "./Inc/user/LQR.h"
#include"./Inc/device/supercap.h"

void Judgement::BuffData()
{
	pd_Rx = xQueueReceive(*queueHandler, m_uartrx, NULL);
	if (pd_Rx)
	{
		m_readnum = m_uart->dataNum;
		if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
		{
			memcpy(m_whand, m_uartrx, m_readnum);
			m_whand = m_whand + m_readnum;
		}
		else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
		{
			memcpy(m_whand, m_uartrx, m_readnum);
			m_whand = m_FIFO;
		}
		else
		{
			const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
			memcpy(m_whand, m_uartrx, left_size);
			m_whand = m_FIFO;
			memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
			m_whand = m_FIFO + m_readnum - left_size;
		}
		m_leftsize = m_leftsize + m_readnum;
	}

}

void Judgement::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
	huart->Init(uart_base, baud).DMARxInit();
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;

}


void Judgement::GetData(void)
{
	if (Transmit(5, m_frame) == 0)return;
	while (m_frame[0] != 0xA5)
	{
		memcpy(m_frame, m_frame + 1, 4);
		if (Transmit(1, m_frame + 4) == 0)return;
	}
	if (VerifyCRC8CheckSum(m_frame, 5))
	{
		const uint16_t datalength = static_cast<uint16_t>((m_frame[1]) | (m_frame[2] << 8));
		if (Transmit(datalength + 4, m_frame + 5) == 0)return;
		if (VerifyCRC16CheckSum(m_frame, (datalength + 9)))Decode(m_frame);
	}
}

void Judgement::DisplayStaticUI()
{
	string_data_struct_t staticStringUI;
	switch (count % 100)
	{
	//case 0:
	//{
	//	char UIFireData[5] = {};
	//	UIFireData[0] = 'F';
	//	UIFireData[1] = 'I';
	//	UIFireData[2] = 'R';
	//	UIFireData[2] = 'E';

	//	Char_Draw(&staticStringUI, "FI", UI_Graph_ADD, 7, UI_Color_Green, 30, 3, 3, 60, 820, UIFireData);//FIRE

	//	Char_ReFresh(&staticStringUI);
	//}
	case 10:
	{
		char UIModeData[5] = {};
		UIModeData[0] = 'M';
		UIModeData[1] = 'O';
		UIModeData[2] = 'D';
		UIModeData[3] = 'E';

		Char_Draw(&staticStringUI, "MO", UI_Graph_ADD, 6, UI_Color_Green, 30, 5, 3, 60, 760, UIModeData);//MODE

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 20:
	{
		char UICapData[5] = {};
		UICapData[0] = 'C';
		UICapData[1] = 'A';
		UICapData[2] = 'P';

		Char_Draw(&staticStringUI, "CA", UI_Graph_ADD, 7, UI_Color_Green, 30, 3, 3, 60, 700, UICapData);//CAP

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 30:
	{
		graphic_data_struct_t staticGraohUI0[5] = {};
		
		Circle_Draw(&staticGraohUI0[0], "ZC", UI_Graph_ADD, 0, UI_Color_Main, 3, 1550, 750, 60);
		LineDraw(&staticGraohUI0[1], "Z1", UI_Graph_ADD, 0, UI_Color_Main, 3, 1550, 750, 1550, 850);
		LineDraw(&staticGraohUI0[2], "JR5", UI_Graph_ADD, 0, UI_Color_Main, 2, 1650, 600, 1722, 600);
		LineDraw(&staticGraohUI0[3], "JL5", UI_Graph_ADD, 0, UI_Color_Main, 2, 170, 600, 242, 600);
	
		UI_ReFresh(5, staticGraohUI0);
		break;
	}
	case 40:
	{
		//char UICaptureData[7] = {};

		//UICaptureData[0] = 'O';
		//UICaptureData[1] = 'P';
		//UICaptureData[2] = 'E';
		//UICaptureData[3] = 'N';
		//UICaptureData[4] = 'R';
		//UICaptureData[5] = 'U';
		//UICaptureData[6] = 'B';

		//Char_Draw(&staticStringUI, "CPT", UI_Graph_ADD, 5, UI_Color_Green, 30, 7, 3, 60, 640, UICaptureData);//CAPTURE

		//Char_ReFresh(&staticStringUI);
		break;
	}
	case 50:
	{
		char UICapDisplayData1[3] = {};
		UICapDisplayData1[0] = '1';
		UICapDisplayData1[1] = '3';
		UICapDisplayData1[2] = 'V';

		Char_Draw(&staticStringUI, "C1", UI_Graph_ADD, 4, UI_Color_Purplish_red, 20, 3, 2, 580, 90, UICapDisplayData1);//13V

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 60:
	{
		char UICapDisplayData2[3] = {};
		UICapDisplayData2[0] = '1';
		UICapDisplayData2[1] = '9';
		UICapDisplayData2[2] = 'V';

		Char_Draw(&staticStringUI, "C2", UI_Graph_ADD, 4, UI_Color_Yellow, 20, 3, 2, 1012, 90, UICapDisplayData2);//19V

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 70:
	{
		char UICapDisplayData3[3] = {};
		UICapDisplayData3[0] = '2';
		UICapDisplayData3[1] = '3';
		UICapDisplayData3[2] = 'V';

		Char_Draw(&staticStringUI, "C2", UI_Graph_ADD, 3, UI_Color_Green, 20, 3, 2, 1300, 90, UICapDisplayData3);//23V

		Char_ReFresh(&staticStringUI);
		break;
	}
	case 80:
	{
		graphic_data_struct_t staticGraohUI3[5] = {};
		Rectangle_Draw(&staticGraohUI3[0], "CR", UI_Graph_ADD, 3, UI_Color_Green, 2, 600, 100, 1320, 125);
		LineDraw(&staticGraohUI3[1], "CL", UI_Graph_ADD, 3, UI_Color_Yellow, 2, 1032, 100, 1032, 125);
		LineDraw(&staticGraohUI3[2], "LL1", UI_Graph_ADD, 3, UI_Color_Main, 2, 480, 100, 650, 400);
		LineDraw(&staticGraohUI3[3], "LL2", UI_Graph_ADD, 3, UI_Color_Main, 2, 1440, 100, 1270, 400);

		UI_ReFresh(5, staticGraohUI3);
		break;
	}
	case 90:
	{
		graphic_data_struct_t staticGraohUI2[7] = {};
		LineDraw(&staticGraohUI2[0], "L1", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 960, 400, 960, 600);
		LineDraw(&staticGraohUI2[1], "L2", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 920, 490, 1000, 490);
		//LineDraw(&staticGraohUI2[0], "L1", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 960, 280, 960, 600);
		//LineDraw(&staticGraohUI2[1], "L2", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 880, 400, 1040, 400);
		//LineDraw(&staticGraohUI2[2], "L3", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 900, 350, 1020, 350);
		//Circle_Draw(&staticGraohUI2[3], "R1", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 960, 540, 40);
		//LineDraw(&staticGraohUI2[4], "L4", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 850, 450, 1070, 450);
		//LineDraw(&staticGraohUI2[5], "L5", UI_Graph_ADD, 1, UI_Color_Purplish_red, 2, 880, 540, 1040, 540);

		UI_ReFresh(2, staticGraohUI2);
		break;
	}
	default:
		break;
	}
}

void Judgement::DisplayPantilePosition(int16_t deltaAngle)
{
	graphic_data_struct_t PositionGraohUI0[2];
	float positionX[2], positionY[2];

	deltaAngle = GetDelta(deltaAngle, 0, 8192);

	positionX[0] = 1550 + 60 * cosf(deltaAngle * PI / 4096.f);
	positionX[1] = 1550 + 60 * cosf(deltaAngle * PI / 4096.f + PI / 2);

	positionY[0] = 750 + 60 * sinf(deltaAngle * PI / 4096.f);
	positionY[1] = 750 + 60 * sinf(deltaAngle * PI / 4096.f + PI / 2);

	if (!graphInit)
	{
		LineDraw(&PositionGraohUI0[0], "Z2", UI_Graph_ADD, 0, UI_Color_Yellow, 4, 1550, 750, positionX[0], positionY[0]);
		LineDraw(&PositionGraohUI0[1], "Z4", UI_Graph_ADD, 0, UI_Color_Green, 4, 1550, 750, positionX[1], positionY[1]);
	}
	else
	{
		LineDraw(&PositionGraohUI0[0], "Z2", UI_Graph_Change, 0, UI_Color_Yellow, 4, 1550, 750, positionX[0], positionY[0]);
		LineDraw(&PositionGraohUI0[1], "Z4", UI_Graph_Change, 0, UI_Color_Green, 4, 1550, 750, positionX[1], positionY[1]);
	}

	UI_ReFresh(2,PositionGraohUI0);
}

void Judgement::DisplayLegPosition(float phi1, float phi2, float phi3, float phi4, bool leftOrRight)
{
	graphic_data_struct_t PositionGraohUI2[5]{}, PositionGraohUI6[5]{};
	float legPosition[4][4];
	uint8_t layer{};
	char legName[4];

	if (leftOrRight)
	{
		legPosition[0][0] = 1722;
		legPosition[0][1] = 600;

		legPosition[2][0] = 1650;
		legPosition[2][1] = 600;

		legName[0] = 'JL1';
		legName[1] = 'JL2';
		legName[2] = 'JL3';
		legName[3] = 'JL4';

		layer = 9;
	}
	else
	{
		legPosition[0][0] = 242;
		legPosition[0][1] = 600;

		legPosition[2][0] = 170;
		legPosition[2][1] = 600;

		legName[0] = 'JR1';
		legName[1] = 'JR2';
		legName[2] = 'JR3';
		legName[3] = 'JR4';

		layer = 2;
	}
	
	legPosition[0][2] = legPosition[0][0] - 72 * cosf(phi1);
	legPosition[0][3] = legPosition[0][1] - 72 * sinf(phi1);

	legPosition[1][0] = legPosition[0][2];
	legPosition[1][1] = legPosition[0][3];
	legPosition[1][2] = legPosition[1][0] - 130 * cosf(phi2);
	legPosition[1][3] = legPosition[1][1] - 130 * sinf(phi2);

	
	legPosition[2][2] = legPosition[2][0] - 72 * cosf(phi4);
	legPosition[2][3] = legPosition[2][1] - 72 * sinf(phi4);

	legPosition[3][0] = legPosition[2][2];
	legPosition[3][1] = legPosition[2][3];
	legPosition[3][2] = legPosition[3][0] - 130 * cosf(phi3);
	legPosition[3][3] = legPosition[3][1] - 130 * sinf(phi3);

	

	if (!graphInit)
	{
		LineDraw(&PositionGraohUI2[0], &legName[0], UI_Graph_ADD, layer, UI_Color_Yellow, 3, legPosition[0][0], legPosition[0][1], legPosition[0][2], legPosition[0][3]);
		LineDraw(&PositionGraohUI2[1], &legName[1], UI_Graph_ADD, layer, UI_Color_Green, 3, legPosition[1][0], legPosition[1][1], legPosition[1][2], legPosition[1][3]);
		LineDraw(&PositionGraohUI2[2], &legName[2], UI_Graph_ADD, layer, UI_Color_Yellow, 3, legPosition[2][0], legPosition[2][1], legPosition[2][2], legPosition[2][3]);
		LineDraw(&PositionGraohUI2[3], &legName[3], UI_Graph_ADD, layer, UI_Color_Green, 3, legPosition[3][0], legPosition[3][1], legPosition[3][2], legPosition[3][3]);
	}
	else
	{
		LineDraw(&PositionGraohUI2[0], &legName[0], UI_Graph_Change, layer, UI_Color_Yellow, 3, legPosition[0][0], legPosition[0][1], legPosition[0][2], legPosition[0][3]);
		LineDraw(&PositionGraohUI2[1], &legName[1], UI_Graph_Change, layer, UI_Color_Green, 3, legPosition[1][0], legPosition[1][1], legPosition[1][2], legPosition[1][3]);
		LineDraw(&PositionGraohUI2[2], &legName[2], UI_Graph_Change, layer, UI_Color_Yellow, 3, legPosition[2][0], legPosition[2][1], legPosition[2][2], legPosition[2][3]);
		LineDraw(&PositionGraohUI2[3], &legName[3], UI_Graph_Change, layer, UI_Color_Green, 3, legPosition[3][0], legPosition[3][1], legPosition[3][2], legPosition[3][3]);
	}

	UI_ReFresh(5, PositionGraohUI2);
}

void Judgement::DisplayCapState(uint8_t _capState)
{
	SUPERCAP::CAPSTATE capState;
	capState = (SUPERCAP::CAPSTATE)_capState;

	char capStateChar[7] = {};
	string_data_struct_t capStateData;

	switch (capState)
	{
	case SUPERCAP::DISABLE:
		capStateChar[0] = 'D';
		capStateChar[1] = 'S';
		capStateChar[2] = 'A';
		capStateChar[3] = 'B';
		capStateChar[4] = 'L';
		capStateChar[5] = 'E';
		break;
	case SUPERCAP::ENABLE:
		capStateChar[0] = 'E';
		capStateChar[1] = 'N';
		capStateChar[2] = 'A';
		capStateChar[3] = 'B';
		capStateChar[4] = 'L';
		capStateChar[5] = 'E';

		break;
	case SUPERCAP::OUTPUT:
		capStateChar[0] = 'O';
		capStateChar[1] = 'U';
		capStateChar[2] = 'T';
		capStateChar[3] = 'P';
		capStateChar[4] = 'U';
		capStateChar[5] = 'T';
		break;
	case SUPERCAP::CLOSE:
		capStateChar[0] = 'C';
		capStateChar[1] = 'L';
		capStateChar[2] = 'O';
		capStateChar[3] = 'S';
		capStateChar[4] = 'E';
		break;
	default:
		break;
	}

	if (!graphInit)
	{
		Char_Draw(&capStateData, "CO1", UI_Graph_ADD, 7, UI_Color_Green, 30, 7, 3, 280, 700, capStateChar);
	}
	else
	{
		Char_Draw(&capStateData, "CO1", UI_Graph_Change, 7, UI_Color_Green, 30, 7, 3, 280, 700, capStateChar);
	}

	Char_ReFresh(&capStateData);
}

void Judgement::DisplpayMode(uint8_t mode)
{
	LQR::MODE _mode;
	_mode = (LQR::MODE)mode;

	char modeChar[7] = {};
	string_data_struct_t modeData;

	switch (_mode)
	{
	case LQR::MODE::NORMAL:
	{
		modeChar[0] = 'N';
		modeChar[1] = 'O';
		modeChar[2] = 'R';
		modeChar[3] = 'M';
		modeChar[4] = 'A';
		modeChar[5] = 'L';
	}
	break;
	case LQR::MODE::FOLLOW:
	{
		modeChar[0] = 'F';
		modeChar[1] = 'O';
		modeChar[2] = 'L';
		modeChar[3] = 'L';
		modeChar[4] = 'O';
		modeChar[5] = 'W';
	}
	break;
	case LQR::MODE::SIDEWAYS:
	{
		modeChar[0] = 'S';
		modeChar[1] = 'I';
		modeChar[2] = 'D';
		modeChar[3] = 'E';
	}
	break;
	case LQR::MODE::UNFORCE:
	{
		modeChar[0] = 'U';
		modeChar[1] = 'N';
		modeChar[2] = 'F';
		modeChar[3] = 'O';
		modeChar[4] = 'R';
		modeChar[5] = 'C';
		modeChar[5] = 'E';
	}
	break;
	case LQR::MODE::RESET:
	{
		modeChar[0] = 'R';
		modeChar[1] = 'E';
		modeChar[2] = 'S';
		modeChar[3] = 'R';
		modeChar[4] = 'T';
	}
	break;
	case LQR::MODE::ROTATE:
	{
		modeChar[0] = 'R';
		modeChar[1] = 'O';
		modeChar[2] = 'T';
		modeChar[3] = 'A';
		modeChar[4] = 'T';
		modeChar[5] = 'E';
	}
	break;
	default:
		break;
	};
	if (!graphInit)
	{
		Char_Draw(&modeData, "MD", UI_Graph_ADD, 6, UI_Color_Green, 30, 7, 3, 280, 760, modeChar);
	}
	else
	{
		Char_Draw(&modeData, "MD", UI_Graph_Change, 6, UI_Color_Green, 30, 7, 3, 280, 760, modeChar);
	}

	Char_ReFresh(&modeData);
}

void Judgement::DisplayFireMode(uint8_t _fireMode)
{
	CONTROL::FIREMODE fireMode;
	fireMode = (CONTROL::FIREMODE)_fireMode;

	char fireModeChar[7] = {};
	string_data_struct_t captureData;

	switch (fireMode)
	{
	case CONTROL::CLOSE:
		fireModeChar[0] = 'C';
		fireModeChar[1] = 'L';
		fireModeChar[2] = 'O';
		fireModeChar[3] = 'S';
		fireModeChar[4] = 'E';
		break;
	case CONTROL::ONEB:
		fireModeChar[0] = 'S';
		fireModeChar[1] = 'I';
		fireModeChar[2] = 'N';
		fireModeChar[3] = 'G';
		fireModeChar[4] = 'E';
		break;
	case CONTROL::THREEB:
		fireModeChar[0] = 'T';
		fireModeChar[1] = 'H';
		fireModeChar[2] = 'R';
		fireModeChar[3] = 'R';
		fireModeChar[4] = 'E';
		break;
	case CONTROL::FIRE:
		fireModeChar[0] = 'F';
		fireModeChar[1] = 'I';
		fireModeChar[2] = 'R';
		fireModeChar[3] = 'E';
		break;
	default:
		break;
	}
	if (!graphInit)
	{
		Char_Draw(&captureData, "CO", UI_Graph_ADD, 5, UI_Color_Green, 30, 5, 3, 280, 820, fireModeChar);
	}
	else
	{
		Char_Draw(&captureData, "CO", UI_Graph_Change, 5, UI_Color_Green, 30, 5, 3, 280, 820, fireModeChar);
	}

	Char_ReFresh(&captureData);
}

void Judgement::DisplayCapVoltage(float capVoltage)
{
	uint32_t voltagePos{};
	graphic_data_struct_t voltageData;

	voltagePos = (capVoltage - 13.f) * 720 / (23.f - 13.f) + 600;

	if (voltagePos < 600)
	{
		voltagePos = 600;
	}
	else if (voltagePos > 1320)
	{
		voltagePos = 1320;
	}

	if (!graphInit)
	{
		LineDraw(&voltageData, "VD", UI_Graph_ADD, 3, UI_Color_Yellow, 25, 600, 112, voltagePos, 112);
	}
	else
	{
		LineDraw(&voltageData, "VD", UI_Graph_Change, 3, UI_Color_Yellow, 25, 600, 112, voltagePos, 112);
	}

	UI_ReFresh(1, &voltageData);
}

void Judgement::SendData(void)
{

	robotId = data.robot_status_t.robot_id;
	clientId = robotId | 0x100;

	if (count < 200)
	{
		DisplayStaticUI();
	}
	else
	{
		switch (count % 27)
		{
		case 0:
		{
			DisplayPantilePosition(DJI_can2_motor[3].GetAngle() - para.initialMotorYaw);//yaw电机
			break;
		}
		case 3:
		{
			DisplayCapState(supercap.Cap.capState);
			break;
		}
		case 6:
		{
			DisplayCapVoltage(supercap.Cap.capVoltage);
			break;
		}
		case 9:
		{
			//DisplayFireMode(ctrl.GetFireMode());
			break;
		}
		case 12:
		{
			DisplpayMode(lqr.GetMode());
			break;
		}
		case 15:
		{
			DisplayLegPosition(lqr.joint[right].legposition.phi1, lqr.joint[right].legposition.phi2,
				lqr.joint[right].legposition.phi3, lqr.joint[right].legposition.phi4, true);
		}
		case 18:
		{
			DisplayLegPosition(lqr.joint[left].legposition.phi1, lqr.joint[left].legposition.phi2,
				lqr.joint[left].legposition.phi3, lqr.joint[left].legposition.phi4, false);
		}
		default:
			break;
		}
	}
	if (count > 500)
	{
		graphInit = true;
	}
	count++;

}

void Judgement::Decode(uint8_t* m_frame)//未完全
{
	const uint16_t cmdID = static_cast<uint16_t>(m_frame[5] | m_frame[6] << 8);
	uint8_t* rawdata = &m_frame[7];
	switch (cmdID)
	{
	case 0x0001:
		data.game_status_t.game_type = static_cast<uint8_t>(rawdata[0] & 0x0F);
		data.game_status_t.game_progress = static_cast<uint8_t>(rawdata[0] >> 4);
		data.game_status_t.stage_remain_time = static_cast<uint16_t>(rawdata[1] | rawdata[2] << 8);
		break;
		//data.ext_game_status_t.SyncTimeStamp

	case 0x0002:
		data.game_result_t.winner = rawdata[0];
		break;

	case 0x0003:
		data.game_robot_HP_t.red_1_robot_HP = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.game_robot_HP_t.red_2_robot_HP = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.game_robot_HP_t.red_3_robot_HP = static_cast<uint16_t>(rawdata[4] | rawdata[5] << 8);
		data.game_robot_HP_t.red_4_robot_HP = static_cast<uint16_t>(rawdata[6] | rawdata[7] << 8);
		data.game_robot_HP_t.red_5_robot_HP = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.game_robot_HP_t.red_7_robot_HP = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.game_robot_HP_t.red_outpost_HP = static_cast<uint16_t>(rawdata[12] | rawdata[13] << 8);
		data.game_robot_HP_t.red_base_HP = static_cast<uint16_t>(rawdata[14] | rawdata[15] << 8);
		data.game_robot_HP_t.blue_1_robot_HP = static_cast<uint16_t>(rawdata[16] | rawdata[17] << 8);
		data.game_robot_HP_t.blue_2_robot_HP = static_cast<uint16_t>(rawdata[18] | rawdata[19] << 8);
		data.game_robot_HP_t.blue_3_robot_HP = static_cast<uint16_t>(rawdata[20] | rawdata[21] << 8);
		data.game_robot_HP_t.blue_4_robot_HP = static_cast<uint16_t>(rawdata[22] | rawdata[23] << 8);
		data.game_robot_HP_t.blue_5_robot_HP = static_cast<uint16_t>(rawdata[24] | rawdata[25] << 8);
		data.game_robot_HP_t.blue_7_robot_HP = static_cast<uint16_t>(rawdata[26] | rawdata[27] << 8);
		data.game_robot_HP_t.blue_outpost_HP = static_cast<uint16_t>(rawdata[28] | rawdata[29] << 8);
		data.game_robot_HP_t.blue_base_HP = static_cast<uint16_t>(rawdata[30] | rawdata[31] << 8);
		break;
	case 0x0101:
		data.event_data_t.event_data = u32_to_float(&rawdata[0]);
		break;
	case 0x0102:
		data.ext_supply_projectile_action_t.reserved = rawdata[0];
		data.ext_supply_projectile_action_t.supply_robot_id = rawdata[1];
		data.ext_supply_projectile_action_t.supply_projectile_step = rawdata[2];
		data.ext_supply_projectile_action_t.supply_projectile_num = rawdata[3];
		break;
	case 0x0104:
		data.referee_warning_t.level = rawdata[0];
		data.referee_warning_t.foul_robot_id = rawdata[1];
		data.referee_warning_t.count = rawdata[2];
		break;
	case 0x0105:
		data.dart_dart_info_t.dart_info = rawdata[0];
		data.dart_dart_info_t.dart_remaining_time = rawdata[1];
		break;
	case 0x0201:
		data.robot_status_t.robot_id = rawdata[0];
		online = true;
		data.robot_status_t.robot_level = rawdata[1];
		data.robot_status_t.current_HP = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.robot_status_t.maximum_HP = static_cast<uint16_t>(rawdata[4] | rawdata[5] << 8);
		data.robot_status_t.shooter_barrel_cooling_value = static_cast<uint16_t>(rawdata[6] | rawdata[7] << 8);
		data.robot_status_t.shooter_barrel_heat_limit = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.robot_status_t.chassis_power_limit = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.robot_status_t.power_management_gimbal_output = static_cast<uint16_t>(rawdata[12] & 0x01);
		data.robot_status_t.power_management_chassis_output = static_cast<uint16_t>(rawdata[12] & 0x02) > 1;
		data.robot_status_t.power_management_shooter_output = static_cast<uint16_t>(rawdata[12] & 0x04) > 2;
		break;
	case 0x0202:
		powerheatready = true;
		data.power_heat_data_t.chassis_voltage = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.power_heat_data_t.chassis_current = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.power_heat_data_t.chassis_power = u32_to_float(&rawdata[4]);
		data.power_heat_data_t.chassis_power_buffer = static_cast<uint16_t>(rawdata[8] | rawdata[9] << 8);
		data.power_heat_data_t.shooter_17mm_1_barrel_heat = static_cast<uint16_t>(rawdata[10] | rawdata[11] << 8);
		data.power_heat_data_t.shooter_17mm_2_barrel_heat = static_cast<uint16_t>(rawdata[12] | rawdata[13] << 8);
		data.power_heat_data_t.shooter_42mm_barrel_heat = static_cast<uint16_t>(rawdata[14] | rawdata[15] << 8);
		break;
	case 0x0203:
		data.robot_pos_t.x = u32_to_float(&rawdata[0]);
		data.robot_pos_t.y = u32_to_float(&rawdata[4]);
		data.robot_pos_t.angle = u32_to_float(&rawdata[12]);
		break;
	case 0x0204:
		data.buff_t.recovery_buff = rawdata[0];
		data.buff_t.cooling_buff = rawdata[1];
		data.buff_t.defence_buff = rawdata[2];
		data.buff_t.vulnerability_buff = rawdata[3];
		data.buff_t.attack_buff = rawdata[4];
		break;
	case 0x0205:
		data.air_support_data_t.airforce_status = rawdata[0];
		data.air_support_data_t.time_remain = rawdata[1];
		break;
	case 0x0206:
		data.hurt_data_t.armor_id = static_cast<uint8_t>(rawdata[0] & 0x0F);
		data.hurt_data_t.HP_deduction_reason = static_cast<uint8_t>(rawdata[0] >> 4);
		break;
	case 0x0207:
		data.shoot_data_t.bullet_type = rawdata[0];
		data.shoot_data_t.shooter_number = rawdata[1];
		data.shoot_data_t.bullet_freq = rawdata[2];
		data.shoot_data_t.bullet_speed = u32_to_float(&rawdata[3]);
		if (prebulletspd != data.shoot_data_t.bullet_speed)
		{
			nBullet++;
			prebulletspd = data.shoot_data_t.bullet_speed;
		}
		break;
	case 0x0208:
		data.projectile_allowance_t.projectile_allowance_17mm = static_cast<uint16_t>(rawdata[0] | rawdata[1] << 8);
		data.projectile_allowance_t.projectile_allowance_42mm = static_cast<uint16_t>(rawdata[2] | rawdata[3] << 8);
		data.projectile_allowance_t.remaining_gold_coin = static_cast <uint16_t>(rawdata[4] | rawdata[5] << 8);
		break;

	case 0x0209:
		data.rfid_status_t.rfid_status = u32_to_float(&rawdata[0]);
		baseRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & 0x01);
		highlandRFID = static_cast<uint8_t>((data.rfid_status_t.rfid_status & 0x02) | data.rfid_status_t.rfid_status & 0x04);
		energyRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 7));
		feipoRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 8));
		outpostRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 12));
		resourseRFID = static_cast<uint8_t>(data.rfid_status_t.rfid_status & (0x01 << 16));
		break;
	case 0x020A:
		data.dart_client_cmd_t.dart_launch_opening_status = rawdata[0];
		data.dart_client_cmd_t.reserved = rawdata[1];
		data.dart_client_cmd_t.target_change_time = rawdata[2];
		data.dart_client_cmd_t.latest_launch_cmd_time = rawdata[3];
		break;
	case 0x020B:
		data.ground_robot_position_t.hero_x = u32_to_float(&rawdata[0]);
		data.ground_robot_position_t.hero_y = u32_to_float(&rawdata[4]);
		data.ground_robot_position_t.engineer_x = u32_to_float(&rawdata[8]);
		data.ground_robot_position_t.engineer_y = u32_to_float(&rawdata[12]);
		data.ground_robot_position_t.standard_3_x = u32_to_float(&rawdata[16]);
		data.ground_robot_position_t.standard_3_y = u32_to_float(&rawdata[20]);
		data.ground_robot_position_t.standard_4_x = u32_to_float(&rawdata[24]);
		data.ground_robot_position_t.standard_4_y = u32_to_float(&rawdata[28]);
		data.ground_robot_position_t.standard_5_x = u32_to_float(&rawdata[32]);
		data.ground_robot_position_t.standard_5_y = u32_to_float(&rawdata[36]);
		break;

	default:
		break;
	}
}

bool Judgement::Transmit(uint32_t read_size, uint8_t* plate)
{
	if (m_leftsize < read_size)return false;

	if ((m_rhand + read_size) < (m_FIFO + BUFSIZE))
	{
		memcpy(plate, m_rhand, read_size);
		m_rhand = m_rhand + read_size;
	}
	else if ((m_rhand + read_size) == (m_FIFO + BUFSIZE))
	{
		memcpy(plate, m_rhand, read_size);
		m_rhand = m_FIFO;
	}
	else
	{
		const uint8_t left_size = m_FIFO + BUFSIZE - m_rhand;
		memcpy(plate, m_rhand, left_size);
		memcpy(plate + left_size, m_rhand = m_FIFO, read_size - left_size);
		m_rhand = m_FIFO + read_size - left_size;
	}
	m_leftsize = m_leftsize - read_size;
	return true;
}


/************************************************绘制直线*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    开始坐标
		End_x、End_y   结束坐标
**********************************************************************************************************/
void Judgement::LineDraw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
	uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Line;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->end_x = End_x;
	image->end_y = End_y;
}

/************************************************绘制矩形*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    开始坐标
		End_x、End_y   结束坐标（对顶角坐标）
**********************************************************************************************************/
void Judgement::Rectangle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, \
	uint32_t Graph_Layer, uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t End_x, uint32_t End_y)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Rectangle;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->end_x = End_x;
	image->end_y = End_y;
}

/************************************************绘制整圆*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Start_x、Start_x    圆心坐标
		Graph_Radius  图形半径
**********************************************************************************************************/
void Judgement::Circle_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
	uint32_t Graph_Color, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Circle;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->radius = Graph_Radius;
}


/************************************************绘制圆弧*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_StartAngle,Graph_EndAngle    开始，终止角度
		Start_y,Start_y    圆心坐标
		x_Length,y_Length   x,y方向上轴长，参考椭圆
**********************************************************************************************************/
void Judgement::Arc_Draw(graphic_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, \
	uint32_t Graph_Color, uint32_t Graph_StartAngle, uint32_t Graph_EndAngle, uint32_t Graph_Width, uint32_t Start_x, \
	uint32_t Start_y, uint32_t x_Length, uint32_t y_Length)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Arc;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->start_angle = Graph_StartAngle;
	image->end_angle = Graph_EndAngle;
	image->end_x = x_Length;
	image->end_y = y_Length;
}


/************************************************绘制浮点型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_Size     字号
		Graph_Digit    小数位数
		Start_x、Start_x    开始坐标
		Graph_Float   要显示的变量
**********************************************************************************************************/
void Judgement::Float_Draw(float_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, \
	uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, float Graph_Float)
{
	int i;
	for (i = 0; i < 2 && imagename[i] != NULL; i++)
		image->figure_name[2 - i] = imagename[i];
	image->figure_tpye = UI_Graph_Float;
	image->operate_tpye = Graph_Operate;
	image->layer = Graph_Layer;
	image->color = Graph_Color;
	image->width = Graph_Width;
	image->start_x = Start_x;
	image->start_y = Start_y;
	image->start_angle = Graph_Size;
	image->end_angle = Graph_Digit;

	int32_t temp1 = Graph_Float * 1000;
	int32_t temp2 = temp1 / 1024;
	image->end_x = temp2;

	image->radius = temp1 - temp2 * 1024;//1->1.024*e-3
}


/************************************************绘制字符型数据*************************************************
**参数：*image Graph_Data类型变量指针，用于存放图形数据
		imagename[3]   图片名称，用于标识更改
		Graph_Operate   图片操作，见头文件
		Graph_Layer    图层0-9
		Graph_Color    图形颜色
		Graph_Width    图形线宽
		Graph_Size     字号
		Graph_Digit    字符个数
		Start_x、Start_x    开始坐标
		*Char_Data          待发送字符串开始地址
**********************************************************************************************************/
void Judgement::Char_Draw(string_data_struct_t* image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Layer, uint32_t Graph_Color, \
	uint32_t Graph_Size, uint32_t Graph_Digit, uint32_t Graph_Width, uint32_t Start_x, uint32_t Start_y, char* Char_Data)
{
	int i;
	for (i = 0; i < 3 && imagename[i] != NULL; i++)
		image->Graph_Control.figure_name[2 - i] = imagename[i];
	image->Graph_Control.figure_tpye = UI_Graph_Char;
	image->Graph_Control.operate_tpye = Graph_Operate;
	image->Graph_Control.layer = Graph_Layer;
	image->Graph_Control.color = Graph_Color;
	image->Graph_Control.width = Graph_Width;
	image->Graph_Control.start_x = Start_x;
	image->Graph_Control.start_y = Start_y;
	image->Graph_Control.start_angle = Graph_Size;
	image->Graph_Control.end_angle = Graph_Digit;

	for (i = 0; i < Graph_Digit; i++)
	{
		image->show_Data[i] = *Char_Data;
		Char_Data++;
	}
}

/************************************************UI删除函数（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::UIDelete(uint8_t deleteOperator, uint8_t deleteLayer)
{
	uint16_t dataLength;
	CommunatianData_graphic_t UIDeleteData;

	UIDeleteData.txFrameHeader.sof = 0xA5;
	UIDeleteData.txFrameHeader.data_length = 8;
	UIDeleteData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &UIDeleteData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	UIDeleteData.CMD = UI_CMD_Robo_Exchange;
	UIDeleteData.txID.data_cmd_id = UI_Data_ID_Del;
	UIDeleteData.txID.receiver_ID = clientId;
	UIDeleteData.txID.sender_ID = robotId;

	memcpy(m_uarttx + 5, (uint8_t*)&UIDeleteData.CMD, 8);

	m_uarttx[13] = deleteOperator;
	m_uarttx[14] = deleteLayer;

	dataLength = sizeof(CommunatianData_graphic_t) + 2;

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;
}


/************************************************UI推送函数（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::UI_ReFresh(int cnt, graphic_data_struct_t* imageData)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 6 + cnt * 15;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;
	switch (cnt)
	{
	case 1:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
		break;
	default:
		break;
	}
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, imageData, cnt * sizeof(graphic_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + cnt * sizeof(graphic_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}

void Judgement::UI_ReFresh(int cnt, float_data_struct_t* floatdata)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 6 + cnt * sizeof(graphic_data_struct_t);
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;
	switch (cnt)
	{
	case 1:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
		break;
	case 2:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
		break;
	case 5:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
		break;
	case 7:
		graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
		break;
	default:
		break;
	}
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, floatdata, cnt * sizeof(graphic_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + cnt * sizeof(graphic_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}

/************************************************UI推送字符（使更改生效）*********************************
**参数： cnt   图形个数
		 ...   图形变量参数


Tips：：该函数只能推送1，2，5，7个图形，其他数目协议未涉及
**********************************************************************************************************/
void Judgement::Char_ReFresh(string_data_struct_t* string_Data)
{
	int i, n;
	uint8_t dataLength;
	CommunatianData_graphic_t graphicData;
	memset(m_uarttx, 0, DMA_TX_SIZE);

	graphicData.txFrameHeader.sof = UI_SOF;
	graphicData.txFrameHeader.data_length = 51;
	graphicData.txFrameHeader.seq = UI_seq;
	memcpy(m_uarttx, &graphicData.txFrameHeader, (sizeof(frame_header_t)));
	AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));	//帧头CRC8校验

	graphicData.CMD = UI_CMD_Robo_Exchange;

	graphicData.txID.data_cmd_id = UI_Data_ID_DrawChar;
	graphicData.txID.sender_ID = robotId;
	graphicData.txID.receiver_ID = clientId;                          //填充操作数据

	memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

	memcpy(m_uarttx + 13, string_Data, sizeof(string_data_struct_t));
	dataLength = sizeof(CommunatianData_graphic_t) + sizeof(string_data_struct_t);

	AppendCRC16CheckSum(m_uarttx, dataLength);

	m_uart->UARTTransmit(m_uarttx, dataLength);
	UI_seq++;                                                         //包序号+1
}



