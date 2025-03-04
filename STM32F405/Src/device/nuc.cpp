#include "./Inc/device/nuc.h"
#include "./Inc/user/label.h"
#include "./Inc/user/CRC.h"
#include <math.h>
#include "./Inc/user/SolveTrajectory.h"
#include "./Inc/device/judgement.h"

void NUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr);// .DMATxInit();
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;
}

void NUC::Decode()
{
	pd_Rx = xQueueReceive(*queueHandler, rxData, NULL);
	if (pd_Rx)
	{
		update = true;
		RxNuc.header = rxData[0];
		if (!Verify_CRC16_Check_Sum(rxData, 12))
			return;
		if (RxNuc.header == 0xA5)
		{
			//RxNuc.checksum = ((uint16_t)rxData[46] << 8 | (uint16_t)rxData[47]);

			//tracking = rxData[1] & 0x01;
			//st.armor_id = (ARMOR_ID)(rxData[1] & 0x0F >> 1);
			//st.armor_num = (ARMOR_NUM)(rxData[1] & 0x80 >> 4);
			////st.reserved
			//st.xw = u8_to_float(rxData + 2);
			//st.yw = u8_to_float(rxData + 6);
			//st.zw = u8_to_float(rxData + 10);
			//st.tar_yaw = u8_to_float(rxData + 14);
			//st.vxw = u8_to_float(rxData + 18);
			//st.vyw = u8_to_float(rxData + 22);
			//st.vzw = u8_to_float(rxData + 26);
			//st.v_yaw = u8_to_float(rxData + 30);
			//st.r1 = u8_to_float(rxData + 34);
			//st.r2 = u8_to_float(rxData + 38);
			//st.dz = u8_to_float(rxData + 42);

			////if (!tracking) {
			////	same_flag++;
			////}
			////else {
			////	same_flag = 0;
			////}
			//autoSolveTrajectory(&pitch_err, &yaw_err, &TxNuc.aim_x, &TxNuc.aim_y, &TxNuc.aim_z);
			if (fabs(u8_to_float(rxData + 2)) < 30)
				yaw_err = u8_to_float(rxData + 2);
			if (fabs(u8_to_float(rxData + 6)) < 30)
				pitch_err = u8_to_float(rxData + 6) ;

			//if (!tracking && pre_yaw_err == yaw_err && pre_pitch_err == pitch_err) {
			//	same_flag++;
			//}
			//else {
			//	same_flag = 0;
			//	refresh++;
			//}
			
			//if (same_flag > 50)
			//	targeted = false;
			//else targeted = true;
		}
	}
}

void NUC::Encode(float pitch, float yaw, float roll)
{
	TxNuc.header = 0x5A;
	TxNuc.detect_color = judgement.data.robot_status_t.robot_id > 7 ? RED : BLUE; //id大于七为蓝方，则瞄准红装甲板
	TxNuc.reset_tracker = 0;
	TxNuc.reserved = 15;
	TxNuc.roll = roll;
	TxNuc.pitch = pitch;
	TxNuc.yaw =yaw;
	//TxNuc.aim_x = 0.f;
	//TxNuc.aim_y = 0.f;
	//TxNuc.aim_z = 0.f;
	TxNuc.checksum = 0;

	memcpy(txData, &TxNuc, 2);
	memcpy(txData + 2, &(TxNuc.roll), 24);
	Append_CRC16_Check_Sum(txData, 28);

	m_uart->UARTTransmit(txData, 28);	//91

}

uint16_t NUC::Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == nullptr) return 0xFFFF;
	while (dwLength--) {
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ CRC_TAB[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}

uint32_t NUC::Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return false;

	w_expected = Get_CRC16_Check_Sum(pchMessage, dwLength - 2, CRC16_INIT);
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

void NUC::Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return;

	w_crc = Get_CRC16_Check_Sum(reinterpret_cast<uint8_t*>(pchMessage), dwLength - 2, CRC16_INIT);

	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);
}