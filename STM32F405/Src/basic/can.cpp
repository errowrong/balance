#include "./Inc/basic/can.h"
#include "./Inc/user/label.h"
#include "string.h"

void CAN::Init(CAN_TypeDef* instance)
{
	hcan.Instance = instance;				//can����ַ��ֵ
	hcan.Init.Prescaler = 6;				//��Ƶϵ��������һ��
	hcan.Init.Mode = CAN_MODE_NORMAL;		//��ͨģʽ
	hcan.Init.SJW = CAN_SJW_1TQ;			//ͬ����Ϊ1���ֽ�
	hcan.Init.BS1 = CAN_BS1_2TQ;			//��λ�����1Ϊ2���ֽ�
	hcan.Init.BS2 = CAN_BS2_4TQ;			//��λ�����2Ϊ4���ֽ�
	hcan.Init.TTCM = DISABLE;				//��ʱ�䴥��ͨ��ģʽ
	hcan.Init.ABOM = ENABLE;				//�����Զ����߹���
	hcan.Init.AWUM = ENABLE;				//�����Զ�����
	hcan.Init.NART = DISABLE;				//��ֹ�����Զ��ش���������ֻ��һ��
	hcan.Init.RFLM = DISABLE;				//��ֹ�����������
	hcan.Init.TXFP = DISABLE;				//�������ȼ���EANBLE:ID���� DISABLE:��������
	HAL_CAN_Init(&hcan);					//����HAL���ʼ������
	HAL_CAN_Transmit_IT(&hcan);				//����CANͨ�ŷ����ж�
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);	//����CANͨ�Ž����ж�
	InitFilter();
}


void CAN::InitFilter()
{
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;

	if (hcan.Instance == CAN1)
	{
		CAN_FilterConfigStructure.FilterNumber = 0;								//ѡ�������0
		CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;			//������������FIFO0
	}
	else if (hcan.Instance == CAN2)
	{
		CAN_FilterConfigStructure.FilterNumber = 14;							//ѡ�������0
		CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO1;			//������������FIFO0
	}
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;				//����ģʽ
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;				//32λ��
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;							//������������

	CAN_FilterConfigStructure.FilterActivation = ENABLE;						//���������
	CAN_FilterConfigStructure.BankNumber = 0;

	HAL_CAN_ConfigFilter(&hcan, &CAN_FilterConfigStructure);

	hcan.pTxMsg = &TxMessage;
	hcan.pRxMsg = &RxMessage;
}


void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if (hcan->Instance == CAN1)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PD0     ------> CAN1_RX
		PD1     ------> CAN1_TX*/
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN1_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
		HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	}
	else if (hcan->Instance == CAN2)
	{
		/* Peripheral clock enable */
		__HAL_RCC_CAN2_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**CAN1 GPIO Configuration
		PB12     ------> CAN2_RX
		PB13     ------> CAN2_TX*/
		GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		/* Peripheral interrupt init */
		HAL_NVIC_SetPriority(CAN2_TX_IRQn, 7, 0);
		HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);
		HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 6, 0);
		HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	}
}

HAL_StatusTypeDef CAN::Transmit(const uint32_t ID, const uint8_t* const pData, const uint8_t len)
{
	hcan.pTxMsg->StdId = ID;					//���ñ�ʶ��
	hcan.pTxMsg->IDE = CAN_ID_STD;				//��׼֡(����չ)
	hcan.pTxMsg->RTR = CAN_RTR_DATA;			//����֡
	hcan.pTxMsg->DLC = len;						//���ó���
	memcpy(hcan.pTxMsg->Data, pData, len);
	if (ID == 0x280)
	{
		canState = HAL_CAN_Transmit(&hcan, 5);
	}
	else
	{
		HAL_CAN_Transmit(&hcan, 5);
	}
	//ready = status != HAL_ERROR;// == HAL_OK;
	return canState;
}

/*
 * @brief       CANͨ�Ž��ջص�����
 * @param       *hcan		:	CAN���ָ��
*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan == &can1.hcan)
	{
		if (hcan->pRxMsg->StdId > 0x140 && hcan->pRxMsg->StdId < 0x200 || hcan->pRxMsg->StdId == 0x280)//LK���
		{
			memcpy(can1.LKmotor_data[hcan->pRxMsg->StdId - 0x0141], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		else if (hcan->pRxMsg->StdId > 0x200 && hcan->pRxMsg->StdId < 0x280)
		{
			memcpy(can1.DJImotor_data[hcan->pRxMsg->StdId - 0x201], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		else if (hcan->pRxMsg->StdId >= 0x01 && hcan->pRxMsg->StdId <= 0x04)//���ֳ��ؽڵ��������
		{
			
			memcpy(can1.DMmotor_data[hcan->pRxMsg->StdId - 0x01], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		can1.pd_Rx = true;

	}
	else
	{
		if (hcan->pRxMsg->StdId > 0x140 && hcan->pRxMsg->StdId < 0x200 || hcan->pRxMsg->StdId == 0x280)
		{
			memcpy(can2.LKmotor_data, hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}

		else if (hcan->pRxMsg->StdId > 0x200 && hcan->pRxMsg->StdId < 0x280)
		{
			memcpy(can2.DJImotor_data[hcan->pRxMsg->StdId - 0x201], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		else if (hcan->pRxMsg->StdId >= 0x01 && hcan->pRxMsg->StdId <= 0x04)//���ֳ��ؽڵ��������
		{
			memcpy(can2.DMmotor_data[hcan->pRxMsg->StdId - 0x01], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		can2.pd_Rx = true;
	}

	/*#### add enable can it again to solve can receive only one ID problem!!!####**/
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
}

extern "C" void CAN1_TX_IRQHandler()
{
	HAL_CAN_IRQHandler(&can1.hcan);
}
extern "C" void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&can1.hcan);
}
extern "C" void CAN2_TX_IRQHandler()
{
	HAL_CAN_IRQHandler(&can2.hcan);
}
extern "C" void CAN2_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&can2.hcan);
}
