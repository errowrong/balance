#include "./Inc/basic/can.h"
#include "./Inc/user/label.h"
#include "string.h"

void CAN::Init(CAN_TypeDef* instance)
{
	hcan.Instance = instance;				//can基地址赋值
	hcan.Init.Prescaler = 6;				//分频系数（不减一）
	hcan.Init.Mode = CAN_MODE_NORMAL;		//普通模式
	hcan.Init.SJW = CAN_SJW_1TQ;			//同步段为1个字节
	hcan.Init.BS1 = CAN_BS1_2TQ;			//相位缓冲段1为2个字节
	hcan.Init.BS2 = CAN_BS2_4TQ;			//相位缓冲段2为4个字节
	hcan.Init.TTCM = DISABLE;				//非时间触发通信模式
	hcan.Init.ABOM = ENABLE;				//允许自动离线管理
	hcan.Init.AWUM = ENABLE;				//允许自动唤醒
	hcan.Init.NART = DISABLE;				//禁止报文自动重传，即数据只传一次
	hcan.Init.RFLM = DISABLE;				//禁止报文溢出锁定
	hcan.Init.TXFP = DISABLE;				//传输优先级，EANBLE:ID优先 DISABLE:报文优先
	HAL_CAN_Init(&hcan);					//调用HAL库初始化函数
	HAL_CAN_Transmit_IT(&hcan);				//开启CAN通信发送中断
	HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);	//开启CAN通信接收中断
	InitFilter();
}


void CAN::InitFilter()
{
	CAN_FilterConfTypeDef		CAN_FilterConfigStructure;

	if (hcan.Instance == CAN1)
	{
		CAN_FilterConfigStructure.FilterNumber = 0;								//选择过滤器0
		CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;			//过滤器关联到FIFO0
	}
	else if (hcan.Instance == CAN2)
	{
		CAN_FilterConfigStructure.FilterNumber = 14;							//选择过滤器0
		CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO1;			//过滤器关联到FIFO0
	}
	CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;				//掩码模式
	CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;				//32位宽
	CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterIdLow = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
	CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;							//接收所有数据

	CAN_FilterConfigStructure.FilterActivation = ENABLE;						//激活过滤器
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
	hcan.pTxMsg->StdId = ID;					//设置标识符
	hcan.pTxMsg->IDE = CAN_ID_STD;				//标准帧(无拓展)
	hcan.pTxMsg->RTR = CAN_RTR_DATA;			//数据帧
	hcan.pTxMsg->DLC = len;						//设置长度
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
 * @brief       CAN通信接收回调函数
 * @param       *hcan		:	CAN句柄指针
*/
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	if (hcan == &can1.hcan)
	{
		if (hcan->pRxMsg->StdId > 0x140 && hcan->pRxMsg->StdId < 0x200 || hcan->pRxMsg->StdId == 0x280)//LK电机
		{
			memcpy(can1.LKmotor_data[hcan->pRxMsg->StdId - 0x0141], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		else if (hcan->pRxMsg->StdId > 0x200 && hcan->pRxMsg->StdId < 0x280)
		{
			memcpy(can1.DJImotor_data[hcan->pRxMsg->StdId - 0x201], hcan->pRxMsg->Data, sizeof(uint8_t) * 8);
		}
		else if (hcan->pRxMsg->StdId >= 0x01 && hcan->pRxMsg->StdId <= 0x04)//区分出关节电机的数据
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
		else if (hcan->pRxMsg->StdId >= 0x01 && hcan->pRxMsg->StdId <= 0x04)//区分出关节电机的数据
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
