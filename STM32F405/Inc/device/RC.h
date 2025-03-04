#ifndef __RC_H__
#define __RC_H__

#include "./Inc/basic/usart.h."
#include "./Inc/user/kalman.h"
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>

	/*
	左拨码s[0],右拨码s[1]
	上：1 中：3 下：2

	右摇杆 上下 ch[1]
	右摇杆 左右 ch[0]
	左摇杆 左右 ch[2]
	左摇杆 上下 ch[3]

	*/

//#define key_W     ((uint8_t)0x01<<0)
//#define key_S     ((uint8_t)0x01<<1)
//#define key_A     ((uint8_t)0x01<<2)
//#define key_D     ((uint8_t)0x01<<3)
//#define key_SHIFT ((uint8_t)0x01<<4)
//#define key_CTR   ((uint8_t)0x01<<5)
//#define key_Q     ((uint8_t)0x01<<6)
//#define key_E     ((uint8_t)0x01<<7)
//
//#define key_R     ((uint8_t)0x01<<0)
//#define key_F     ((uint8_t)0x01<<1)
//#define key_G     ((uint8_t)0x01<<2)
//#define key_Z     ((uint8_t)0x01<<3)
//#define key_X     ((uint8_t)0x01<<4)
//#define key_C     ((uint8_t)0x01<<5)
//#define key_V     ((uint8_t)0x01<<6)
//#define key_B     ((uint8_t)0x01<<7)

#define Key_W 0
#define Key_S 1
#define Key_D 2
#define Key_A 3
#define Key_Shift 4
#define Key_Ctrl 5
#define Key_Q 6
#define Key_E 7
#define Key_R 8
#define Key_F 9
#define Key_G 10
#define Key_Z 11
#define Key_X 12
#define Key_C 13
#define Key_V 14
#define Key_B 15

class RC
{
public:
	typedef union
	{
		struct // 用于访问键盘状态
		{
			uint16_t W : 1;
			uint16_t S : 1;
			uint16_t D : 1;
			uint16_t A : 1;
			uint16_t SHIFT : 1;
			uint16_t CTRL : 1;
			uint16_t Q : 1;
			uint16_t E : 1;
			uint16_t R : 1;
			uint16_t F : 1;
			uint16_t G : 1;
			uint16_t Z : 1;
			uint16_t X : 1;
			uint16_t C : 1;
			uint16_t V : 1;
			uint16_t B : 1;
		};
		uint16_t keys; // 用于memcpy而不需要进行强制类型转换
	} Key_t;
	typedef struct mapRC
	{
		int16_t ch[4];
		uint8_t s[2];
	};
	typedef struct mapPC
	{
		int16_t x, y, z;
		uint8_t press_l, press_r;
		Key_t key;
	};
	enum POSITION { UP = 1, DOWN, MID };

	Kalman pcKalman[2] = { {3,10.f},{3,12.f} };

	void Decode();
	void OnRC();
	void OnPC();
	void Update();
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	bool ShiftMode();

private:
	BaseType_t pd_Rx;
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN]{};
	QueueHandle_t* queueHandler = NULL;

	mapRC rc, preRc;
	mapPC pc, prePc;
	uint8_t keyCount[16], mouseCount[2];
};


extern RC rc;

#endif


