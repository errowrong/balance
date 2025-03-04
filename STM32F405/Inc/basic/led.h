#ifndef __LED_H__
#define __LED_H__

#include <stm32f4xx_hal.h>
#include "./Inc/basic/gpio.h"

class LED
{
public:
	void Init(const gpio& r)
	{
		this->r = r;
		GPIO_Init(r.GPIOx, GPIO_MODE_OUTPUT_PP, GPIO_PULLUP, r.Pin);
	}
	void on()
	{
		HAL_GPIO_WritePin(r.GPIOx, r.Pin, GPIO_PIN_RESET);
	}
	void off()
	{
		HAL_GPIO_WritePin(r.GPIOx, r.Pin, GPIO_PIN_SET);
	}
private:
	gpio r{};
};


extern LED led1, led2;

#endif // !__LED_H__

