#ifndef __DELAY_H__
#define __DELAY_H__

#include"stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

class DELAY
{
public:
	uint32_t  fac_us = 0;							//us延时倍乘数			   
	uint32_t fac_ms = 1000 / configTICK_RATE_HZ;	// ms延时倍乘数

	void Init(const uint32_t sysclk)
	{
		uint32_t reload;
		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//选择外部时钟  HCLK
		fac_us = SystemCoreClock / 1000000;				//不论是否使用OS,fac_us都需要使用
		reload = SystemCoreClock / 1000000;				//每秒钟的计数次数 单位为M  
		reload *= 1000000 / configTICK_RATE_HZ;			//根据configTICK_RATE_HZ设定溢出时间
													//reload为24位寄存器,最大值:16777216,在72M下,约合0.233s左右	
		fac_ms = 1000 / configTICK_RATE_HZ;				//代表OS可以延时的最少单位	   

		SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
		SysTick->LOAD = reload; 						//每1/configTICK_RATE_HZ秒中断一次	
		SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK			
	}

	void delay_us(uint32_t nus)
	{
		uint32_t ticks;
		uint32_t told, tnow, tcnt = 0;
		uint32_t reload = SysTick->LOAD;				//LOAD的值	    	 
		ticks = nus * fac_us; 						//需要的节拍数 
		told = SysTick->VAL;        				//刚进入时的计数器值
		while (1)
		{
			tnow = SysTick->VAL;
			if (tnow != told)
			{
				if (tnow < told)tcnt += told - tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
				else tcnt += reload - tnow + told;
				told = tnow;
				if (tcnt >= ticks)break;			//时间超过/等于要延迟的时间,则退出.
			}
		};
	}

	void delay_ms(uint32_t nms)
	{
		if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)//系统已经运行
		{
			if (nms >= fac_ms)						//延时的时间大于OS的最少时间周期 
			{
				vTaskDelay(nms / fac_ms);	 		//FreeRTOS延时
			}
			nms %= fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
		}
		delay_us((uint32_t)(nms * 1000));				//普通方式延时
	}

	void delay_xms(uint32_t nms)
	{
		uint32_t i;
		for (i = 0; i < nms; i++) delay_us(1000);
	}

};

extern DELAY delay;

#endif // !_DELAY_H
