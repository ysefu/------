#include "stm32f10x.h"                  // Device header
#include "Delay.h"

/*----------------------------------------------------------------------------
 * 文件：Delay.c / Delay.h
 * 用处：提供微秒/毫秒/秒级延时。DHT11 是单总线器件，必须靠微秒级延时卡时序
 * 对应外设：SysTick（内核自带的 24 位递减计数器，这里只做阻塞延时，不产生中断）
 * 对应器件：无（被 DHT11、OLED、W25QXX 等模块调用）
 * 备注：1) 这里用 SysTick 的查询方式延时，不占用任何定时器，也不影响 TIM2 节拍
 *       2) LOAD 是 24 位，72MHz 下单次最多约 233ms，所以 Delay_us 不要传太大的值
 *----------------------------------------------------------------------------*/

#define SYSCLK_FREQ     72          //系统主频 72MHz：1us = 72 个 SysTick 计数

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，单位微秒，范围 0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = SYSCLK_FREQ * xus;                              //设置重装值
	SysTick->VAL = 0x00;                                            //清空当前计数
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;   //时钟源 HCLK，启动

	while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);       //等待计数到 0

	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;                     //关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，单位毫秒
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}

/**
  * @brief  秒级延时
  * @param  xs 延时时长，单位秒
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
}
