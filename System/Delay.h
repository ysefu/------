#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Delay.c / Delay.h
 * 用处：阻塞式微秒/毫秒延时，主要给软件 IIC、单总线这类严格时序的驱动用
 * 对应外设：SysTick（内核定时器）
 * 对应器件：OLED、MPU6050 等需要按时序翻转电平的器件
 * 备注：本文件与江协科技教程工程里的 Delay.c 完全一致，直接沿用；
 *       周期任务请不要用它做节拍，节拍统一走 Timer.c 的 TIM2
 *----------------------------------------------------------------------------*/

void Delay_us(uint32_t xus);
void Delay_ms(uint32_t xms);
void Delay_s(uint32_t xs);

#endif
