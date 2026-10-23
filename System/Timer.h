#ifndef __TIMER_H
#define __TIMER_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Timer.c / Timer.h
 * 用处：整个系统的 1ms 节拍源。所有"周期任务"都靠它判断是否到期
 * 对应外设：TIM2（72MHz / 72 = 1MHz，1000 个数 = 1ms）
 * 对应器件：无（全系统心跳）
 * 备注：TIM2 同时还是 LED1/LED2 的 PWM 时基（LED.c 里配置通道 2/3），
 *       所以 LED_Init() 必须在 Timer_Init() 之后调用
 *----------------------------------------------------------------------------*/

/* 初始化 1ms 节拍（TIM2 更新中断） */
void Timer_Init(void);

/* 读取上电到现在的毫秒数（0~4294967295ms，约 49 天回绕） */
uint32_t Timer_GetTick(void);

/* 判断某个周期任务是否到期：1=该执行了，0=还没到
   任务编号 0~15（最多 16 个任务，见 Timer.c 的 TASK_MAX） */
uint8_t Task_IsDue(uint8_t TaskID, uint32_t Period_ms);

#endif
