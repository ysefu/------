#ifndef __APP_H
#define __APP_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：App.c / App.h
 * 用处：应用层 —— 负责外设初始化顺序和主循环的任务调度
 * 对应外设：全部（TIM1/TIM2/TIM3、USART1、GPIOA/GPIOB）
 * 对应器件：OLED、KEY1/KEY2、编码器、MPU6050、SG90、2 个 LED、USB-TTL
 * 备注：main.c 里只有 App_Init() 和 App_Loop() 两句调用，业务代码全在本文件
 *----------------------------------------------------------------------------*/

/* 应用初始化：按依赖顺序初始化所有外设，并做 IMU 零偏校准 */
void App_Init(void);

/* 应用主循环：内部是 while(1)，按 1ms 节拍分派各个周期任务，不会返回 */
void App_Loop(void);

#endif
