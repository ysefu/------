#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Encoder.c / Encoder.h
 * 用处：读取旋转编码器的转动方向和格数（时间加减、闹钟加减、舵机角度加减都靠它）
 * 对应外设：普通 GPIO（A/B 两相）+ TIM3（每 500us 中断采样一次）
 * 对应器件：旋钮式编码器模块（两相输出常标 CLK/DT，或 A/B）
 * 备注：1) 采样在 TIM3 中断里做，主循环里【不要】再调用 Encoder_Scan()
 *       2) Encoder_Get() 保留不足一格的零头，不会出现"转很多圈才有反应"
 *       3) 正向反向和直觉相反时，把 A/B 两条线对调即可
 *----------------------------------------------------------------------------*/

/* 初始化（配置两相引脚 + 启动 TIM3 定时采样） */
void Encoder_Init(void);

/* 中断采样函数：由 TIM3 中断每 500us 调用，主循环不要调 */
void Encoder_Scan(void);

/* 返回自上次调用以来转过的格数：正=顺时针，负=逆时针，0=没转满一格 */
int8_t Encoder_Get(void);

/* 返回累计原始计数（调试用，看有没有丢步） */
int32_t Encoder_GetCount(void);

/* 返回 A/B 两相当前电平：bit1=A，bit0=B（查线用） */
uint8_t Encoder_GetRawState(void);

/* 返回两相电平变化的总次数（查线用：转动时必须往上涨，一直为 0 就是线没接上） */
uint16_t Encoder_GetEdgeCount(void);

#endif
