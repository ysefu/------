#ifndef __CLOCK_H
#define __CLOCK_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Clock.c / Clock.h
 * 用处：系统实时时钟（24 小时制）与三个定时闹钟 —— 时间设置页、定时闹钟页、
 *       串口 TIME/ALARM 命令全部读写这里的数据
 * 对应外设：无（时基来自 Timer.c 的 TIM2 1ms 节拍）
 * 对应器件：OLED（显示时间）、PA1指示灯（闹钟到点闪烁）
 * 备注：1) 不使用 STM32 内置 RTC（那需要 32.768kHz 晶振和备份域），
 *          直接用 1ms 节拍累加，验证秒/分/时进位更直观
 *       2) 上电默认时间 23:59:55（题目要求），方便几秒内就看到 23:59:59 → 00:00:00
 *----------------------------------------------------------------------------*/

#define CLOCK_ALARM_NUM     3       /* 闹钟个数（串口命令 ALARM1/2/3） */

/* 闹钟开关状态：用枚举而不是一堆 #define（标准库没有对应符号，属于本项目状态码） */
typedef enum
{
	ALARM_OFF = 0,                  /* 闹钟关闭 */
	ALARM_ON  = 1                   /* 闹钟开启 */
}AlarmState_t;

/* 时间结构：时 0~23、分 0~59、秒 0~59 */
typedef struct
{
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
}ClockTime_t;

/* 初始化：上电默认 23:59:55，三个闹钟全部关闭 */
void Clock_Init(void);

/* 时钟任务：主循环里每 10ms 调用一次，内部按 1ms 节拍累加并处理进位 */
void Clock_Task(void);

/* 读取当前时间 */
ClockTime_t Clock_GetTime(void);

/* 读取时/分/秒（单独取用方便） */
uint8_t Clock_GetHour(void);
uint8_t Clock_GetMinute(void);
uint8_t Clock_GetSecond(void);

/* 直接设置时间（自动对 24:59:59 取模），成功返回 SUCCESS */
ErrorStatus Clock_SetTime(uint8_t Hour, uint8_t Minute, uint8_t Second);

/* 在某个单位上加减：Unit 传 'H'/'M'/'S'，Delta 为 ±格数（自动循环进位） */
void Clock_AddTime(uint8_t Unit, int8_t Delta);

/* 设置第 Index 个闹钟（Index 从 1 开始），并立即开启它 */
ErrorStatus Clock_SetAlarm(uint8_t Index, uint8_t Hour, uint8_t Minute, uint8_t Second);

/* 设置第 Index 个闹钟的开关 */
ErrorStatus Clock_SetAlarmEnable(uint8_t Index, AlarmState_t State);

/* 删除第 Index 个闹钟：时间清零并关闭 */
ErrorStatus Clock_DelAlarm(uint8_t Index);

/* 读取第 Index 个闹钟的时间 */
ErrorStatus Clock_GetAlarm(uint8_t Index, ClockTime_t *Time);

/* 读取第 Index 个闹钟的开关状态 */
AlarmState_t Clock_GetAlarmEnable(uint8_t Index);

/* 闹钟任务：主循环里每 10ms 调用一次，判断是否到点 */
void Clock_AlarmTask(void);

/* 当前是否正在闹钟提醒中：1=正在提醒（PA1指示灯 应在闪烁） */
uint8_t Clock_IsRinging(void);

/* 停止本次闹钟提醒（双键同时按住时调用），不会改闹钟的开关状态 */
void Clock_StopRing(void);

/* 功能：查询第Index路闹钟的修改版本；Index=1~3，返回版本号，非法编号返回0。
 * 菜单用于同步串口写入，读取不会修改闹钟。
 */
uint32_t Clock_GetAlarmRevision(uint8_t Index);
#endif
