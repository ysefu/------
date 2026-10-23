#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "Clock.h"

/*----------------------------------------------------------------------------
 * 文件：Clock.c / Clock.h
 * 用处：软件实时时钟（24 小时制）+ 三个定时闹钟
 * 对应外设：无（时基 = Timer.c 的 TIM2 1ms 节拍）
 * 对应器件：OLED（显示）、PA1指示灯（到点闪烁报警）
 * 备注：1) 【进位方式】用 Timer_GetTick() 的绝对毫秒数做基准，累加到 1000ms
 *          就让秒 +1，而不是"每次调用都 +10ms"。这样即使主循环里偶尔有
 *          十几毫秒的阻塞（读 IMU / 刷 OLED），时间也不会走慢
 *       2) 【闹钟判点】只在"秒发生变化"的那一刻判断，同一秒不会重复触发
 *       3) 【报警停止】两键稳定同时按住才停止本次提醒（用户指定），
 *          停止后只有当秒钟数再次变化、并且秒数不再等于闹钟设定值时才会重新武装
 *       4) 上电默认 23:59:55，三秒后就能看到 23:59:59 → 00:00:00 的进位
 *----------------------------------------------------------------------------*/

#define CLOCK_DEFAULT_HOUR      23
#define CLOCK_DEFAULT_MINUTE    59
#define CLOCK_DEFAULT_SECOND    55

typedef struct
{
	uint32_t Revision;              /* 该路时间/状态修改版本，供页面同步 */
	ClockTime_t Time;               /* 闹钟目标时间 */
	uint8_t     Enable;             /* 0=关，1=开 */
}AlarmItem_t;

static ClockTime_t ClockNow;                        /* 当前时间 */
static uint32_t    ClockLastTick = 0;               /* 上一次加满 1 秒的时间点 */
static uint8_t     ClockInitFlag = 0;               /* 首次调用时先把基准时间记下来 */

static AlarmItem_t AlarmList[CLOCK_ALARM_NUM];       /* 三个闹钟 */

static volatile uint8_t AlarmRinging = 0;           /* 1=正在提醒 */
static uint8_t          AlarmLastSec = 0xFF;        /* 上一次判点的秒数，防止同一秒重复触发 */

/**
  * @brief  初始化：上电默认 23:59:55，三个闹钟全部关闭并清零
  * @param  无
  * @retval 无
  */
void Clock_Init(void)
{
	uint8_t i;

	ClockNow.Hour   = CLOCK_DEFAULT_HOUR;
	ClockNow.Minute = CLOCK_DEFAULT_MINUTE;
	ClockNow.Second = CLOCK_DEFAULT_SECOND;

	ClockLastTick = Timer_GetTick();
	ClockInitFlag = 1;

	for(i=0;i<CLOCK_ALARM_NUM;i++)
	{
		AlarmList[i].Time.Hour   = 0;
		AlarmList[i].Time.Minute = 0;
		AlarmList[i].Time.Second = 0;
		AlarmList[i].Enable      = 0;
		AlarmList[i].Revision    = 0;
	}

	AlarmRinging = 0;
	AlarmLastSec = 0xFF;
}

/**
  * @brief  时钟任务：主循环每 10ms 调一次，内部按毫秒节拍累加并进位
  * @param  无
  * @retval 无
  */
void Clock_Task(void)
{
	if(ClockInitFlag == 0)
	{
		ClockLastTick = Timer_GetTick();
		ClockInitFlag = 1;
	}

	while(Timer_GetTick() - ClockLastTick >= 1000)      //用 while：万一被拖慢也能补齐
	{
		ClockLastTick += 1000;

		ClockNow.Second++;
		if(ClockNow.Second >= 60)
		{
			ClockNow.Second = 0;
			ClockNow.Minute++;
			if(ClockNow.Minute >= 60)
			{
				ClockNow.Minute = 0;
				ClockNow.Hour++;
				if(ClockNow.Hour >= 24)
				{
					ClockNow.Hour = 0;                  //23:59:59 的下一秒 → 00:00:00
				}
			}
		}
	}
}

/**
  * @brief  读取当前时间
  * @param  无
  * @retval 当前时间结构（时/分/秒）
  */
ClockTime_t Clock_GetTime(void)
{
	return ClockNow;
}

/**
  * @brief  读取当前小时
  * @param  无
  * @retval 0~23
  */
uint8_t Clock_GetHour(void)
{
	return ClockNow.Hour;
}

/**
  * @brief  读取当前分钟
  * @param  无
  * @retval 0~59
  */
uint8_t Clock_GetMinute(void)
{
	return ClockNow.Minute;
}

/**
  * @brief  读取当前秒
  * @param  无
  * @retval 0~59
  */
uint8_t Clock_GetSecond(void)
{
	return ClockNow.Second;
}

/**
  * @brief  直接设置时间（越界值自动按 24 小时制取模）
  * @param  Hour   时
  * @param  Minute 分
  * @param  Second 秒
  * @retval SUCCESS=设置成功（时间照常从新值往下走）
  */
ErrorStatus Clock_SetTime(uint8_t Hour, uint8_t Minute, uint8_t Second)
{
	ClockNow.Hour   = (uint8_t)(Hour   % 24);
	ClockNow.Minute = (uint8_t)(Minute % 60);
	ClockNow.Second = (uint8_t)(Second % 60);

	ClockLastTick = Timer_GetTick();                    //从设定这一刻起重新计 1 秒

	return SUCCESS;
}

/**
  * @brief  在当前时间上加减（时间设置页用：编码器顺时针 +1、逆时针 -1）
  * @param  Unit  'H'=时、'M'=分、'S'=秒
  * @param  Delta 变化量，正数加、负数减
  * @retval 无
  * 备注：时按 0~23 循环，分/秒按 0~59 循环（题目要求的边界循环）
  */
void Clock_AddTime(uint8_t Unit, int8_t Delta)
{
	int16_t Value;

	if(Unit == 'H')
	{
		Value = (int16_t)ClockNow.Hour + Delta;
		while(Value < 0)     Value += 24;
		while(Value > 23)    Value -= 24;
		ClockNow.Hour = (uint8_t)Value;
	}
	else if(Unit == 'M')
	{
		Value = (int16_t)ClockNow.Minute + Delta;
		while(Value < 0)     Value += 60;
		while(Value > 59)    Value -= 60;
		ClockNow.Minute = (uint8_t)Value;
	}
	else if(Unit == 'S')
	{
		Value = (int16_t)ClockNow.Second + Delta;
		while(Value < 0)     Value += 60;
		while(Value > 59)    Value -= 60;
		ClockNow.Second = (uint8_t)Value;
	}
}

/**
  * @brief  设置第 Index 个闹钟的时间，并立即开启它
  * @param  Index  闹钟编号，从 1 开始（1~CLOCK_ALARM_NUM）
  * @param  Hour   时
  * @param  Minute 分
  * @param  Second 秒
  * @retval SUCCESS=成功   ERROR=编号越界
  */
ErrorStatus Clock_SetAlarm(uint8_t Index, uint8_t Hour, uint8_t Minute, uint8_t Second)
{
	if(Index < 1 || Index > CLOCK_ALARM_NUM)
	{
		return ERROR;
	}

	AlarmList[Index-1].Time.Hour   = (uint8_t)(Hour   % 24);
	AlarmList[Index-1].Time.Minute = (uint8_t)(Minute % 60);
	AlarmList[Index-1].Time.Second = (uint8_t)(Second % 60);
	AlarmList[Index-1].Enable      = ALARM_ON;

	AlarmList[Index-1].Revision++;
	return SUCCESS;
}

/**
  * @brief  设置第 Index 个闹钟的开关
  * @param  Index 闹钟编号，从 1 开始
  * @param  State ALARM_ON / ALARM_OFF
  * @retval SUCCESS=成功   ERROR=编号越界
  */
ErrorStatus Clock_SetAlarmEnable(uint8_t Index, AlarmState_t State)
{
	if(Index < 1 || Index > CLOCK_ALARM_NUM)
	{
		return ERROR;
	}

	AlarmList[Index-1].Enable = (uint8_t)State;

	AlarmList[Index-1].Revision++;
	return SUCCESS;
}

/**
  * @brief  删除第 Index 个闹钟：时间清零并关闭
  * @param  Index 闹钟编号，从 1 开始
  * @retval SUCCESS=成功   ERROR=编号越界
  */
ErrorStatus Clock_DelAlarm(uint8_t Index)
{
	if(Index < 1 || Index > CLOCK_ALARM_NUM)
	{
		return ERROR;
	}

	AlarmList[Index-1].Time.Hour   = 0;
	AlarmList[Index-1].Time.Minute = 0;
	AlarmList[Index-1].Time.Second = 0;
	AlarmList[Index-1].Enable      = ALARM_OFF;

	AlarmList[Index-1].Revision++;
	return SUCCESS;
}

/**
  * @brief  读取第 Index 个闹钟的时间
  * @param  Index 闹钟编号，从 1 开始
  * @param  Time  输出：闹钟时间
  * @retval SUCCESS=成功   ERROR=编号越界或指针为空
  */
ErrorStatus Clock_GetAlarm(uint8_t Index, ClockTime_t *Time)
{
	if(Index < 1 || Index > CLOCK_ALARM_NUM || Time == 0)
	{
		return ERROR;
	}

	*Time = AlarmList[Index-1].Time;

	return SUCCESS;
}

/**
  * @brief  读取第 Index 个闹钟的开关状态
  * @param  Index 闹钟编号，从 1 开始
  * @retval ALARM_ON / ALARM_OFF（编号越界返回 ALARM_OFF）
  */
AlarmState_t Clock_GetAlarmEnable(uint8_t Index)
{
	if(Index < 1 || Index > CLOCK_ALARM_NUM)
	{
		return ALARM_OFF;
	}

	return (AlarmState_t)AlarmList[Index-1].Enable;
}

/**
  * @brief  闹钟任务：主循环每 10ms 调一次，秒一变化就判断三个闹钟是否到点
  * @param  无
  * @retval 无
  */
void Clock_AlarmTask(void)
{
	uint8_t i;

	if(ClockNow.Second != AlarmLastSec)         //只在秒变化的那一刻判点
	{
		AlarmLastSec = ClockNow.Second;

		for(i=0;i<CLOCK_ALARM_NUM;i++)
		{
			if(AlarmList[i].Enable != ALARM_ON)
			{
				continue;
			}

			if(AlarmList[i].Time.Hour   == ClockNow.Hour &&
			   AlarmList[i].Time.Minute == ClockNow.Minute &&
			   AlarmList[i].Time.Second == ClockNow.Second)
			{
				AlarmRinging = 1;               //到点了 → PA1指示灯 开始 1Hz 闪烁
			}
		}
	}
}

/**
  * @brief  当前是否正在闹钟提醒中
  * @param  无
  * @retval 1=正在提醒（PA1指示灯 应在闪烁）  0=没有提醒
  */
uint8_t Clock_IsRinging(void)
{
	return AlarmRinging;
}

/**
  * @brief  停止本次闹钟提醒（报警期间两键同时按住时调用）
  * @param  无
  * @retval 无
  * 备注：只停闪烁，不动闹钟的开关状态，也不改闹钟时间
  */
void Clock_StopRing(void)
{
	AlarmRinging = 0;
}

uint32_t Clock_GetAlarmRevision(uint8_t Index)
{
	if(Index<1 || Index>CLOCK_ALARM_NUM) return 0;
	return AlarmList[Index-1].Revision;
}
