#include "stm32f10x.h"                  // Device header
#include "Timer.h"
#include "LED.h"
#include "Key.h"
#include "Encoder.h"
#include "OLED.h"
#include "LightSensor.h"
#include "OLED_Chinese.h"
#include "Serial.h"
#include "Servo.h"
#include "MPU6050.h"
#include "Clock.h"
#include "Menu.h"
#include "Protocol.h"
#include "App.h"

/*----------------------------------------------------------------------------
 * 文件：App.c / App.h
 * 用处：应用层调度 —— 初始化顺序 + 主循环周期任务
 * 对应外设：全部
 * 对应器件：OLED、KEY1/KEY2、编码器、MPU6050、SG90、1 个 LED、USB-TTL
 * 备注：1) 【初始化顺序的理由】
 *          ① Timer_Init() 必须最先：1ms 节拍是 Clock、IMU、舵机平滑、按键消抖的基准
 *          ② Serial_Init() 其次：后面任何一步出问题都能用串口看出来
 *          ③ LED_Init() 必须在 Timer_Init() 之后（LED 复用 TIM2 的时基做 PWM）
 *          ④ OLED 最后显示"Calibrating"，再做 MPU6050 校准（校准要 0.4 秒，
 *             期间必须让用户看到提示，不能黑屏）
 *       2) 【任务分派原则】所有周期任务都在主循环里按 Timer_GetTick() 判断是否到期，
 *          中断里只做"累加计数"和"采样编码器"。单个任务最长耗时约 3ms
 *          （读一次 MPU6050），远小于最短任务周期（10ms），
 *          所以时钟、按键、闹钟、串口、舵机 PWM 都不会被彼此拖慢
 *       3) 闹钟报警期间吞掉操作事件，仅两键同时按住可停止本次提醒，
 *          消音后等待两键全部松开，避免翻页或保存（题目硬性要求）
 *----------------------------------------------------------------------------*/

/* 任务编号（0~11，不同任务必须用不同编号） */
#define TASK_ID_PROTOCOL    0       /* 串口命令解析 */
#define TASK_ID_KEY         1       /* 按键扫描 */
#define TASK_ID_MENU        2       /* 界面逻辑与显示 */
#define TASK_ID_DISPLAY     3       /* 定时刷屏 */
#define TASK_ID_ALARM       4       /* 闹钟判点 */
#define TASK_ID_CLOCK       5       /* 时钟走秒 */
#define TASK_ID_IMU         6       /* IMU 采样与姿态解算 */
#define TASK_ID_SERVO       7       /* 舵机平滑逼近 */
#define TASK_ID_LED         8       /* 呼吸灯 */
#define TASK_ID_TRACE       9       /* 串口 TRACE 数据输出 */
#define TASK_ID_HEARTBEAT   10      /* 串口心跳（可选，调试用） */
#define TASK_ID_BOOT        11      /* 开机动画 */
#define TASK_ID_HOLD        12      /* IMU 姿态定角闭环 */

/* 报警输入捕获：直到两个键全部松开才恢复正常菜单操作。 */
static uint8_t AlarmInputCaptured=0;

static void App_ProcessMenuInput(void)
{
	if(Clock_IsRinging())
	{
		Menu_Wake(); /* 报警退出时钟待机，并保持唤醒直到消警。 */
		AlarmInputCaptured=1;
		(void)Key_GetEvent(KEY_ID_1);
		(void)Key_GetEvent(KEY_ID_2);
		(void)Encoder_Get();
		/* 检查稳定按住状态，无需两键在同一瞬间按下。 */
		if(Key_GetBothPress())
		{
			Clock_StopRing();
			LED_AlarmTask(0);
		}
		return;
	}
	if(AlarmInputCaptured)
	{
		(void)Key_GetEvent(KEY_ID_1);
		(void)Key_GetEvent(KEY_ID_2);
		(void)Encoder_Get();
		if(!Key_GetPress(KEY_ID_1) && !Key_GetPress(KEY_ID_2)) AlarmInputCaptured=0;
		return;
	}
	Menu_Task();
}

/**
  * @brief  应用初始化：按依赖顺序初始化所有外设，并做 IMU 零偏校准
  * @param  无
  * @retval 无
  */
/* 主循环协作：让IMU启动等待与校准期间的动画持续推进。 */
static void App_StartupPoll(void)
{
 Menu_BootTask();LED_BreathTask();Clock_Task();LightSensor_Update();
}
void App_Init(void)
{
	AlarmInputCaptured=0;
	/* ① 1ms 系统节拍（必须最先，其它模块都要用它计时） */
	Timer_Init();

	/* ② 串口：越早越好，后面任何一步都能用它把现象打出来 */
	Serial_Init();
	Serial_SendString("\r\n=== eXin 2026 STM32 Menu + Servo ===\r\n");
	Serial_SendString("BUILD v41 10-16  (LED: PA1=PWM呼吸+报警)\r\n");

	/* ③ PA1同一颗LED：TIM2_CH2 PWM承担启动呼吸和闹钟闪烁
	   LED_Init() 无等待；PA1高电平点亮，初始化时熄灭。 */
	LED_Init();
	LED_DumpRegs();
	Serial_SendString("LED init done: HIGH=ON, PA1 OFF\r\n");

	/* ④ 两个按键 + 旋转编码器（编码器采样跑在 TIM3 中断里） */
	Key_Init();
	Encoder_Init();

	/* ⑤ 舵机 PWM（TIM1_CH1 / PA8），上电默认 90° */
	Servo_Init();

	/* ⑥ OLED 与时钟
	   显示只走 OLED.h 里的基础显示函数（原版驱动，实测最稳），
	   不使用显存/像素填充接口 */
	OLED_Init();
	OLED_Clear();
	Clock_Init();
	LightSensor_Init(); /* PA0/ADC1后台采样，OLED渐变亮度。 */

	/* ⑦ OLED 开机动画开始（非阻塞：主循环里的 TASK_ID_BOOT 会继续推进它）。
	   IMU初始化等待和校准之间协作推进动画，总动画约3.2秒 */
	Menu_Init();
	Menu_BootTask();
	MPU6050_SetStartupHook(App_StartupPoll);

	/* ⑧ MPU6050：上电静止时完成零偏校准（校准约 0.4 秒） */
	if(MPU6050_Init() == MPU_OK)
	{
		Serial_printf("MPU6050 WHO_AM_I=0x%02X OK\r\n",(unsigned int)MPU6050_WhoAmI());
		MPU6050_Calibrate();                    /* 校准期间请让模块保持静止 */
		if(MPU6050_IsCalibrated()) Serial_SendString("IMU calibrated\r\n");
		else Serial_SendString("IMU calibration failed: keep still and reboot\r\n");
	}
	else
	{
		Serial_printf("MPU6050 not found, WHO_AM_I=0x%02X\r\n",(unsigned int)MPU6050_WhoAmI());
	}

	MPU6050_SetStartupHook(0); /* 后续IMU采样不再调用启动绘图。 */
	/* ⑨ 串口命令解析器 + 姿态定角默认关闭（上电默认 90° 手动控制不受影响） */
	Protocol_Init();
	Servo_SetHold(0);

	/* ⑩ 继续同一条动画时间线；不重置、不重复播放首帧。 */
	while(Menu_BootDone() == 0)
	{
		App_StartupPoll();
	}

	Menu_SetDirty();
	Menu_DrawHeader();
	Menu_DrawPage();

	Serial_SendString("Commands: HELP / TIME(hh:mm:ss) / ALARM1(07:30:00) / "
	                  "ALARM ? / IMU ? / SERVO ? / SERVO SET 120 / SERVO HOLD ON / LIGHT ?\r\n");
	Serial_SendString("Ready.\r\n");
}

/**
  * @brief  应用主循环：按 1ms 节拍分派各个周期任务，不会返回
  * @param  无
  * @retval 无
  */
void App_Loop(void)
{
	while(1)
	{
		LightSensor_Update(); /* 内部50ms节拍，无转换等待，不唤醒时钟待机页。 */

		/* ---- 每 10ms：串口命令（全局后台功能，任何界面都要能收） ---- */
		if(Task_IsDue(TASK_ID_PROTOCOL,10) == 1)
		{
			Protocol_Task();
		}

		/* ---- 每 10ms：按键消抖扫描 ---- */
		if(Task_IsDue(TASK_ID_KEY,10) == 1)
		{
			Key_Scan();
		}

		/* ---- 每 10ms：时钟走秒（内部按 1000ms 进位，含 23:59:59→00:00:00） ---- */
		if(Task_IsDue(TASK_ID_CLOCK,10) == 1)
		{
			Clock_Task();
		}

		/* ---- 每 10ms：闹钟判点 ---- */
		if(Task_IsDue(TASK_ID_ALARM,10) == 1)
		{
			Clock_AlarmTask();
		}

		/* ---- 每 10ms：界面逻辑与操作（开机动画期间 Menu_Task 自己会直接返回） ---- */
		if(Task_IsDue(TASK_ID_MENU,10) == 1)
		{
			App_ProcessMenuInput();
		}

		/* ---- 每 20ms：LED 变速呼吸（上电后跑 3 秒） ---- */
		if(Task_IsDue(TASK_ID_LED,20) == 1)
		{
			LED_BreathTask();
		}

		/* ---- 每 20ms：IMU 采样 + 互补滤波 ---- */
		if(Task_IsDue(TASK_ID_IMU,20) == 1)
		{
			MPU6050_Update();
		}

		/* ---- 每 20ms：舵机平滑逼近目标角度 ---- */
		if(Task_IsDue(TASK_ID_SERVO,20) == 1)
		{
			Servo_Update();
		}

		/* ---- 每 20ms：IMU 姿态定角闭环（未开启定角时内部直接返回） ---- */
		if(Task_IsDue(TASK_ID_HOLD,20) == 1)
		{
			Servo_HoldTask();
		}

		/* ---- 每 20ms：开机动画（跑满 MENU_BOOT_MS 后自动进主菜单） ---- */
		if(Task_IsDue(TASK_ID_BOOT,20) == 1)
		{
			Menu_BootTask();
		}

		/* ---- 每 50ms：串口 TRACE（SERVO TRACE ON 时才有输出） ---- */
		if(Task_IsDue(TASK_ID_TRACE,50) == 1)
		{
			Protocol_Update();
		}

		/* ---- 定时刷屏（只重画有变化的内容；开机动画期间不刷界面） ---- */
		if(Menu_BootDone() == 1)
		{
			if(Task_IsDue(TASK_ID_DISPLAY,Menu_GetRefreshMs()) == 1)
			{
				Menu_DrawHeader();
				Menu_DrawPage();
			}
		}
		else
		{
			/* 开机动画期间也要按节拍走一次"到期检查"，
			   否则动画结束后第一次刷屏会被动画占用的时间点顶掉 */
			(void)Task_IsDue(TASK_ID_DISPLAY,Menu_GetRefreshMs());
		}

		/* ---- 闹钟 LED 闪烁（1Hz，亮 500ms、灭 500ms）：每个循环都判断一次 ---- */
		LED_AlarmTask(Clock_IsRinging());
	}
}
