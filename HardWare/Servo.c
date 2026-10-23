#include "stm32f10x.h"                  // Device header
#include "bsp_pin.h"
#include "MPU6050.h"
#include "Menu.h"
#include "Servo.h"
#include "Timer.h"
#include <math.h>

/*----------------------------------------------------------------------------
 * 文件：Servo.c / Servo.h
 * 用处：TIM1_CH1(PA8) 输出 50Hz PWM，控制 SG90 舵机转到 0°~180°
 * 对应外设：TIM1（高级定时器）的通道 1
 * 对应器件：SG90 舵机
 * 备注：1) 【PWM 参数怎么来的】
 *          72MHz / 72 = 1MHz → 1 个计数 = 1us
 *          ARR = 20000-1 → 周期 20000us = 20ms = 50Hz（舵机的标准频率）
 *          CCR = 脉宽：500us → 0°，2500us → 180°，即每度 11.11us
 *       2) 【高级定时器必须多一步】TIM1 是高级定时器，光配通道还不够，
 *          必须调用 TIM_CtrlPWMOutputs(TIM1,ENABLE) 打开主输出，
 *          否则 PA8 一点波形都没有（这是很容易踩的坑）
 *       3) 【供电】舵机必须独立 5V 供电 + 与开发板共地，详见 Servo.h 的警告
 *       4) 【平滑】目标角度与实际输出角度分开：
 *          编码器可以飞快地转，但舵机按每 20ms 最多 2°（100°/s）逼近，
 *          既不会抖动也不会电流冲击
 *----------------------------------------------------------------------------*/

#define SERVO_PWM_ARR           20000   /* 20ms 周期 = 50Hz */
#define SERVO_PWM_PSC           72      /* 72MHz/72 = 1MHz，1 个数 = 1us */

#define SERVO_MIN_US            500     /* 0° 对应脉宽 */
#define SERVO_MAX_US            2500    /* 180° 对应脉宽 */

#define SERVO_DEFAULT_ANGLE     90      /* 上电默认角度（题目要求） */
#define SERVO_MAX_STEP10        20      /* 每 20ms 最多变化 2.0°（单位 0.1°） */

static uint8_t  ServoAngle    = SERVO_DEFAULT_ANGLE;        /* 目标角度 0~180 */
static uint16_t ServoOut10    = SERVO_DEFAULT_ANGLE * 10;   /* 实际输出角度，单位 0.1° */

/* 姿态外环：误差决定小幅调节速度，角速度提供阻尼，实际dt积分为舵机位置。
 * 以下均为项目控制/保护参数，SPL不提供机构相关配置。本机实测90->100度时Pitch从-9.0变为-18.0，采用Pitch反向。
 */
#ifndef HOLD_AXIS_ROLL
#define HOLD_AXIS_ROLL 0       /* 1:Roll，0:Pitch；不能用此算法做绝对Yaw保持 */
#endif
#ifndef HOLD_DIR_SIGN
#define HOLD_DIR_SIGN 255        /* 1:舵机角增加时姿态增加；255:相反 */
#endif
#define HOLD_KP_PER_SEC 1.5f
#define HOLD_RATE_DAMP 0.45f
#define HOLD_RATE_FILTER_SEC 0.08f
#define HOLD_MAX_SPEED_DPS 10.0f
#define HOLD_OUTPUT_MIN10 200.0f
#define HOLD_OUTPUT_MAX10 1600.0f
#define HOLD_TRAVEL10 600.0f   /* 开启位置附近最多正负60度，仍裁剪到20~160度 */
#define HOLD_ENTER_BAND10 5.0f
#define HOLD_EXIT_BAND10 10.0f
#define HOLD_MAX_ERROR10 600.0f /* 姿态偏差超过60度停止补偿 */

static uint8_t HoldOn=0,HoldFault=0,HoldSettled=1;
static int16_t HoldTarget10=0,HoldCur10=0,HoldErr10=0;
static float HoldCommand10=900.0f,HoldHome10=900.0f,HoldRate10=0.0f;
static float HoldBestError10=0.0f,HoldPeakError10=0.0f,HoldResponseCommand10=900.0f;
static uint32_t HoldLastTick=0,HoldLastSample=0,HoldProgressTick=0,HoldGrowthTick=0,HoldLimitTick=0;

/* 故障码仅内部使用：1数据无效/过期，2启用时仍运动或超范围，3调度中断，
 * 4偏差过大，5反馈方向/机构无响应，6达到安全行程。故障锁存直到明确关闭/重启。 */
/**
  * @brief  内部使用：按 0.1° 的角度值直接算出比较值并写入 TIM1
  * @param  Angle10 角度，单位 0.1°（如 900 = 90.0°）
  * @retval 无
  */
static void Servo_WriteAngle10(uint16_t Angle10)
{
	uint32_t Pulse;

	if(Angle10 > 1800)
	{
		Angle10 = 1800;
	}

	/* 脉宽(us) = 500 + 角度 × (2500-500)/1800 */
	Pulse = SERVO_MIN_US + (uint32_t)Angle10 * (SERVO_MAX_US - SERVO_MIN_US) / 1800;

	TIM_SetCompare1(SERVO_TIM,(uint16_t)Pulse);         /* PA8 = TIM1 通道 1 */
}

/**
  * @brief  舵机初始化：PA8 输出 50Hz PWM，上电默认转到 90°
  * @param  无
  * @retval 无
  */
void Servo_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef  TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

	/* PA8 = TIM1_CH1，复用推挽输出 */
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=SERVO_PIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SERVO_PORT,&GPIO_InitStruct);

	/* 时基：1MHz 计数，20000 个数 = 20ms = 50Hz */
	TIM_InternalClockConfig(SERVO_TIM);
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=SERVO_PWM_ARR-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler=SERVO_PWM_PSC-1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(SERVO_TIM,&TIM_TimeBaseInitStruct);

	/* 通道 1 配成 PWM1 模式（注意：TIM_OCInitTypeDef 里没有 TIM_Channel 成员，
	   通道号由调用哪个 TIM_OCxInit 决定） */
	TIM_OCStructInit(&TIM_OCInitStruct);
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;    //正常极性，脉宽即高电平时间
	TIM_OCInitStruct.TIM_Pulse=0;
	TIM_OC1Init(SERVO_TIM,&TIM_OCInitStruct);

	TIM_OC1PreloadConfig(SERVO_TIM,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(SERVO_TIM,ENABLE);

	/* ★高级定时器关键一步：打开主输出，否则 PA8 没有波形 */
	TIM_CtrlPWMOutputs(SERVO_TIM,ENABLE);

	ServoAngle = SERVO_DEFAULT_ANGLE;
	ServoOut10 = SERVO_DEFAULT_ANGLE * 10;
	Servo_WriteAngle10(ServoOut10);

	TIM_Cmd(SERVO_TIM,ENABLE);

	/* 定角开始时是【关闭】状态：这样上电默认的 90° 手动控制完全不受影响，
	   要演示姿态定角时用串口 "SERVO HOLD ON" 或舵机页长按 KEY2 打开。
	   （题目"静止时完成零偏校准"由 App.c 里的 MPU6050_Calibrate() 负责） */
	Servo_SetHold(0);
}

/**
  * @brief  设置目标角度（自动限制在 0~180）
  * @param  Angle 目标角度，0~180；超过范围会被夹到边界
  * @retval 实际生效的目标角度
  */
uint8_t Servo_SetAngle(uint8_t Angle)
{
	if(Angle > 180)
	{
		Angle = 180;
	}

	if(HoldOn || HoldFault) return ServoAngle;
	ServoAngle = Angle;

	return ServoAngle;
}

/**
  * @brief  读取当前目标角度
  * @param  无
  * @retval 0~180
  */
uint8_t Servo_GetAngle(void)
{
	return ServoAngle;
}

/**
  * @brief  读取实际输出角度（平滑过程中的中间值）
  * @param  无
  * @retval 0.1° 为单位的角度，如 900 表示 90.0°
  */
uint16_t Servo_GetOutputAngle10(void)
{
	return ServoOut10;
}

/**
  * @brief  平滑逼近任务：主循环每 20ms 调用一次，让实际角度逐步靠近目标角度
  * @param  无
  * @retval 无
  */
void Servo_Update(void)
{
	uint16_t Target10;

	if(HoldOn || HoldFault) return;
	Target10 = (uint16_t)ServoAngle * 10;

	if(ServoOut10 == Target10)
	{
		return;                                     //已经到位，什么都不用做
	}

	if(Target10 > ServoOut10)
	{
		if(Target10 - ServoOut10 > SERVO_MAX_STEP10)
		{
			ServoOut10 += SERVO_MAX_STEP10;
		}
		else
		{
			ServoOut10 = Target10;
		}
	}
	else
	{
		if(ServoOut10 - Target10 > SERVO_MAX_STEP10)
		{
			ServoOut10 -= SERVO_MAX_STEP10;
		}
		else
		{
			ServoOut10 = Target10;
		}
	}

	Servo_WriteAngle10(ServoOut10);
}


static float Servo_HoldDirection(void)
{
#if HOLD_DIR_SIGN == 255
	return -1.0f;
#else
	return 1.0f;
#endif
}
static int16_t Servo_ReadAxis10(void)
{
	int16_t X,Y,Z; MPU6050_GetAngle10(&X,&Y,&Z);
	return HOLD_AXIS_ROLL ? X : Y;
}
static int16_t Servo_ReadRate10(void)
{
	int16_t X,Y; MPU6050_GetTiltRate10(&X,&Y);
	return HOLD_AXIS_ROLL ? X : Y;
}
static void Servo_LatchHoldFault(uint8_t Reason)
{
	HoldFault=Reason;
	HoldCommand10=(float)ServoOut10;
	ServoAngle=(uint8_t)((ServoOut10+5u)/10u);
}
static void Servo_CaptureHold(void)
{
	HoldCur10=Servo_ReadAxis10(); HoldTarget10=HoldCur10; HoldErr10=0;
	HoldCommand10=(float)ServoOut10; HoldHome10=HoldCommand10;
	HoldRate10=(float)Servo_ReadRate10(); HoldSettled=1; HoldBestError10=0.0f;
	HoldPeakError10=0.0f; HoldResponseCommand10=HoldCommand10;
	HoldLastTick=Timer_GetTick(); HoldProgressTick=HoldLastTick;
	HoldLastSample=MPU6050_GetSampleCount(); HoldGrowthTick=0; HoldLimitTick=0;
}
void Servo_HoldStart(void)
{
	if(!MPU6050_HasFreshSample()) {Servo_LatchHoldFault(1);return;}
	if(fabsf((float)ServoAngle*10.0f-(float)ServoOut10)>5.0f ||
	   ServoOut10<HOLD_OUTPUT_MIN10 || ServoOut10>HOLD_OUTPUT_MAX10)
	{Servo_LatchHoldFault(2);return;}
	HoldFault=0; Servo_CaptureHold(); HoldOn=1;
}
void Servo_HoldCal(void)
{
	if(HoldFault || !MPU6050_HasFreshSample()) return;
	Servo_CaptureHold();
}
void Servo_SetHold(uint8_t Enable)
{
	if(Enable) Servo_HoldStart();
	else
	{
		if(HoldOn || HoldFault) ServoAngle=(uint8_t)((ServoOut10+5u)/10u);
		HoldOn=0; HoldFault=0; HoldSettled=1;
		HoldCommand10=(float)ServoOut10; HoldRate10=0.0f;
	}
}
uint8_t Servo_GetHold(void) {return HoldOn;}
uint8_t Servo_GetHoldFault(void) {return HoldFault!=0;}
uint8_t Servo_GetHoldFaultCode(void) {return HoldFault;}
void Servo_SetHoldAngle10(int16_t Angle10)
{
	if(Angle10>300) Angle10=300;
	if(Angle10< -300) Angle10=-300;
	HoldTarget10=Angle10;
	HoldSettled=0;
	HoldBestError10=fabsf((float)HoldTarget10-(float)Servo_ReadAxis10());
	HoldPeakError10=HoldBestError10; HoldResponseCommand10=HoldCommand10;
	HoldProgressTick=Timer_GetTick(); HoldGrowthTick=0; HoldLimitTick=0;
}
int16_t Servo_GetHoldAngle10(void) {return HoldTarget10;}
int16_t Servo_GetHoldCurAngle10(void) {return HoldCur10;}
int16_t Servo_GetHoldErr10(void) {return HoldErr10;}
void Servo_HoldTask(void)
{
	uint32_t Now,Elapsed,Sample; float Dt,Err,AbsErr,Rate,Velocity,Next,Low,High;
	uint16_t Output;
	if(!HoldOn || HoldFault) return;
	if(!MPU6050_HasFreshSample()) {Servo_LatchHoldFault(1);return;}
	Sample=MPU6050_GetSampleCount();
	if(Sample==HoldLastSample) return; /* 同一份数据不重复控制 */
	Now=Timer_GetTick(); Elapsed=Now-HoldLastTick;
	if(Elapsed==0) return;
	if(Elapsed>100u) {Servo_LatchHoldFault(3);return;}
	HoldLastTick=Now; HoldLastSample=Sample; Dt=(float)Elapsed*0.001f;
	HoldCur10=Servo_ReadAxis10(); Err=(float)HoldTarget10-(float)HoldCur10;
	HoldErr10=(int16_t)Err; AbsErr=fabsf(Err);
	if(AbsErr>HOLD_MAX_ERROR10) {Servo_LatchHoldFault(4);return;}
	Rate=(float)Servo_ReadRate10();
	HoldRate10+=(Dt/(HOLD_RATE_FILTER_SEC+Dt))*(Rate-HoldRate10);
	if(AbsErr<=HOLD_ENTER_BAND10 && fabsf(HoldRate10)<15.0f)
	{
		HoldSettled=1; HoldCommand10=(float)ServoOut10; return;
	}
	if(HoldSettled)
	{
		if(AbsErr<HOLD_EXIT_BAND10 && fabsf(HoldRate10)<30.0f) return;
		HoldSettled=0; HoldBestError10=AbsErr; HoldProgressTick=Now; HoldGrowthTick=0;
		HoldPeakError10=AbsErr; HoldResponseCommand10=HoldCommand10;
	}
	/* 新扰动后从本次修正起点计行程；偏差正在减小时不累计发散故障。 */
	if(AbsErr>HoldPeakError10) HoldPeakError10=AbsErr;
	if(AbsErr+2.0f<HoldPeakError10)
	{HoldPeakError10=AbsErr; HoldGrowthTick=0; HoldProgressTick=Now;}
	if(AbsErr+2.0f<HoldBestError10)
	{HoldBestError10=AbsErr; HoldProgressTick=Now; HoldGrowthTick=0;}
	if(AbsErr>HoldBestError10+50.0f && fabsf(HoldCommand10-HoldResponseCommand10)>20.0f)
	{
		if(!HoldGrowthTick) HoldGrowthTick=Now;
		if(Now-HoldGrowthTick>=1200u) {Servo_LatchHoldFault(5);return;}
	}
	else HoldGrowthTick=0;
	if(AbsErr>20.0f && Now-HoldProgressTick>2500u && fabsf(HoldCommand10-HoldResponseCommand10)>30.0f)
	{Servo_LatchHoldFault(5);return;}
	/* 单位0.1度/秒，实际dt积分。不是每周期把整项P修正直接叠到角度。 */
	Velocity=Servo_HoldDirection()*(HOLD_KP_PER_SEC*Err-HOLD_RATE_DAMP*HoldRate10);
	if(Velocity>HOLD_MAX_SPEED_DPS*10.0f) Velocity=HOLD_MAX_SPEED_DPS*10.0f;
	if(Velocity< -HOLD_MAX_SPEED_DPS*10.0f) Velocity=-HOLD_MAX_SPEED_DPS*10.0f;
	Low=HoldHome10-HOLD_TRAVEL10; High=HoldHome10+HOLD_TRAVEL10;
	if(Low<HOLD_OUTPUT_MIN10) Low=HOLD_OUTPUT_MIN10;
	if(High>HOLD_OUTPUT_MAX10) High=HOLD_OUTPUT_MAX10;
	Next=HoldCommand10+Velocity*Dt;
	if(Next<Low || Next>High)
	{
		if(!HoldLimitTick) HoldLimitTick=Now;
		if(AbsErr>20.0f && Now-HoldLimitTick>=1000u) {Servo_LatchHoldFault(6);return;}
		if(Next<Low) Next=Low;
		if(Next>High) Next=High;
	}
	else HoldLimitTick=0;
	HoldCommand10=Next;
	Output=(uint16_t)(Next+0.5f);
	/* PWM不追逐小于0.2度的细碎变化，累积到阈值再写。 */
	if(fabsf((float)Output-(float)ServoOut10)>=2.0f)
	{
		ServoOut10=Output;
		ServoAngle=(uint8_t)((Output+5u)/10u);
		Servo_WriteAngle10(Output);
	}
}
