#include "stm32f10x.h"                  // Device header
#include "Timer.h"

/*----------------------------------------------------------------------------
 * 文件：Timer.c / Timer.h
 * 用处：用 TIM2 产生 1ms 中断，作为整个系统的节拍；主循环用 Task_IsDue()
 *       判断"某个周期任务是不是该执行了"
 * 对应外设：TIM2（72MHz / 72 = 1MHz，1000 个数 = 1ms）
 * 对应器件：无（全系统心跳）
 * 备注：1) 中断里【只做 TimerTick++】，所有业务都在主循环里执行
 *       2) 本工程不再用 SysTick 做延时（SysTick 归 Delay.c 的阻塞延时用），
 *          周期节拍一律走 TIM2，避免两者互相干扰
 *       3) TIM2 的时基在 LED.c 里被复用做 PWM（1kHz），请勿改动 PSC/ARR
 *----------------------------------------------------------------------------*/

#define PSC     72          //预分频值：72MHz / 72 = 1MHz，即 1us 计一个数
#define ARR     1000        //重装值：1000us = 1ms 中断一次

#define TASK_MAX    16      //最多支持 16 个周期任务（App.c 目前用到 0~12）

static volatile uint32_t TimerTick = 0;             //系统运行的毫秒数（中断里累加）
static uint32_t TaskLastTick[TASK_MAX] = {0};       //每个任务上次执行的时间点
static uint8_t  TaskUsed[TASK_MAX] = {0};           //该任务号是否已经用过

/**
  * @brief  定时器初始化，产生 1ms 中断作为系统节拍
  * @param  无
  * @retval 无
  */
void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	TIM_InternalClockConfig(TIM2);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=ARR-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler=PSC-1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStruct);
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);

	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef  NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;    //节拍优先级最高
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM2,ENABLE);
}

/**
  * @brief  TIM2 中断服务函数（每 1ms 进来一次）
  * @param  无
  * @retval 无
  */
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
		TimerTick++;
	}
}

/**
  * @brief  读取系统运行毫秒数
  * @param  无
  * @retval 上电到现在的毫秒数
  */
uint32_t Timer_GetTick(void)
{
	return TimerTick;
}

/**
  * @brief  判断某个任务是否到期（到期了才执行，实现周期任务调度）
  * @param  TaskID    任务编号，自己起，0 ~ TASK_MAX-1，不同任务用不同编号
  * @param  Period_ms 周期，单位毫秒（如 10 = 每 10ms 执行一次）
  * @retval 1=该执行了   0=还没到时间（第一次调用只登记时间，不立即执行）
  */
uint8_t Task_IsDue(uint8_t TaskID, uint32_t Period_ms)
{
	if(TaskID >= TASK_MAX)
	{
		return 0;
	}

	if(TaskUsed[TaskID] == 0)               //第一次调用：只记录时间，不立即执行
	{
		TaskUsed[TaskID] = 1;
		TaskLastTick[TaskID] = TimerTick;
		return 0;
	}

	if(TimerTick - TaskLastTick[TaskID] >= Period_ms)
	{
		TaskLastTick[TaskID] = TimerTick;
		return 1;
	}

	return 0;
}
