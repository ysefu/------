#include "stm32f10x.h"                  // Device header
#include "bsp_pin.h"
#include "Timer.h"
#include "Serial.h"
#include "LED.h"

/*----------------------------------------------------------------------------
 * 文件：LED.c / LED.h
 * 用处：PA1上电三秒变速呼吸，之后复用为闹钟1Hz闪灯。
 * 对应外设：TIM2_CH2；TIM2时基由Timer.c初始化。
 * 对应器件：一颗外接LED，IO接正极、负极接GND，高电平点亮。
 * 备注：LED必须串限流电阻，PWM不能替代电阻；初始化无等待。
 *----------------------------------------------------------------------------*/
/* 项目效果参数，标准库没有呼吸时长、亮度或速度的对应配置。 */
#define LED_BREATH_STEP_MS 20u
#define LED_BREATH_TOTAL_MS 3000u
#define LED_BREATH_POINTS (LED_BREATH_TOTAL_MS / LED_BREATH_STEP_MS)
#define LED_BREATH_PEAK 80u

static uint8_t LedTable[LED_BREATH_POINTS];
static uint8_t LedStarted = 0;
static uint8_t LedFinished = 0;
static uint16_t LedIndex = LED_BREATH_POINTS;
static uint32_t LedStartTick = 0;
static uint8_t LedAlarmActive = 0;
static uint32_t LedAlarmStartTick = 0;

void LED_SetDuty(uint8_t Channel, uint8_t Percent)
{
	uint16_t Pulse;
	if(Channel != 1) return;
	if(Percent > 100) Percent = 100;
	Pulse = (uint16_t)((uint32_t)Percent * (TIM2->ARR + 1u) / 100u);
	TIM_SetCompare2(TIM2,Pulse);
}




void LED_Init(void)
{
	GPIO_InitTypeDef Gpio;
	TIM_OCInitTypeDef Oc;
	uint16_t i;
	float Phase = 0.0f;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,ENABLE);
	/* 输出锁存器预置低，再配置模式，避免初始化误亮。 */
	GPIO_ResetBits(LED1_PORT,LED1_PIN);
	Gpio.GPIO_Speed = GPIO_Speed_2MHz;
	/* 保持默认PA1映射，仅设置通道，不改TIM2的1ms时基。 */
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2,DISABLE);
	TIM_OCStructInit(&Oc);
	Oc.TIM_OCMode = TIM_OCMode_PWM1;
	Oc.TIM_OutputState = TIM_OutputState_Enable;
	Oc.TIM_OCPolarity = TIM_OCPolarity_High;
	Oc.TIM_Pulse = 0;
	TIM_OC2Init(TIM2,&Oc);
	TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);
	TIM_SetCompare2(TIM2,0);
	Gpio.GPIO_Mode = GPIO_Mode_AF_PP;
	Gpio.GPIO_Pin = LED1_PIN;
	GPIO_Init(LED1_PORT,&Gpio);
	/* 呼吸速度从0.6连续增加至3次/秒，平滑三角波实现亮灭循环。 */
	for(i=0;i<LED_BREATH_POINTS;i++)
	{
		float Speed = 0.6f + 2.4f * (float)i / (float)(LED_BREATH_POINTS-1u);
		float Inner;
		float Tri;
		Phase += Speed * 0.02f;
		Inner = Phase - (float)((uint16_t)Phase);
		Tri = Inner < 0.5f ? Inner * 2.0f : (1.0f-Inner) * 2.0f;
		LedTable[i] = (uint8_t)((float)LED_BREATH_PEAK * Tri * Tri * (3.0f-2.0f*Tri) + 0.5f);
	}
	LedTable[0] = 0;
	LedTable[LED_BREATH_POINTS-1u] = 0;
	LedStarted = 0;
	LedFinished = 0;
	LedIndex = LED_BREATH_POINTS;
	LedAlarmActive=0; LedAlarmStartTick=0;
}

void LED_DumpRegs(void)
{
	Serial_printf("LED v26 PA1 BREATH+ALARM HIGH=ON ARR=%u CCR2=%u CCER=%04X CRL=%08lX ODR=%04X\r\n",
		(unsigned int)TIM2->ARR,(unsigned int)TIM2->CCR2,
		(unsigned int)TIM2->CCER,(unsigned long)LED1_PORT->CRL,
		(unsigned int)LED1_PORT->ODR);
}

void LED_BreathTask(void)
{
	uint32_t Elapsed;
	uint16_t Index;
	if(LedFinished || LedAlarmActive) return;
	/* 首次调用才开始计时，初始化、校准不会吃掉三秒呼吸曲线。 */
	if(!LedStarted)
	{
		LedStarted = 1;
		LedStartTick = Timer_GetTick();
	}
	Elapsed = Timer_GetTick() - LedStartTick;
	if(Elapsed >= LED_BREATH_TOTAL_MS)
	{
		LED_SetDuty(1,0);
		LedFinished = 1;
		Serial_SendString("LED breath done: PA1 OFF\r\n");
		return;
	}
	Index = (uint16_t)(Elapsed / LED_BREATH_STEP_MS);
	if(Index == LedIndex) return;
	LedIndex = Index;
	LED_SetDuty(1,LedTable[Index]);
}

uint8_t LED_BreathDone(void)
{
	return LedFinished;
}

void LED_AlarmTask(uint8_t BlinkOn)
{
	if(!BlinkOn)
	{
		/* 非报警状态不写PWM，保留启动呼吸；仅在报警退出时熄灭。 */
		if(LedAlarmActive)
		{
			LedAlarmActive=0;
			LED_SetDuty(1,0);
		}
		return;
	}
	if(!LedAlarmActive)
	{
		LedAlarmActive=1;
		LedAlarmStartTick=Timer_GetTick();
		LedFinished=1; /* 报警优先，终止尚未完成的启动呼吸，消音后保持灭。 */
	}
	LED_SetDuty(1,((Timer_GetTick()-LedAlarmStartTick)/500u)%2u==0u ? 100 : 0);
}


void LED1_ON(void)
{
	LED_SetDuty(1,100);
}

void LED1_OFF(void)
{
	LED_SetDuty(1,0);
}
