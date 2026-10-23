#include "stm32f10x.h"                  // Device header
#include "bsp_pin.h"
#include "Encoder.h"

/*----------------------------------------------------------------------------
 * 文件：Encoder.c / Encoder.h
 * 用处：读取旋转编码器的转动方向和格数（时间/闹钟/舵机角度的加减输入）
 * 对应外设：普通 GPIO（A/B 两相输入）+ TIM3（每 500us 定时采样）
 * 对应器件：旋钮式编码器模块（两相输出常标 CLK / DT，或 A / B）
 * 备注：1) 【为什么用定时器采样】主循环里有阻塞任务 —— 软件 IIC 读一次 MPU6050
 *          约 2~3ms、OLED 刷一屏约 10~15ms。若靠主循环轮询编码器，这些时间里
 *          两相的状态跳变会被漏掉，状态机判为非法跳变就不计数，表现就是
 *          "转很多圈或者不固定圈数才有反应"。TIM3 每 500us 采样一次，
 *          间隔固定，主循环再忙也不会漏
 *       2) 采样在中断里做，所以主循环里【不要】再调用 Encoder_Scan()
 *       3) 两相输出的公共端（有的标 C / GND / COM）必须和开发板共地
 *       4) 引脚来自 bsp_pin.h（A=PB14，B=PB15），本文件不写死引脚
 *       5) 题目只要求使用 A、B 两相旋转输出，不使用编码器的按压开关
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * 【灵敏度标定】机械编码器转一格产生多少个计数
 *   标准情况是 4（正交信号完整走一圈：00→10→11→01）
 *   如果实测"转一格要走两下才动一次"，就把这里改成 2
 *   标定方法：看串口输出里的 Enc= 累计计数，清零后慢慢转 10 格，
 *             看增加了多少，再除以 10 就是"每格计数"
 *----------------------------------------------------------------------------*/
#define ENC_COUNTS_PER_DETENT    4

#define ENC_SAMPLE_US            500        /* 每 500us 采样一次（2kHz） */

static int32_t  EncoderCount  = 0;      /* 累计计数（原始计数，未换算成格） */
static int32_t  EncoderLast   = 0;      /* 上次取走时的累计计数 */
static int32_t  EncoderRemain = 0;      /* 不足一格的零头，留着下次一起算 */
static uint16_t EncoderEdge   = 0;      /* 两相状态变化总次数（查线用） */

/**
  * @brief  读取 A/B 两相的原始状态
  * @param  无
  * @retval 0~3：bit1 = A 相电平，bit0 = B 相电平
  */
uint8_t Encoder_GetRawState(void)
{
	uint8_t State=0;

	if(GPIO_ReadInputDataBit(ENC_A_PORT,ENC_A_PIN) == SET)
	{
		State |= 0x02;
	}
	if(GPIO_ReadInputDataBit(ENC_B_PORT,ENC_B_PIN) == SET)
	{
		State |= 0x01;
	}

	return State;
}

/**
  * @brief  读取两相状态变化的总次数（调试用：转编码器时这个值必须往上涨）
  * @param  无
  * @retval 变化次数，0 表示从开机到现在两个脚的电平从来没变过 → 线没接上
  */
uint16_t Encoder_GetEdgeCount(void)
{
	return EncoderEdge;
}

/**
  * @brief  编码器扫描（由 TIM3 中断每 500us 调用一次，主循环里不要再调）
  * @param  无
  * @retval 无
  */
void Encoder_Scan(void)
{
	static uint8_t LastState=0;
	uint8_t NowState;
	uint8_t Change;

	NowState = Encoder_GetRawState();

	if(NowState != LastState)
	{
		EncoderEdge++;                                      /* 只要变了就记一笔（查线用） */

		Change = (uint8_t)((LastState << 2) | NowState);    /* 上一状态 + 当前状态 */

		/* 正转的 4 种合法跳变：00→10→11→01→00 */
		if(Change==0x02 || Change==0x0B || Change==0x0D || Change==0x04)
		{
			EncoderCount++;
		}
		/* 反转的 4 种合法跳变：00→01→11→10→00 */
		if(Change==0x01 || Change==0x07 || Change==0x0E || Change==0x08)
		{
			EncoderCount--;
		}
		/* 其它跳变（如 00→11）属于抖动或漏采样，直接丢弃，不计数 */

		LastState = NowState;
	}
}

/**
  * @brief  编码器初始化（配置两相引脚 + 启动 TIM3 定时采样）
  * @param  无
  * @retval 无
  */
void Encoder_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;            //上拉输入（编码器开漏输出时靠它拉高）
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;

	GPIO_InitStruct.GPIO_Pin=ENC_A_PIN;
	GPIO_Init(ENC_A_PORT,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin=ENC_B_PIN;
	GPIO_Init(ENC_B_PORT,&GPIO_InitStruct);

	/* ---- TIM3：72MHz / 72 = 1MHz，每 ENC_SAMPLE_US 个数中断一次 ---- */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	TIM_InternalClockConfig(TIM3);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_Period=ENC_SAMPLE_US-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler=72-1;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStruct);
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef  NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;    //比 1ms 节拍(TIM2)低一级即可
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStruct);

	TIM_Cmd(TIM3,ENABLE);
}

/**
  * @brief  TIM3 中断服务函数：每 500us 进来扫一次编码器
  * @param  无
  * @retval 无
  */
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);

		Encoder_Scan();                                 //采样间隔固定，不会漏掉跳变
	}
}

/**
  * @brief  返回自上次调用以来转过的格数，并把增量清零
  * @param  无
  * @retval 正数=顺时针转了 n 格   负数=逆时针转了 n 格   0=没转满一格
  * 备注：不足一格的零头会累加到下一次，不会被丢掉
  */
int8_t Encoder_Get(void)
{
	int32_t Temp;
	int32_t Detent;

	Temp = EncoderCount - EncoderLast;
	EncoderLast = EncoderCount;

	EncoderRemain += Temp;                              //零头留着，下次一起算

	Detent = EncoderRemain / ENC_COUNTS_PER_DETENT;
	EncoderRemain -= Detent * ENC_COUNTS_PER_DETENT;    //把已经报出去的扣掉

	if(Detent > 127)   Detent = 127;
	if(Detent < -128)  Detent = -128;

	return (int8_t)Detent;
}

/**
  * @brief  返回累计计数（原始计数，调试时用来看有没有丢步）
  * @param  无
  * @retval 累计计数值
  */
int32_t Encoder_GetCount(void)
{
	return EncoderCount;
}
