#include "stm32f10x.h"                  // Device header
#include "bsp_pin.h"
#include "Key.h"

/*----------------------------------------------------------------------------
 * 文件：Key.c / Key.h
 * 用处：读取两个独立按键，做消抖并识别短按/长按
 * 对应外设：普通 GPIO（KEY1 = PB5，KEY2 = PB11）
 * 对应器件：2 个独立按键
 * 备注：1) KEY1(PB5) 另一端接 VCC → 必须配下拉输入（GPIO_Mode_IPD），空闲=低、按下=高
 *          KEY2(PB11) 另一端接 GND → 必须配上拉输入（GPIO_Mode_IPU），空闲=高、按下=低
 *          极性写在 bsp_pin.h 里，本文件不写死，换接法只改头文件
 *       2) 不用中断、不阻塞：主循环每 10ms 调一次 Key_Scan()
 *       3) 消抖做的是"按下和松开都要连续稳定 20ms（2 次扫描）才确认"，
 *          避免松手抖动把一次按压误报成两次短按
 *       4) 长按只在按满 1 秒时上报一次；短按在松手时立即上报
 *----------------------------------------------------------------------------*/

#define KEY_COUNT       2       //按键个数
#define KEY_DEBOUNCE    2       //连续读到几次新状态才确认（Key_Scan 每 10ms 一次 → 20ms 消抖）
#define KEY_LONG_SCANS  100     //按住多少次算长按（100 次 × 10ms = 1 秒）

static uint8_t  KeyStable[KEY_COUNT] = {0,0};   //消抖后的稳定状态：1=按住
static uint8_t  KeyCount[KEY_COUNT]  = {0,0};   //连续读到"和稳定状态不同"的次数
static uint8_t  KeySent[KEY_COUNT]   = {0,0};   //本次按下是否已经上报过长按
static uint16_t KeyHold[KEY_COUNT]   = {0,0};   //本次按住持续了多少次扫描
static uint8_t  KeyEvent[KEY_COUNT]  = {0,0};   //待取走的事件

/**
  * @brief  按键初始化（KEY1 下拉、KEY2 上拉，极性来自 bsp_pin.h）
  * @param  无
  * @retval 无
  */
void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	GPIO_InitTypeDef  GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;

	GPIO_InitStruct.GPIO_Mode=KEY1_MODE;
	GPIO_InitStruct.GPIO_Pin=KEY1_PIN;
	GPIO_Init(KEY1_PORT,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode=KEY2_MODE;
	GPIO_InitStruct.GPIO_Pin=KEY2_PIN;
	GPIO_Init(KEY2_PORT,&GPIO_InitStruct);
}

/**
  * @brief  读取按键原始电平（查线用：不消抖，直接反映引脚当前电平）
  * @param  KeyID KEY_ID_1 或 KEY_ID_2
  * @retval 1=高电平  0=低电平
  */
uint8_t Key_GetRawLevel(uint8_t KeyID)
{
	if(KeyID == KEY_ID_1)
	{
		return GPIO_ReadInputDataBit(KEY1_PORT,KEY1_PIN);
	}
	else
	{
		return GPIO_ReadInputDataBit(KEY2_PORT,KEY2_PIN);
	}
}

/**
  * @brief  查询按键当前是否按住（已经过消抖）
  * @param  KeyID KEY_ID_1 或 KEY_ID_2
  * @retval 1=按住  0=松开
  */
uint8_t Key_GetPress(uint8_t KeyID)
{
	if(KeyID == KEY_ID_1)
	{
		return KeyStable[0];
	}
	else if(KeyID == KEY_ID_2)
	{
		return KeyStable[1];
	}
	return 0;
}

/**
  * @brief  查询两个键是否同时按住（已消抖）—— 闹钟评分点"两个按键同时按下"用它
  * @param  无
  * @retval 1=两个键都按住
  */
uint8_t Key_GetBothPress(void)
{
	if(KeyStable[0] == 1 && KeyStable[1] == 1)
	{
		return 1;
	}
	return 0;
}

/**
  * @brief  取出一个按键事件（取走后自动清空，不会重复上报）
  * @param  KeyID KEY_ID_1 或 KEY_ID_2
  * @retval KEY_EVT_NONE / KEY_EVT_SHORT / KEY_EVT_LONG
  */
uint8_t Key_GetEvent(uint8_t KeyID)
{
	uint8_t i;
	uint8_t Evt;

	if(KeyID == KEY_ID_1)
	{
		i = 0;
	}
	else if(KeyID == KEY_ID_2)
	{
		i = 1;
	}
	else
	{
		return KEY_EVT_NONE;
	}

	Evt = KeyEvent[i];
	KeyEvent[i] = KEY_EVT_NONE;
	return Evt;
}

/**
  * @brief  按键扫描：主循环里每 10ms 调用一次（不用中断、不阻塞）
  * @param  无
  * @retval 无
  */
void Key_Scan(void)
{
	uint8_t i;
	uint8_t Level;
	uint8_t Active[KEY_COUNT] = {KEY1_ACTIVE_LEVEL,KEY2_ACTIVE_LEVEL};

	for(i=0;i<KEY_COUNT;i++)
	{
		/* 把原始电平换算成"1=按下"，与按键接 VCC 还是 GND 无关 */
		if(Key_GetRawLevel(i+1) == Active[i])
		{
			Level = 1;
		}
		else
		{
			Level = 0;
		}

		if(Level == KeyStable[i])
		{
			KeyCount[i] = 0;                    //状态和稳定值一致，计数清零
		}
		else
		{
			if(KeyCount[i] < 255)
			{
				KeyCount[i]++;
			}

			if(KeyCount[i] >= KEY_DEBOUNCE)     //连续 20ms 都是新状态 → 确认状态改变
			{
				KeyStable[i] = Level;
				KeyCount[i] = 0;

				if(Level == 1)                  //刚刚稳定按下
				{
					KeyHold[i] = 0;
					KeySent[i] = 0;
				}
				else                            //刚刚稳定松开
				{
					if(KeyHold[i] < KEY_LONG_SCANS)     //短按：没按满 1 秒就松开了
					{
						KeyEvent[i] = KEY_EVT_SHORT;
					}
				}
			}
		}

		/* 按住计时：只有稳定按住时才累加 */
		if(KeyStable[i] == 1)
		{
			if(KeyHold[i] < 0xFFFF)
			{
				KeyHold[i]++;
			}

			if(KeyHold[i] == KEY_LONG_SCANS && KeySent[i] == 0)     //按满 1 秒 → 长按
			{
				KeyEvent[i] = KEY_EVT_LONG;
				KeySent[i] = 1;
			}
		}
	}
}
