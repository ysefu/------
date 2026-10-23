#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"
/*----------------------------------------------------------------------------
 * 文件：LED.c / LED.h
 * 用处：PA1上电三秒PWM变速呼吸，与闹钟1Hz闪烁共用。
 * 对应外设：PA1/TIM2_CH2；对应器件：一个外接LED。
 * 备注：正极经各自限流电阻接IO，负极接GND；高电平亮，低电平灭。
 *----------------------------------------------------------------------------*/
/* 功能：初始化PA1 PWM；无参数、无返回值，先调用Timer_Init。 */
void LED_Init(void);
/* 功能：设置PA1 PWM亮度；Channel=1，Percent=0~100，其他通道忽略；无返回值。 */
void LED_SetDuty(uint8_t Channel, uint8_t Percent);
/* 功能：USART1打印LED版本、极性和寄存器；无参数、无返回值。 */
void LED_DumpRegs(void);
/* 功能：推进PA1三秒呼吸；启动等待和主循环调用，无参数、无返回值。 */
void LED_BreathTask(void);
/* 功能：查询PA1呼吸状态；无参数，返回1已完成、0未完成。 */
uint8_t LED_BreathDone(void);
/* 功能：推进PA1的1Hz报警（优先于呼吸）；BlinkOn=1报警、0停止，无返回值。 */
void LED_AlarmTask(uint8_t BlinkOn);
/* 功能：PA1全亮；无参数、无返回值。 */
void LED1_ON(void);
/* 功能：PA1熄灭；无参数、无返回值。 */
void LED1_OFF(void);
#endif
