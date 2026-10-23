#ifndef __OLED_CHINESE_H
#define __OLED_CHINESE_H
#include "stm32f10x.h"
/*----------------------------------------------------------------------------
 * 文件：OLED_Chinese.c / OLED_Chinese.h
 * 用处：OLED中文词库、16x16字模与中英文混排，复用原OLED写屏时序。
 * 对应外设：PB8/PB9软件IIC；对应器件：128x64 OLED。
 * 备注：汉字占2个8像素字符格。字模索引独立于Keil源文件编码，不发送UTF-8字节。
 *----------------------------------------------------------------------------*/
typedef enum {
	CN_MAIN,
	CN_TIME,
	CN_ALARM,
	CN_IMU,
	CN_SERVO,
	CN_TIME_SET,
	CN_ALARM_SET,
	CN_IMU_DATA,
	CN_SERVO_ANGLE,
	CN_NEXT,
	CN_EDIT,
	CN_HOUR,
	CN_MINUTE,
	CN_SECOND,
	CN_SAVE,
	CN_CURRENT,
	CN_TARGET,
	CN_SWITCH,
	CN_ON,
	CN_OFF,
	CN_RING,
	CN_IMU_FAULT,
	CN_CHECK,
	CN_ANGLE,
	CN_HOLD,
	CN_FAULT,
	CN_CLEAR,
	CN_MANUAL,
	CN_SHORT_OFF,
	CN_SHORT_ON,
	CN_LONG_ON,
	CN_KEEP,
	CN_TITLE,
	CN_INIT,
	CN_OLED_READY,
	CN_IMU_READY,
	CN_LOADING,
 CN_APPLY, /* K2保存 */
 CN_LONG_OFF, /* 长按关 */
 CN_BRIGHT,
 CN_BRIGHT_SET,
 CN_AUTO,
 CN_CHANGE,
 CN_LIGHT,
 CN_SET_VALUE,
 CN_TEXT_COUNT
}OLED_ChineseText;
/* 功能：将词库文字写入16格行缓冲；Column从0开始，返回结束列，越界拒绝。 */
uint8_t OLED_Chinese_PutText(char *Buf, uint8_t Column, OLED_ChineseText Text);
/* 功能：填入度符号，占1格；Column从0开始，无返回值。 */
void OLED_Chinese_PutDegree(char *Buf, uint8_t Column);
/* 功能：按字符掩码写OLED；Line=1~4，Buf含16格，Inverse为反白格掩码。 */
void OLED_Chinese_DrawCells(uint8_t Line, const char *Buf, uint16_t Mask, uint16_t Inverse);
/* 功能：清空OLED指定行；Line=1~4，无返回值。 */
void OLED_Chinese_ClearLine(uint8_t Line);
/* 功能：直接显示词库文字；Line=1~4，Column=1~16，无返回值。 */
void OLED_Chinese_ShowText(uint8_t Line, uint8_t Column, OLED_ChineseText Text);
#endif
