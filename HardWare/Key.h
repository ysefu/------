#ifndef __KEY_H
#define __KEY_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Key.c / Key.h
 * 用处：读取两个独立按键，做消抖并识别"短按 / 长按"（页面切换、参数选择、保存都靠它）
 * 对应外设：普通 GPIO（KEY1 = PB5，KEY2 = PB11）
 * 对应器件：2 个独立按键（有且仅有两个，题目硬性要求）
 * 备注：1) KEY1(PB5) 另一端接 VCC → 下拉输入，空闲=低、按下=高
 *          KEY2(PB11) 另一端接 GND → 上拉输入，空闲=高、按下=低
 *       2) 极性由 bsp_pin.h 的 KEYx_ACTIVE_LEVEL 决定，本文件不写死
 *       3) 消抖规则：按下和松开都要连续稳定 20ms 才确认，
 *          这样松手时的抖动不会把一次按压误报成两次短按
 *       4) 长按只上报一次；短按在松手时立即上报（不会因为后面按久了就丢掉）
 *----------------------------------------------------------------------------*/

#define KEY_ID_1        1       /* KEY1：PB5， 另一端接 VCC，按下为高 */
#define KEY_ID_2        2       /* KEY2：PB11，另一端接 GND，按下为低 */

/* 按键事件：Key_GetEvent() 的返回值 */
#define KEY_EVT_NONE    0       /* 没有事件 */
#define KEY_EVT_SHORT   1       /* 短按（按下后 1 秒内松开） */
#define KEY_EVT_LONG    2       /* 长按（按住超过 1 秒，只上报一次） */

/* 初始化两个按键（KEY1 下拉、KEY2 上拉） */
void Key_Init(void);

/* 按键扫描：主循环里每 10ms 调用一次（20ms 消抖） */
void Key_Scan(void);

/* 取出一个按键事件（取走后自动清空，不会重复上报） */
uint8_t Key_GetEvent(uint8_t KeyID);

/* 查询按键当前是否按住（已消抖）：1=按住 */
uint8_t Key_GetPress(uint8_t KeyID);

/* 读取按键原始电平（查线用，不消抖）：1=高电平 */
uint8_t Key_GetRawLevel(uint8_t KeyID);

/* 查询两个键是否【同时按住】（已消抖）：1=同时按住。闹钟评分点"两个按键同时按下"用它 */
uint8_t Key_GetBothPress(void);

#endif
