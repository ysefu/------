#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Menu.c / Menu.h
 * 用处：开机动画 + 6 个界面的图形化显示与操作
 *       （主菜单 / 时间设置 / 三路定时闹钟 / IMU 数据 / 舵机角度与定角 / 亮度设置）
 * 对应外设：OLED（软件 IIC，PB8/PB9）；数据来自 Clock、MPU6050、Servo 三个驱动
 * 对应器件：OLED、KEY1、KEY2、旋转编码器、MPU6050、SG90 舵机
 * 备注：1) 界面编号固定顺序：主菜单→时间设置→定时闹钟→IMU 数据→舵机角度→亮度设置→主菜单
 *          KEY1固定顺序切页；主菜单编码器循环滚动、KEY2确认进入；功能页KEY2原操作不变
 *       2) 每个界面有"绘制内容"和"刷新间隔"两个接口，App.c 按间隔调用，
 *          静态界面不重复刷屏（避免闪烁重影，也省下 IIC 时间）
 *       3) 顶部状态栏（第 1 行）在任何界面都持续显示实时时间 + 页面标识 +
 *          闹钟进行中的 [!] 提示
 *       4) 闹钟页KEY2依次选择编号/时/分/秒/开关，编码器调整，长按保存当前一路。
 *          切换编号前需保存；串口修改当前一路时页面同步刷新。
 *----------------------------------------------------------------------------*/

/* 6 个界面编号 */
#define MENU_PAGE_MAIN          0       /* 主菜单 */
#define MENU_PAGE_TIME          1       /* 时间设置 */
#define MENU_PAGE_ALARM         2       /* 定时闹钟 */
#define MENU_PAGE_IMU           3       /* IMU 数据查看 */
#define MENU_PAGE_SERVO         4       /* 舵机角度控制 */

#define MENU_PAGE_BRIGHTNESS     5       /* 自动/手动亮度 */

/* 界面数量 */
#define MENU_PAGE_NUM           6

/* 开机动画：总时长（毫秒），题目要求不少于 1 秒 */
#define MENU_BOOT_MS            3200

/* 无按键/编码器操作60秒进入时钟待机；改变此值即可调整时间。 */
#define MENU_IDLE_MS 60000u
void Menu_Wake(void);
uint8_t Menu_IsStandby(void);

/* 初始化：进入开机动画，动画跑完后自动到主菜单 */
void Menu_Init(void);

/* 开机动画任务：主循环每 20ms 调用一次；动画播放与非阻塞延时都在这里 */
void Menu_BootTask(void);

/* 开机动画是否已经结束 */
uint8_t Menu_BootDone(void);

/* 按固定顺序切下一页（KEY1短按）；返回界面编号 */
uint8_t Menu_NextPage(void);

/* 取当前界面编号 */
uint8_t Menu_GetPage(void);

/* 标记"需要重画"（串口设置值后调用，保证页面显示与串口同步） */
void Menu_SetDirty(void);

/* 绘制第 1 行状态栏：HH:MM:SS + 页面标识 + 闹钟提示 */
void Menu_DrawHeader(void);

/* 绘制当前界面的主体内容 */
void Menu_DrawPage(void);

/* 当前界面的刷新周期（毫秒）：动态界面短、静态界面长 */
uint16_t Menu_GetRefreshMs(void);

/* 操作任务：主循环每 10ms 调用一次，内部读取 KEY1/KEY2 事件和编码器格数 */
void Menu_Task(void);

/* 界面切换是否发生过（App.c 用来决定"必须立刻重画一次"） */
uint8_t Menu_NeedRedraw(void);

#endif
