#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

/*----------------------------------------------------------------------------
 * 文件：Serial.c / Serial.h
 * 用处：USART1 收发。发送用于回命令、回数据、调试打印；接收用于全局后台命令
 * 对应外设：USART1（PA9 = TX、PA10 = RX），115200-8-N-1
 * 对应器件：USB-TTL 串口模块（CH340），另一头插电脑
 * 备注：1) 【本模块的核心设计】收发全部走环形缓冲 + 中断：
 *          接收：RXNE 中断把字节塞进 RxBuf，主循环来取
 *          发送：Serial_SendXxx() 只是把字符塞进 TxBuf 就立刻返回，
 *                真正的发送由 TXE 中断完成 → 【绝不阻塞主循环】
 *          这样"任意界面都能收命令"才不会拖慢时钟、按键、IMU 和舵机 PWM
 *       2) 打印函数是 Serial_printf()，用法和 printf 一样
 *       3) 缓冲区满了会丢弃多余字符（不等待），并用 Serial_GetStats() 可查丢字节数
 *----------------------------------------------------------------------------*/

/* 初始化串口：115200-8-N-1，开接收中断与发送中断 */
void Serial_Init(void);

/* 发送一个字节（只入队，不等待，立即返回） */
void Serial_SendByte(uint8_t Byte);

/* 发送一个以 '\0' 结尾的字符串（只入队） */
void Serial_SendString(char *String);

/* 发送一段内存（只入队），Length 为字节数 */
void Serial_SendArray(uint8_t *Array, uint16_t Length);

/* 按固定位数发送无符号数，不足补前导 0（如 Length=5、23 → "00023"） */
void Serial_SendNum(uint32_t Number, uint8_t Length);

/* 格式化发送，用法同 printf（如 Serial_printf("T=%d\r\n",t)） */
void Serial_printf(char *format, ...);

/* 查询收到几个还没取走的字节：0=没有新数据 */
uint16_t Serial_Available(void);

/* 取走一个收到的字节（调用前先确认 Serial_Available() 不为 0） */
uint8_t Serial_ReadByte(void);

/* 发送缓冲是否已经全部发完：1=发完了，可以安全修改要打印的内容 */
uint8_t Serial_TxIdle(void);

/* 查询统计：Bad=格式错误次数，Drop=发送被丢弃的字节数 */
void Serial_GetStats(uint16_t *Bad, uint16_t *Drop);

#endif
