#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "bsp_pin.h"
#include "Serial.h"

/*----------------------------------------------------------------------------
 * 文件：Serial.c / Serial.h
 * 用处：USART1 非阻塞收发。发送用于回命令与调试，接收用于全局后台串口命令
 * 对应外设：USART1（PA9 = TX 接 USB-TTL 的 RXD，PA10 = RX 接 USB-TTL 的 TXD）
 * 对应器件：USB-TTL 串口模块（CH340 / CP2102）
 * 备注：1) 【为什么必须做成非阻塞】题目要求串口作为全局后台功能，无论当前在哪个
 *          界面都要能收命令且不得阻塞计时、按键、IMU 和舵机。若用"死等 TXE"的
 *          发送方式，115200 下发一行 70 字符要 6ms，HELP 全文要几十毫秒，
 *          主循环就被拖住了。所以这里把发送也改成"入队 + TXE 中断"
 *       2) 环形缓冲的下标只由本文件的读写方各自修改：
 *          TxHead 只在中断里 +1，TxTail 只在主循环里 +1；RxHead 只在中断里 +1，
 *          RxTail 只在主循环里 +1 —— 这样不用关中断也不会撕裂
 *       3) 缓冲区满时直接丢字符（绝不等待），丢弃数可用 Serial_GetStats() 查
 *       4) 中断优先级设为抢占 3，低于 TIM2(1) 和 TIM3(2)，
 *          串口中断不会打断 1ms 节拍与编码器采样
 *----------------------------------------------------------------------------*/

#define SERIAL_TX_BUF_SIZE      1024    /* 必须为 2 的幂 */
#define SERIAL_RX_BUF_SIZE      256     /* 必须为 2 的幂 */

static volatile uint8_t  TxBuf[SERIAL_TX_BUF_SIZE];
static volatile uint16_t TxHead = 0;    /* 中断里推进：下一个要发的字节 */
static volatile uint16_t TxTail = 0;    /* 主循环推进：下一个要入队的位置 */
static volatile uint16_t TxDrop = 0;    /* 因缓冲区满被丢掉的字节数 */

static volatile uint8_t  RxBuf[SERIAL_RX_BUF_SIZE];
static volatile uint16_t RxHead = 0;    /* 中断里推进 */
static volatile uint16_t RxTail = 0;    /* 主循环推进 */
static volatile uint16_t RxBad  = 0;    /* 格式错误/溢出次数 */

/**
  * @brief  串口初始化（115200-8-N-1，开启收发中断）
  * @param  无
  * @retval 无
  */
void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitTypeDef  GPIO_InitStruct;                  //PA9 复用推挽输出 = 发送
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin=SERIAL_TX_PIN;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(SERIAL_TX_PORT,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IPU;            //PA10 上拉输入 = 接收
	GPIO_InitStruct.GPIO_Pin=SERIAL_RX_PIN;
	GPIO_Init(SERIAL_RX_PORT,&GPIO_InitStruct);

	USART_InitTypeDef  USART_InitStruct;
	USART_InitStruct.USART_BaudRate=SERIAL_BAUDRATE;
	USART_InitStruct.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStruct.USART_Parity=USART_Parity_No;
	USART_InitStruct.USART_StopBits=USART_StopBits_1;
	USART_InitStruct.USART_WordLength=USART_WordLength_8b;
	USART_Init(SERIAL_USART,&USART_InitStruct);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef  NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=3;    //低于 TIM2/TIM3，不抢节拍
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStruct);

	/* 接收中断常开；发送中断等真有数据要发时再打开（见 UsartKick） */
	USART_ITConfig(SERIAL_USART,USART_IT_RXNE,ENABLE);
	USART_Cmd(SERIAL_USART,ENABLE);
}

/**
  * @brief  内部使用：如果发送缓冲里还有数据、且发送中断没开，就把它打开
  * @param  无
  * @retval 无
  */
static void UsartKick(void)
{
	if(TxHead != TxTail)
	{
		USART_ITConfig(SERIAL_USART,USART_IT_TXE,ENABLE);
	}
}

/**
  * @brief  内部使用：把一个字节放进发送缓冲（满了就丢，不等待）
  * @param  Byte 要发送的字节
  * @retval 无
  */
static void TxPush(uint8_t Byte)
{
	uint16_t Next;

	Next = (uint16_t)((TxHead + 1) & (SERIAL_TX_BUF_SIZE - 1));

	if(Next == TxTail)              //缓冲区满：丢弃，绝不阻塞
	{
		TxDrop++;
		return;
	}

	TxBuf[TxHead] = Byte;
	TxHead = Next;

	UsartKick();
}

/**
  * @brief  发送一个字节（只入队，不等待）
  * @param  Byte 要发送的字节
  * @retval 无
  */
void Serial_SendByte(uint8_t Byte)
{
	TxPush(Byte);
}

/**
  * @brief  发送一个字符串（以 '\0' 结尾）
  * @param  String 要发送的字符串
  * @retval 无
  */
void Serial_SendString(char *String)
{
	uint16_t i;
	for(i=0;String[i]!='\0';i++)
	{
		TxPush((uint8_t)String[i]);
	}
}

/**
  * @brief  发送一个数组
  * @param  Array  数组首地址
  * @param  Length 发送多少个字节
  * @retval 无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for(i=0;i<Length;i++)
	{
		TxPush(Array[i]);
	}
}

/**
  * @brief  发送一个数字（按固定位数发，不够的补前导 0）
  * @param  Number 要发送的数字
  * @param  Length 发送几位，如 Length=5、Number=23 会发出 "00023"
  * @retval 无
  */
void Serial_SendNum(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Temp;

	for(i=0;i<Length;i++)
	{
		Temp = 1;

		uint8_t j;
		for(j=0;j<Length-i-1;j++)
		{
			Temp *= 10;
		}

		TxPush((uint8_t)(Number / Temp % 10 + '0'));
	}
}

/**
  * @brief  printf 重定向钩子，让 C 标准库的 printf 也走串口
  * @param  ch 要输出的字符
  * @param  f  文件流（不用）
  * @retval 输出的字符
  */
int fputc(int ch, FILE *f)
{
	TxPush((uint8_t)ch);
	return ch;
}

/**
  * @brief  格式化发送（用法和 printf 一样，最常用的调试与应答函数）
  * @param  format 格式字符串，如 "OK TIME=%02d:%02d:%02d\r\n"
  * @param  ...    可变参数
  * @retval 无
  */
void Serial_printf(char *format, ...)
{
	char String[128];
	va_list arg;

	va_start(arg,format);
	vsnprintf(String,sizeof(String),format,arg);        //限制长度，防止缓冲区溢出
	va_end(arg);

	Serial_SendString(String);
}

/**
  * @brief  查询收到几个还没取走的字节
  * @param  无
  * @retval 可读字节数，0 表示没有新数据
  */
uint16_t Serial_Available(void)
{
	return (uint16_t)((RxHead - RxTail) & (SERIAL_RX_BUF_SIZE - 1));
}

/**
  * @brief  取走一个收到的字节
  * @param  无
  * @retval 收到的字节（调用前请先用 Serial_Available() 判断）
  */
uint8_t Serial_ReadByte(void)
{
	uint8_t Byte;

	if(RxHead == RxTail)
	{
		return 0;
	}

	Byte = RxBuf[RxTail];
	RxTail = (uint16_t)((RxTail + 1) & (SERIAL_RX_BUF_SIZE - 1));
	return Byte;
}

/**
  * @brief  查询发送缓冲是否已经全部发完
  * @param  无
  * @retval 1=发完了（可以安全改内容）  0=还在发
  */
uint8_t Serial_TxIdle(void)
{
	if(TxHead == TxTail)
	{
		return 1;
	}
	return 0;
}

/**
  * @brief  查询串口统计信息（排查丢数据用）
  * @param  Bad  输出：接收侧的格式错误/溢出次数
  * @param  Drop 输出：发送侧因缓冲满被丢弃的字节数
  * @retval 无
  */
void Serial_GetStats(uint16_t *Bad, uint16_t *Drop)
{
	if(Bad  != 0)  *Bad  = RxBad;
	if(Drop != 0)  *Drop = TxDrop;
}

/**
  * @brief  USART1 中断服务函数：收一个字节入队、发一个字节出队
  * @param  无
  * @retval 无
  */
void USART1_IRQHandler(void)
{
	/* ---- 接收 ---- */
	if(USART_GetITStatus(SERIAL_USART,USART_IT_RXNE) == SET)
	{
		uint8_t Byte = (uint8_t)USART_ReceiveData(SERIAL_USART);
		uint16_t Next = (uint16_t)((RxHead + 1) & (SERIAL_RX_BUF_SIZE - 1));

		if(Next != RxTail)              //没满才存，满了丢弃旧数据不动
		{
			RxBuf[RxHead] = Byte;
			RxHead = Next;
		}
		else
		{
			RxBad++;
		}
	}

	/* ---- 溢出/帧错误：必须"先读 SR 再读 DR"才能清掉 ORE，否则会卡死中断 ---- */
	if(USART_GetFlagStatus(SERIAL_USART,USART_FLAG_ORE) == SET)
	{
		(void)SERIAL_USART->SR;
		(void)SERIAL_USART->DR;
		RxBad++;
	}

	/* ---- 发送 ---- */
	if(USART_GetITStatus(SERIAL_USART,USART_IT_TXE) == SET)
	{
		if(TxHead != TxTail)
		{
			USART_SendData(SERIAL_USART,TxBuf[TxTail]);
			TxTail = (uint16_t)((TxTail + 1) & (SERIAL_TX_BUF_SIZE - 1));
		}
		else
		{
			USART_ITConfig(SERIAL_USART,USART_IT_TXE,DISABLE);  //没数据了，关掉发送中断
		}
	}
}
