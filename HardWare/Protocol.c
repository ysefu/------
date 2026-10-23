#include "stm32f10x.h"                  // Device header
#include <string.h>
#include <stdlib.h>
#include "Serial.h"
#include "Clock.h"
#include "MPU6050.h"
#include "Servo.h"
#include "Menu.h"
#include "LightSensor.h"
#include "Protocol.h"
#include "Timer.h"

/*----------------------------------------------------------------------------
 * 文件：Protocol.c / Protocol.h
 * 用处：全局后台串口命令解析与应答
 * 对应外设：USART1
 * 对应器件：USB-TTL + 电脑串口助手
 * 备注：1) 【为什么放在主循环而不是中断里】命令要读写时钟、闹钟、舵机、
 *          还要刷 OLED，这些都不能在中断里做；主循环每 10ms 处理一次，
 *          响应时间远小于人眼可辨的延迟
 *       2) 【绝不阻塞】每次调用最多吃 48 个字节、最多执行 1 条命令就返回；
 *          发送也只是往串口环形缓冲里塞数据（见 Serial.c），所以
 *          "任意页面都能收命令"不会拖慢时钟、按键、IMU 和舵机 PWM
 *       3) 命令大小写不敏感；参数非法时回显 ERR，不影响后续命令
 *       4) 命令表（与题目要求逐字一致）：
 *          HELP                         多行帮助
 *          TIME(hh:mm:ss)               OK TIME=hh:mm:ss
 *          ALARMn(hh:mm:ss)             OK ALARMn=hh:mm:ss ON
 *          ALARMn OFF                   OK ALARMn=hh:mm:ss OFF
 *          ALARMn DEL                   OK ALARMn=00:00:00 OFF
 *          ALARM ?                      ALARM1=... / ALARM2=... / ALARM3=...
 *          IMU ?                        X=+1.2,Y=-3.4,Z=+0.5（0.1°，与 OLED 同源）
 *          SERVO ?                      SERVO=90 HOLD=OFF
 *          SERVO SET <angle>            OK SERVO=120 HOLD=OFF
 *          SERVO HOLD ON/OFF/?          OK SERVO HOLD=ON / ...FAULT=OFF
 *          SERVO TRACE ON/OFF/?         表头 + OK SERVO TRACE=ON，随后 20Hz 输出 H,...
 *----------------------------------------------------------------------------*/

#define PROTO_LINE_MAX      64              /* 单条命令最长 63 字符 */
#define PROTO_CHUNK_MAX     48              /* 每次任务最多处理的字节数 */

static char    LineBuf[PROTO_LINE_MAX];
static uint8_t LineLen   = 0;
static uint8_t LineOver  = 0;               /* 1=本行太长，丢弃 */

static uint8_t TraceOn   = 0;               /* SERVO TRACE 开关 */
/* 定角（HOLD）的状态由 Servo 模块统一维护，本模块只做命令解析与转发，
   避免两处各存一份状态导致显示不一致 */

/* Protocol_Update() 每 50ms 被调用一次，题目要求 20Hz（50ms 一行），
   所以输出分频取 1：每次调用都发一行 */
#define PROTO_TRACE_DIV     1

/**
  * @brief  初始化命令解析器（清空行缓冲与状态）
  * @param  无
  * @retval 无
  */
void Protocol_Init(void)
{
	LineLen   = 0;
	LineOver  = 0;
	TraceOn   = 0;
}

/**
  * @brief  内部使用：把字符转成大写
  * @param  Ch 输入字符
  * @retval 大写后的字符
  */
static char Proto_Upper(char Ch)
{
	if(Ch >= 'a' && Ch <= 'z')
	{
		return (char)(Ch - 'a' + 'A');
	}
	return Ch;
}

/**
  * @brief  内部使用：判断字符串是不是某个全大写关键字（大小写不敏感）
  * @param  Str  输入字符串
  * @param  Word 全大写关键字
  * @retval 1=相同   0=不同
  */
static uint8_t Proto_IsWord(char *Str, char *Word)
{
	uint8_t i;

	for(i=0;;i++)
	{
		if(Proto_Upper(Str[i]) != Word[i])      //只要有一个字符不同（含长度不同）就不相等
		{
			return 0;
		}

		if(Str[i] == '\0')                      //两边同时走完 → 完全相等
		{
			return 1;
		}
	}
}

/**
  * @brief  内部使用：从 "HH:MM:SS" 里取出时/分/秒
  * @param  Str    形如 "14:30:05" 的字符串
  * @param  Hour   输出：时
  * @param  Minute 输出：分
  * @param  Second 输出：秒
  * @retval 1=格式正确   0=格式错误
  */
static uint8_t Proto_ParseHms(char *Str, uint8_t *Hour, uint8_t *Minute, uint8_t *Second)
{
	uint8_t Len = 0;

	while(Str[Len] != '\0')
	{
		Len++;
	}

	/* 必须是 8 个字符且第 3、6 位是冒号，中间只能是数字 */
	if(Len != 8)
	{
		return 0;
	}
	if(Str[2] != ':' || Str[5] != ':')
	{
		return 0;
	}

	uint8_t i;
	for(i=0;i<8;i++)
	{
		if(i == 2 || i == 5)
		{
			continue;
		}
		if(Str[i] < '0' || Str[i] > '9')
		{
			return 0;
		}
	}

	*Hour   = (uint8_t)((Str[0]-'0')*10 + (Str[1]-'0'));
	*Minute = (uint8_t)((Str[3]-'0')*10 + (Str[4]-'0'));
	*Second = (uint8_t)((Str[6]-'0')*10 + (Str[7]-'0'));

	if(*Hour > 23 || *Minute > 59 || *Second > 59)
	{
		return 0;
	}

	return 1;
}

/**
  * @brief  内部使用：把 0~9 的数字转成字符（用于拼 "1"~"3"）
  * @param  Num 数字
  * @retval 字符
  */
static char Proto_DigitChar(uint8_t Num)
{
	return (char)('0' + Num);
}

/**
  * @brief  内部使用：输出 HELP 多行帮助文本
  * @param  无
  * @retval 无
  */
static void Proto_Help(void)
{
	Serial_SendString("Command list:\r\n");
	Serial_SendString("HELP\r\n");
	Serial_SendString("LIGHT ? / LIGHT DIR LOW / LIGHT DIR HIGH\r\n");
	Serial_SendString("LIGHT AUTO ON / OFF / LIGHT SET <8-255>\r\n");
	Serial_SendString("TIME(hh:mm:ss)\r\n");
	Serial_SendString("ALARMn(hh:mm:ss) / ALARMn OFF / ALARMn DEL\r\n");
	Serial_SendString("ALARM ?\r\n");
	Serial_SendString("IMU ?\r\n");
	Serial_SendString("SERVO ?\r\n");
	Serial_SendString("SERVO SET <0-180>\r\n");
	Serial_SendString("SERVO HOLD ON / OFF / CAL / ?\r\n");
	Serial_SendString("SERVO TRACE ON / OFF / ?\r\n");
	Serial_SendString("OK\r\n");
}

/**
  * @brief  内部使用：应答 SERVO ? / SERVO SET / SERVO HOLD 的当前状态
  * @param  Prefix 前缀，如 "OK " 或空串
  * @retval 无
  * 备注：定角打开时把目标姿态角也带出来，方便现场核对
  */
static void Proto_ServoStatus(char *Prefix)
{
	int16_t Target10;
	int16_t Abs;

	if(Servo_GetHold() == 1 || Servo_GetHoldFault() == 1)
	{
		Target10 = Servo_GetHoldAngle10();
		Abs = (Target10 < 0) ? (int16_t)(-Target10) : Target10;

		Serial_printf("%sSERVO=%d HOLD=%s TARGET=%s%d.%d FAULT=%s\r\n",
		              Prefix,
		              (int)Servo_GetAngle(),
		              Servo_GetHold() ? "ON" : "OFF",
		              (Target10 < 0) ? "-" : "+",
		              (int)(Abs/10),
		              (int)(Abs%10),
		              Servo_GetHoldFault() ? "ON" : "OFF");
	}
	else
	{
		Serial_printf("%sSERVO=%d HOLD=OFF\r\n",
		              Prefix,
		              (int)Servo_GetAngle());
	}
}

/**
  * @brief  内部使用：解析并执行一条命令（Line 已经转成大写再比较，但原文保留）
  * @param  Line 命令行（不含换行符）
  * @retval 无
  */
static void Proto_LightStatus(void)
{
 Serial_printf("LIGHT RAW=%u FILTER=%u CONTRAST=%u READY=%u BRIGHT=%s AUTO=%s MANUAL=%u\r\n",
  (unsigned int)LightSensor_GetRaw(),(unsigned int)LightSensor_GetFiltered(),
  (unsigned int)LightSensor_GetContrast(),(unsigned int)LightSensor_IsReady(),
  LightSensor_GetDirection() ? "HIGH" : "LOW",LightSensor_GetAuto() ? "ON" : "OFF",(unsigned int)LightSensor_GetManual());
}

static void Proto_Exec(char *Line)
{
	uint8_t Hour,Minute,Second;
	uint8_t Index;
	long    Value;
	char   *Arg;

 /* AO极性不同可现场切换；掉电恢复默认LOW，不改动已有命令及TRACE字段。 */
 if(Proto_IsWord(Line,"LIGHT ?")) {Proto_LightStatus();return;}
 if(Proto_IsWord(Line,"LIGHT DIR LOW"))
 {LightSensor_SetDirection(0);Serial_SendString("OK LIGHT DIR=LOW\r\n");Proto_LightStatus();return;}
 if(Proto_IsWord(Line,"LIGHT DIR HIGH"))
 {LightSensor_SetDirection(1);Serial_SendString("OK LIGHT DIR=HIGH\r\n");Proto_LightStatus();return;}

 if(Proto_IsWord(Line,"LIGHT AUTO ON"))
 {LightSensor_SetAuto(1);Menu_SetDirty();Serial_SendString("OK LIGHT AUTO=ON\r\n");Proto_LightStatus();return;}
 if(Proto_IsWord(Line,"LIGHT AUTO OFF"))
 {LightSensor_SetAuto(0);Menu_SetDirty();Serial_SendString("OK LIGHT AUTO=OFF\r\n");Proto_LightStatus();return;}
 if(strlen(Line)>10u)
 {
  static const char Prefix[]="LIGHT SET ";uint8_t Match=1,i;
  for(i=0;i<10u;i++) if(Proto_Upper(Line[i])!=Prefix[i]) Match=0;
  if(Match)
  {
   Arg=Line+10;
   if(*Arg<'0' || *Arg>'9') {Serial_SendString("ERR LIGHT SET RANGE=8-255\r\n");return;}
   Value=strtol(Arg,&Arg,10);
   if(*Arg!='\0' || Value<8 || Value>255) {Serial_SendString("ERR LIGHT SET RANGE=8-255\r\n");return;}
   LightSensor_SetManual((uint8_t)Value);Menu_SetDirty();
   Serial_printf("OK LIGHT SET=%u AUTO=OFF\r\n",(unsigned int)Value);return;
  }
 }

	/* ---------------- HELP ---------------- */
	if(Proto_IsWord(Line,"HELP"))
	{
		Proto_Help();
		return;
	}

	/* ---------------- TIME(hh:mm:ss) ---------------- */
	if(strncmp(Line,"TIME",4) == 0 || strncmp(Line,"time",4) == 0)
	{
		if(Line[4] != '(')
		{
			Serial_SendString("ERR\r\n");
			return;
		}

		Arg = Line + 5;
		/* 去掉结尾的 ')' */
		{
			uint8_t Len = 0;
			while(Arg[Len] != '\0')  Len++;
			if(Len > 0 && Arg[Len-1] == ')')
			{
				Arg[Len-1] = '\0';
			}
			else
			{
				Serial_SendString("ERR\r\n");
				return;
			}
		}

		if(Proto_ParseHms(Arg,&Hour,&Minute,&Second) == 0)
		{
			Serial_SendString("ERR\r\n");
			return;
		}

		Clock_SetTime(Hour,Minute,Second);
		Menu_SetDirty();                        /* 让页面显示与串口设置结果同步 */
		Serial_printf("OK TIME=%02d:%02d:%02d\r\n",(int)Hour,(int)Minute,(int)Second);
		return;
	}

	/* ---------------- ALARM 系列 ---------------- */
	if(strncmp(Line,"ALARM",5) == 0 || strncmp(Line,"alarm",5) == 0)
	{
		/* ALARM ? ：查询三个闹钟 */
		if(Line[5] == ' ' && Line[6] == '?')
		{
			for(Index=1;Index<=CLOCK_ALARM_NUM;Index++)
			{
				ClockTime_t T;
				if(Clock_GetAlarm(Index,&T) == SUCCESS)
				{
					Serial_printf("ALARM%d=%02d:%02d:%02d %s\r\n",
					              (int)Index,
					              (int)T.Hour,(int)T.Minute,(int)T.Second,
					              (Clock_GetAlarmEnable(Index) == ALARM_ON) ? "ON" : "OFF");
				}
			}
			return;
		}

		/* ALARMn...：n 必须是 1~3 */
		if(Line[5] < '1' || Line[5] > (char)('0' + CLOCK_ALARM_NUM))
		{
			Serial_SendString("ERR\r\n");
			return;
		}
		Index = (uint8_t)(Line[5] - '0');

		/* ALARMn(hh:mm:ss) ：设置并立即开启 */
		if(Line[6] == '(')
		{
			uint8_t Len = 0;

			Arg = Line + 7;
			while(Arg[Len] != '\0')  Len++;
			if(Len > 0 && Arg[Len-1] == ')')
			{
				Arg[Len-1] = '\0';
			}
			else
			{
				Serial_SendString("ERR\r\n");
				return;
			}

			if(Proto_ParseHms(Arg,&Hour,&Minute,&Second) == 0)
			{
				Serial_SendString("ERR\r\n");
				return;
			}

			Clock_SetAlarm(Index,Hour,Minute,Second);
			Menu_SetDirty();
			Serial_printf("OK ALARM%d=%02d:%02d:%02d ON\r\n",
			              (int)Index,(int)Hour,(int)Minute,(int)Second);
			return;
		}

		/* ALARMn OFF / ALARMn DEL */
		if(Line[6] == ' ')
		{
			ClockTime_t T;

			Arg = Line + 7;

			if(Proto_IsWord(Arg,"OFF"))
			{
				Clock_SetAlarmEnable(Index,ALARM_OFF);
				Clock_GetAlarm(Index,&T);
				Menu_SetDirty();
				Serial_printf("OK ALARM%d=%02d:%02d:%02d OFF\r\n",
				              (int)Index,(int)T.Hour,(int)T.Minute,(int)T.Second);
				return;
			}

			if(Proto_IsWord(Arg,"DEL"))
			{
				Clock_DelAlarm(Index);
				Menu_SetDirty();
				Serial_printf("OK ALARM%d=00:00:00 OFF\r\n",(int)Index);
				return;
			}
		}

		Serial_SendString("ERR\r\n");
		return;
	}

	/* ---------------- IMU ? ---------------- */
	if(strncmp(Line,"IMU",3) == 0 || strncmp(Line,"imu",3) == 0)
	{
		int16_t X,Y,Z;

		if(Line[3] != ' ' || Line[4] != '?')
		{
			Serial_SendString("ERR\r\n");
			return;
		}

		if(MPU6050_IsReady() == 0)
		{
			Serial_SendString("ERR IMU\r\n");
			return;
		}

		MPU6050_GetAngle10(&X,&Y,&Z);
		Serial_printf("X=%s%d.%d,Y=%s%d.%d,Z=%s%d.%d\r\n",
		              (X<0)?"-":"+", (int)((X<0?-X:X)/10), (int)((X<0?-X:X)%10),
		              (Y<0)?"-":"+", (int)((Y<0?-Y:Y)/10), (int)((Y<0?-Y:Y)%10),
		              (Z<0)?"-":"+", (int)((Z<0?-Z:Z)/10), (int)((Z<0?-Z:Z)%10));
		return;
	}

	/* ---------------- SERVO 系列 ---------------- */
	if(strncmp(Line,"SERVO",5) == 0 || strncmp(Line,"servo",5) == 0 ||
	   strncmp(Line,"Servo",5) == 0)
	{
		Arg = Line + 5;

		/* SERVO ? */
		if(Arg[0] == ' ' && Arg[1] == '?')
		{
			Proto_ServoStatus("");
			return;
		}

		if(Arg[0] != ' ')
		{
			Serial_SendString("ERR\r\n");
			return;
		}

		Arg = Line + 6;                 /* 跳过空格 */

		/* SERVO SET <angle> */
		if(strncmp(Arg,"SET",3) == 0 || strncmp(Arg,"set",3) == 0)
		{
			if(Arg[3] != ' ')
			{
				Serial_SendString("ERR\r\n");
				return;
			}

			/* 定角模式开启时拒绝手动设置（题目要求） */
			if(Servo_GetHold() == 1)
			{
				Serial_SendString("ERR HOLD ON\r\n");
				return;
			}

			if(Servo_GetHoldFault())
			{
				Serial_SendString("ERR HOLD FAULT; USE SERVO HOLD OFF\r\n");
				return;
			}

			Value = strtol(Arg+4,&Arg,10);
			if(Value < 0 || Value > 180)
			{
				Serial_SendString("ERR\r\n");
				return;
			}

			Servo_SetAngle((uint8_t)Value);
			Menu_SetDirty();            /* 舵机页面显示与串口设置同步 */
			Proto_ServoStatus("OK ");
			return;
		}

		/* SERVO HOLD ON / OFF / ? */
		if(strncmp(Arg,"HOLD",4) == 0 || strncmp(Arg,"hold",4) == 0)
		{
			if(Arg[4] == ' ' && Arg[5] == '?')
			{
				Serial_printf("SERVO HOLD=%s FAULT=%s CODE=%u\r\n",
				              Servo_GetHold() ? "ON" : "OFF",
				              Servo_GetHoldFault() ? "ON" : "OFF",
				              (unsigned int)Servo_GetHoldFaultCode());
				return;
			}

			if(Arg[4] != ' ')
			{
				Serial_SendString("ERR\r\n");
				return;
			}

			/* 定角闭环：开启后以当前 IMU 姿态为目标，
			   通过带角速度阻尼的慢速外环把机构保持在目标姿态 */
			if(Proto_IsWord(Arg+5,"ON"))
			{
				Servo_SetHold(1);
				Menu_SetDirty();
				if(Servo_GetHold() && !Servo_GetHoldFault()) Serial_SendString("OK SERVO HOLD=ON\r\n");
				else Serial_printf("ERR SERVO HOLD FAULT=%u\r\n",(unsigned int)Servo_GetHoldFaultCode());
				return;
			}

			if(Proto_IsWord(Arg+5,"OFF"))
			{
				Servo_SetHold(0);
				Menu_SetDirty();
				Serial_SendString("OK SERVO HOLD=OFF\r\n");
				return;
			}

			/* SERVO HOLD CAL：把当前姿态重新记为目标姿态（现场调试用） */
			if(Proto_IsWord(Arg+5,"CAL"))
			{
				Servo_HoldCal();
				Menu_SetDirty();
				if(MPU6050_HasFreshSample() && !Servo_GetHoldFault()) Serial_SendString("OK SERVO HOLD CAL\r\n");
				else Serial_SendString("ERR SERVO HOLD CAL\r\n");
				return;
			}

			Serial_SendString("ERR\r\n");
			return;
		}

		/* SERVO TRACE ON / OFF / ? */
		if(strncmp(Arg,"TRACE",5) == 0 || strncmp(Arg,"trace",5) == 0)
		{
			if(Arg[5] == ' ' && Arg[6] == '?')
			{
				Serial_printf("SERVO TRACE=%s\r\n",TraceOn ? "ON" : "OFF");
				return;
			}

			if(Arg[5] != ' ')
			{
				Serial_SendString("ERR\r\n");
				return;
			}

			if(Proto_IsWord(Arg+6,"ON"))
			{
				TraceOn = 1;
				Serial_SendString("TYPE,TIME_MS,IMU_X10,IMU_Y10,IMU_Z10,TARGET10,ANGLE10,HOLD,FAULT\r\n");
				Serial_SendString("OK SERVO TRACE=ON\r\n");
				return;
			}

			if(Proto_IsWord(Arg+6,"OFF"))
			{
				TraceOn = 0;
				Serial_SendString("OK SERVO TRACE=OFF\r\n");
				return;
			}

			Serial_SendString("ERR\r\n");
			return;
		}

		Serial_SendString("ERR\r\n");
		return;
	}

	/* ---------------- 无法识别 ---------------- */
	Serial_SendString("ERR\r\n");
}

/**
  * @brief  命令任务：主循环每 10ms 调一次；把收到的字节拼成行并解析
  * @param  无
  * @retval 无
  */
void Protocol_Task(void)
{
	uint8_t Count = 0;
	uint8_t Ch;
	char    Cmd[PROTO_LINE_MAX];

	while(Serial_Available() > 0 && Count < PROTO_CHUNK_MAX)
	{
		Count++;
		Ch = Serial_ReadByte();

		if(Ch == '\r' || Ch == '\n')
		{
			if(LineLen > 0 && LineOver == 0)
			{
				LineBuf[LineLen] = '\0';
				strcpy(Cmd,LineBuf);            /* 拷贝一份再解析，解析里会改字符串 */
				LineLen = 0;
				if(!Proto_IsWord(Cmd,"LIGHT ?")) Menu_Wake(); /* 亮度只读轮询不打断60秒待机，其他命令保留唤醒。 */
				Proto_Exec(Cmd);                /* 一次任务最多执行一条命令就返回 */
				return;
			}

			LineLen  = 0;                       /* 空行或超长行：直接丢弃 */
			LineOver = 0;
			continue;
		}

		if(LineLen < (PROTO_LINE_MAX - 1))
		{
			LineBuf[LineLen] = (char)Ch;
			LineLen++;
		}
		else
		{
			LineOver = 1;                       /* 太长了，本行作废 */
		}
	}
}

/**
  * @brief  周期任务：主循环每 50ms 调一次；SERVO TRACE ON 时输出一行 20Hz 调试数据
  * @param  无
  * @retval 无
  * 备注：只有在 TRACE 打开时才发送，且长度不到 50 字节，
  *       115200 下约 4ms 就能发完，不会影响其它任务
  */
void Protocol_Update(void)
{
	static uint8_t Div = 0;
	int16_t X,Y,Z;

	if(TraceOn == 0)
	{
		return;
	}

	Div++;
	if(Div < PROTO_TRACE_DIV)
	{
		return;
	}
	Div = 0;

	MPU6050_GetAngle10(&X,&Y,&Z);

	/* 字段：时间戳(秒×10)、三轴姿态角(0.1°)、目标姿态角(0.1°)、
	         舵机实际输出角度(0.1°)、定角开关、故障标志 */
	Serial_printf("H,%lu,%d,%d,%d,%d,%d,%d,%d\r\n",
	              (unsigned long)Timer_GetTick(),     /* 毫秒时间戳，观察实际调度间隔 */
	              (int)X,
	              (int)Y,
	              (int)Z,
	              (int)Servo_GetHoldAngle10(),
	              (int)Servo_GetOutputAngle10(),
	              (int)Servo_GetHold(),
	              (int)Servo_GetHoldFault());
}
