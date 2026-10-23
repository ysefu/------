#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "OLED_Chinese.h"
#include "OLED_Effects.h"
#include "Timer.h"
#include "Key.h"
#include "Encoder.h"
#include "Clock.h"
#include "MPU6050.h"
#include "Servo.h"
#include "LightSensor.h"
#include "Menu.h"

/*----------------------------------------------------------------------------
 * 文件：Menu.c / Menu.h
 * 用处：中文开机动画、五个功能页面、按键和编码器交互。
 * 对应外设：OLED软件IIC PB8/PB9；中文字模由OLED_Chinese模块提供。
 * 备注：原OLED驱动时序不变。汉字16x16占两格，ASCII8x16占一格。
 *       全部显示限定四行十六格；主菜单单列三行，光标移动到边缘时滚动列表。
 *       KEY1始终顺序切页；主菜单旋转选择、KEY2进入所选项，KEY2选择/保存/切换定角，报警期间按键由App处理。
 *----------------------------------------------------------------------------*/

#define MENU_MAIN_ITEM_NUM      5       /* 主菜单条目数 */

/* 版本标记：上电时显示在 OLED 第 1 行、并打印到串口。
   作用：一眼确认烧进去的固件到底是哪一版（改过显示后如果这里没变，说明没编译进去） */
#define MENU_BUILD_ID           "v41 10-16"

#define MENU_MARK_CHAR          '*'     /* 功能页当前参数项标记 */


static uint8_t  MenuPage    = MENU_PAGE_MAIN;   /* 当前界面 */
static uint8_t MainTop = 0; /* 可见三行的第一项索引，0~2 */
static uint8_t  MainCursor  = 0;                /* 主菜单光标 0~4 */

static uint8_t  TimeCursor  = 0;                /* 时间设置页：0=时 1=分 2=秒 */
static uint8_t  TimeEditH   = 0;                /* 时间设置页的编辑副本（时） */
static uint8_t  TimeEditM   = 0;                /* 时间设置页的编辑副本（分） */
static uint8_t  TimeEditS   = 0;                /* 时间设置页的编辑副本（秒） */
static uint8_t  TimeLoaded  = 0;                /* 本轮进入时间页后是否已取过系统时间 */
static uint8_t AlarmIndex = 1; /* 当前编辑的闹钟，1~CLOCK_ALARM_NUM */
static uint32_t AlarmEditRevision = 0;
static uint8_t  AlarmCursor = 0;                /* 闹钟页：0=编号 1=时 2=分 3=秒 4=开关 */

static uint8_t  AlarmEditH  = 0;                /* 闹钟页正在编辑的时/分/秒副本 */
static uint8_t  AlarmEditM  = 0;
static uint8_t  AlarmEditS  = 0;
static uint8_t  AlarmEditEn = 0;
static uint8_t  AlarmLoaded = 0;                /* 本轮进入闹钟页后是否已取过数据 */

static uint8_t  PageDirty     = 1;              /* 1=本页内容需要重画一次 */
static uint8_t  NeedRedraw    = 1;              /* 1=界面切换过，必须整屏重画 */
static uint8_t  LineDirty     = 0x0E;           /* 按位记录哪一行要重画：bit1=第2行 bit2=第3行 bit3=第4行 */

/* 行掩码（配合 LineDirty 使用）：标记某一行需要重画 */
#define LINE2_DIRTY     0x02
#define LINE3_DIRTY     0x04
#define LINE4_DIRTY     0x08
#define LINE234_DIRTY   0x0E

/*----------------------------------------------------------------------------
 * 【消屏闪的最终手段：按"字符格"重画】
 *   CellMask2/3/4 每一位对应本行的一个字符格（bit0 = 第 1 列，bit15 = 第 16 列）。
 *   转编码器改"时"时，只把第 1~2 列这两位置 1 —— 于是屏幕上只有那两位数字被重写，
 *   其余 14 个字符（含 Edit 标签）一个字节都不发，自然就不会闪。
 *   只有换页 / 串口改值 / 时钟走秒这类"整行都可能变"的场合才整行重画。
 *----------------------------------------------------------------------------*/
static uint16_t CellMask2 = 0xFFFF;
static uint16_t CellMask3 = 0xFFFF;
static uint16_t CellMask4 = 0xFFFF;

/* 标记某一行第 col 列（1~16）需要重画 */
static void Menu_MarkCell(uint8_t Line, uint8_t Col)
{
	if(Line == 2)      CellMask2 |= (uint16_t)(1u << (Col - 1));
	else if(Line == 3) CellMask3 |= (uint16_t)(1u << (Col - 1));
	else if(Line == 4) CellMask4 |= (uint16_t)(1u << (Col - 1));
}

/* 标记某一行第 col1~col2 列需要重画 */
static void Menu_MarkCellRange(uint8_t Line, uint8_t Col1, uint8_t Col2)
{
	uint8_t c;
	for(c=Col1;c<=Col2;c++)
	{
		Menu_MarkCell(Line,c);
	}
}
static uint8_t  HeaderLastSec = 0xFF;           /* 上次画状态栏时的秒数 */
static uint8_t  HeaderLastPage= 0xFF;           /* 上次画状态栏时的界面 */
static uint8_t  HeaderLastRng = 0xFF;           /* 上次画状态栏时的报警状态 */

/* ---- 显示内容缓存：和上次画的值一样就不重画（消屏闪的关键） ---- */
static int16_t  LastGyroX = 0x7FFF;             /* IMU 页上次显示的 GX/GY/GZ */
static int16_t  LastGyroY = 0x7FFF;
static int16_t  LastGyroZ = 0x7FFF;
static uint8_t  LastServoAngle = 0xFF;          /* 舵机页上次显示的角度 */
static int16_t  LastHoldMode = -1;              /* 舵机页上次显示的定角状态 */
static int16_t  LastHoldAngle10 = 0x7FFF;       /* 舵机页上次显示的目标姿态角 */

/* 目标编辑副本：只有短按KEY2确认后才写入闭环。 */
static int16_t HoldEditAngle10=0,HoldEditBase10=0;
static uint8_t HoldEditPending=0;
static int16_t LastServoY10=32767;
static uint8_t LastHoldEdit=255,LastHoldFresh=255,LastHoldFaultCode=255;
static uint8_t LastLightAuto=255,LastLightContrast=0,LastLightManual=0,LastLightReady=255;
static uint16_t LastLightADC=0xFFFF;

/* ---- 开机动画状态 ---- */
static uint8_t  BootStep    = 0;                /* 动画阶段 */
static uint8_t  BootDone    = 0;                /* 1=动画结束 */
static uint32_t BootStart   = 0;                /* 动画开始时刻 */
static uint8_t Standby=0,WakeRelease=0;
static uint32_t LastActivity=0,StandbyTime=0xFFFFFFFFu;
static uint8_t  BootLastPct = 0xFF;             /* 上次画到的进度，变了才重画 */

/* 5 个界面的名称 */


/* 主菜单条目（对应四个功能页面，一屏预览） */


/*============================================================================
 * 一、基础绘图小工具
 *============================================================================*/

/**
  * @brief  内部使用：往一行里写【固定列宽】的字符串，不足补空格、超出截断
  * @param  Line   行 1~4
  * @param  Column 起始列
  * @param  Str    字符串
  * @param  Width  固定占几个字符位（不足补空格，超出截断）
  * @retval 无
  * 备注：固定列宽是"显示永不错位"的根本保证 ——
  *       不管页面名是 3 个字符还是 6 个字符，右边内容的位置都不会变
  */



/**
  * @brief  内部使用：把"时:分:秒"画到指定行的指定列（固定占 8 个字符位）
  * @param  Line   行号 1~4
  * @param  Column 起始列 1~16
  * @param  Hour   时
  * @param  Minute 分
  * @param  Second 秒
  * @retval 无
  */
static void Menu_DrawTimeAt(uint8_t Line, uint8_t Column, uint8_t Hour, uint8_t Minute, uint8_t Second)
{
	/* 逐个字符显式写出，不走 OLED_ShowNum 的"位数"参数 ——
	   避免参数顺序/位数传错导致"秒只显示一位、整行错位"这类问题。
	   本函数固定占 8 个字符位（HH:MM:SS），位置永远是确定的。 */
	OLED_ShowChar(Line,(uint8_t)(Column+0),(char)('0' + Hour   / 10));
	OLED_ShowChar(Line,(uint8_t)(Column+1),(char)('0' + Hour   % 10));
	OLED_ShowChar(Line,(uint8_t)(Column+2),':');
	OLED_ShowChar(Line,(uint8_t)(Column+3),(char)('0' + Minute / 10));
	OLED_ShowChar(Line,(uint8_t)(Column+4),(char)('0' + Minute % 10));
	OLED_ShowChar(Line,(uint8_t)(Column+5),':');
	OLED_ShowChar(Line,(uint8_t)(Column+6),(char)('0' + Second / 10));
	OLED_ShowChar(Line,(uint8_t)(Column+7),(char)('0' + Second % 10));
}

/**
  * @brief  内部使用：把"±xxx.x"形式的带符号一位小数画到指定位置
  * @param  Line    行号 1~4
  * @param  Column  起始列 1~16
  * @param  Value10 数值，单位 0.1（如 -1234 表示 -123.4）
  * @param  Width   整数部分占几位
  * @retval 无
  */



/**
  * @brief  内部使用：往一行里写字符串并补空格到固定宽度（Menu_ShowFixed 的别名）
  * @param  Line   行 1~4
  * @param  Column 起始列
  * @param  Str    字符串
  * @param  Width  补到几个字符宽
  * @retval 无
  */



/*============================================================================
 * 二、开机动画（非阻塞，全部用基础显示函数画）
 *============================================================================*/

/**
  * @brief  初始化：清屏并从第 1 阶段开始播放开机动画
  * @param  无
  * @retval 无
  */
void Menu_Init(void)
{
	MenuPage    = MENU_PAGE_MAIN;
	MainCursor  = 0;
	MainTop = 0;
	TimeCursor  = 0;
	AlarmCursor = 0;
	AlarmIndex=1; AlarmEditRevision=0;
	AlarmLoaded = 0;
	HoldEditPending=0;

	PageDirty      = 1;
	NeedRedraw     = 1;
	HeaderLastSec  = 0xFF;
	HeaderLastPage = 0xFF;
	HeaderLastRng  = 0xFF;

	/* 首次进入：整片都要画 */
	LineDirty = LINE234_DIRTY;
	CellMask2 = 0xFFFF;
	CellMask3 = 0xFFFF;
	CellMask4 = 0xFFFF;

	BootStep    = 0;
	BootDone    = 0;
	BootStart   = 0;
	BootLastPct = 0xFF;
	Standby=0;WakeRelease=0;LastActivity=Timer_GetTick();StandbyTime=0xFFFFFFFFu;
	OLED_Effects_Invalidate();
}

/**
  * @brief  开机动画任务：主循环每 20ms 调用一次
  * @param  无
  * @retval 无
  * 备注：1) 动画总时长 MENU_BOOT_MS（默认 2 秒，题目要求不少于 1 秒）
  *       2) 【非阻塞】没有一处 delay，全部按 1ms 节拍分阶段推进，
  *          动画期间时钟照常走秒、串口照常收命令
  *       3) 进度条用 '#' 和 '-' 字符拼，不涉及像素级操作
  */
void Menu_BootTask(void)
{
 uint32_t Elapsed;uint8_t Index;
 if(BootDone)return;
 if(!BootStep){BootStart=Timer_GetTick();BootStep=1;BootLastPct=255;OLED_Effects_Invalidate();}
 Elapsed=Timer_GetTick()-BootStart;
 if(Elapsed>=MENU_BOOT_MS)
 {
  BootDone=1;LastActivity=Timer_GetTick();OLED_Clear();Menu_SetDirty();NeedRedraw=1;HeaderLastPage=255;return;
 }
 Index=(uint8_t)(Elapsed*OLED_EFFECTS_BOOT_FRAMES/MENU_BOOT_MS);
 if(Index!=BootLastPct){OLED_Effects_Boot(Index);BootLastPct=Index;}
}


/**
  * @brief  开机动画是否已经结束
  * @param  无
  * @retval 1=结束
  */
uint8_t Menu_BootDone(void)
{
	return BootDone;
}

/*============================================================================
 * 三、界面切换与状态栏
 *============================================================================*/

/**
  * @brief  内部页面进入与状态复位；KEY1调用顺序入口，KEY2调用选择入口
  * @param  无
  * @retval 切换后的界面编号
  */
static uint8_t Menu_EnterPage(uint8_t Page)
{
	if(Page>=MENU_PAGE_NUM) return MenuPage;
	MenuPage=Page;
	HoldEditPending=0; /* 换页丢弃未确认编辑，已确认目标继续保持。 */

	TimeCursor     = 0;
	TimeLoaded     = 0;             /* 下次进时间页时重新取一次当前时间当编辑副本 */
	AlarmCursor    = 0;
	AlarmLoaded    = 0;             /* 下次进闹钟页时重新从 Clock 取闹钟数据 */
	PageDirty      = 1;
	NeedRedraw     = 1;
	LineDirty      = LINE234_DIRTY;     /* 换页要整片重画 */
	CellMask2      = 0xFFFF;
	CellMask3      = 0xFFFF;
	CellMask4      = 0xFFFF;
	HeaderLastSec  = 0xFF;
	HeaderLastPage = 0xFF;

	return MenuPage;
}

uint8_t Menu_NextPage(void)
{
	return Menu_EnterPage((uint8_t)((MenuPage+1)%MENU_PAGE_NUM));
}


/**
  * @brief  取当前界面编号
  * @param  无
  * @retval 0~4（MENU_PAGE_xxx）
  */
uint8_t Menu_GetPage(void)
{
	return MenuPage;
}

/**
  * @brief  标记需要重画（串口把值改了之后调用，保证页面显示与串口同步）
  * @param  无
  * @retval 无
  */
void Menu_SetDirty(void)
{
	PageDirty      = 1;
	LineDirty      = LINE234_DIRTY;     /* 串口改了值，整片重画一次保证同步 */
	CellMask2      = 0xFFFF;            /* 整行都要重画 */
	CellMask3      = 0xFFFF;
	CellMask4      = 0xFFFF;
	HeaderLastSec  = 0xFF;
}

/**
  * @brief  界面是否发生过切换
  * @param  无
  * @retval 1=切换过，需要整屏重画一次（取走后自动清零）
  */
/* 切换编号/外部修改后加载该路闹钟。未保存的编辑在切换编号时丢弃。 */
static void Menu_LoadAlarmEditor(void)
{
	ClockTime_t Time;
	if(Clock_GetAlarm(AlarmIndex,&Time)!=SUCCESS) return;
	AlarmEditH=Time.Hour; AlarmEditM=Time.Minute; AlarmEditS=Time.Second;
	AlarmEditEn=Clock_GetAlarmEnable(AlarmIndex)==ALARM_ON;
	AlarmEditRevision=Clock_GetAlarmRevision(AlarmIndex);
	AlarmLoaded=1;
	Menu_SetDirty();
}

uint8_t Menu_NeedRedraw(void)
{
	uint8_t Temp = NeedRedraw;

	NeedRedraw = 0;

	return Temp;
}

/**
  * @brief  绘制第 1 行状态栏：HH:MM:SS + 页面标识 + 报警提示
  * @param  无
  * @retval 无
  * 备注：只在"秒变化 / 切页面 / 报警状态变化"时重画
  */
uint8_t Menu_IsStandby(void){return Standby;}
void Menu_Wake(void)
{
 LastActivity=Timer_GetTick();
 if(Standby)
 {
  Standby=0;OLED_Clear();Menu_SetDirty();NeedRedraw=1;HeaderLastPage=255;
 }
}
static void Menu_DrawStandby(void)
{
 uint8_t H=Clock_GetHour(),M=Clock_GetMinute(),S=Clock_GetSecond();
 uint32_t Value=(uint32_t)H*3600u+(uint32_t)M*60u+S;
 if(Value!=StandbyTime){OLED_Effects_Clock(H,M,S);StandbyTime=Value;}
}
void Menu_DrawHeader(void)
{
 static const OLED_ChineseText Names[MENU_PAGE_NUM]={CN_MAIN,CN_TIME,CN_ALARM,CN_IMU,CN_SERVO,CN_BRIGHT};
 uint8_t Second=Clock_GetSecond(),Ringing=Clock_IsRinging();
 if(Standby)return;
 if(Second==HeaderLastSec && MenuPage==HeaderLastPage && Ringing==HeaderLastRng) return;
 HeaderLastSec=Second; HeaderLastPage=MenuPage; HeaderLastRng=Ringing;
 Menu_DrawTimeAt(1,1,Clock_GetHour(),Clock_GetMinute(),Second);
 OLED_ShowString(1,9,"        ");
 OLED_Chinese_ShowText(1,10,Names[MenuPage]);
 OLED_ShowChar(1,16,Ringing ? '!' : ' ');
}


/*============================================================================
 * 四、各界面主体内容
 *============================================================================*/

/**
  * @brief  内部使用：把一行组装成 16 个字符，然后"只把被标记的格子"写到屏上
  * @param  Line 行号 1~4
  * @param  Buf  16 个字符的内容
  * @param  Mask 位掩码，bit0 = 第 1 列 …… bit15 = 第 16 列
  * @retval 无
  * 备注：这是消屏闪的核心 —— 只有被标记的格子才产生 IIC 流量
  */
static void Menu_WriteCells(uint8_t Line, char *Buf, uint16_t Mask)
{
 OLED_Chinese_DrawCells(Line,Buf,Mask,0);
}


/**
  * @brief  内部使用：把被标记的字符格重画一遍（按当前页面组装内容）
  * @param  无
  * @retval 无
  */
static void Menu_BufferClear(char *Buf)
{
 uint8_t i; for(i=0;i<16;i++) Buf[i]=' ';
}
static void Menu_BufferTime(char *Buf,uint8_t H,uint8_t M,uint8_t S)
{
 Buf[0]=(char)('0'+H/10); Buf[1]=(char)('0'+H%10); Buf[2]=':';
 Buf[3]=(char)('0'+M/10); Buf[4]=(char)('0'+M%10); Buf[5]=':';
 Buf[6]=(char)('0'+S/10); Buf[7]=(char)('0'+S%10);
}
static int16_t Menu_ReadPitch10(void)
{
 int16_t X,Y,Z;MPU6050_GetAngle10(&X,&Y,&Z);return Y;
}
static void Menu_SyncHoldEditor(void)
{
 if(HoldEditPending && (!Servo_GetHold() || Servo_GetHoldFault() ||
    Servo_GetHoldAngle10()!=HoldEditBase10))
 {HoldEditPending=0; Menu_SetDirty();}
}
/* 当前/目标标签占4格，Y占1格，带符号角度与度符号占6格。 */
static void Menu_BufferPitch(char *Buf,OLED_ChineseText Label,int16_t Value,uint8_t Valid)
{
 uint16_t Abs=Value<0 ? (uint16_t)(-(int32_t)Value) : (uint16_t)Value;
 Menu_BufferClear(Buf); OLED_Chinese_PutText(Buf,0,Label); Buf[4]='Y';
 if(!Valid) {Buf[5]='-';Buf[6]='-';Buf[7]='-';Buf[8]='.';Buf[9]='-';}
 else {Buf[5]=Value<0 ? '-' : '+';Buf[6]=(char)('0'+Abs/100%10);
 Buf[7]=(char)('0'+Abs/10%10);Buf[8]='.';Buf[9]=(char)('0'+Abs%10);}
 OLED_Chinese_PutDegree(Buf,10);
}
static void Menu_BufferBrightness(char *Buf,OLED_ChineseText Label,uint8_t Level)
{
 Menu_BufferClear(Buf);OLED_Chinese_PutText(Buf,0,Label);
 Buf[5]=(char)('0'+Level/100);Buf[6]=(char)('0'+Level/10%10);Buf[7]=(char)('0'+Level%10);
 Buf[8]='/';Buf[9]='2';Buf[10]='5';Buf[11]='5';
}
static void Menu_DrawCellPage(void)
{
 char Buf[16]; uint8_t i; int16_t X,Y,Z,Value;
 if(MenuPage==MENU_PAGE_MAIN)
 {
  static const OLED_ChineseText Names[MENU_MAIN_ITEM_NUM]={CN_TIME_SET,CN_ALARM_SET,CN_IMU_DATA,CN_SERVO_ANGLE,CN_BRIGHT_SET};
  for(i=0;i<3;i++)
  {
   uint8_t Item=(uint8_t)(MainTop+i);
   uint16_t Mask=i==0 ? CellMask2 : (i==1 ? CellMask3 : CellMask4);
   Menu_BufferClear(Buf);
   /* MainTop为0~2，Item始终在五个条目范围内。 */
   if(Item<MENU_MAIN_ITEM_NUM)
   {
    Buf[0]=Item==MainCursor ? '>' : ' ';
    OLED_Chinese_PutText(Buf,2,Names[Item]);
   }
   /* 右侧箭头提示屏幕之外还有内容。 */
   if(i==0 && MainTop>0) Buf[15]='^';
   if(i==2 && MainTop+3<MENU_MAIN_ITEM_NUM) Buf[15]='v';
   Menu_WriteCells((uint8_t)(i+2),Buf,Mask);
  }
  return;
 }
 if(MenuPage==MENU_PAGE_TIME)
 {
  Menu_BufferClear(Buf); Menu_BufferTime(Buf,TimeEditH,TimeEditM,TimeEditS);
  OLED_Chinese_PutText(Buf,10,CN_EDIT); Menu_WriteCells(2,Buf,CellMask2);
  Menu_BufferClear(Buf);
  Buf[0]=TimeCursor==0 ? '*' : ' '; Buf[5]=TimeCursor==1 ? '*' : ' '; Buf[10]=TimeCursor==2 ? '*' : ' ';
  OLED_Chinese_PutText(Buf,1,CN_HOUR); OLED_Chinese_PutText(Buf,6,CN_MINUTE); OLED_Chinese_PutText(Buf,11,CN_SECOND);
  Menu_WriteCells(3,Buf,CellMask3);
  Menu_BufferClear(Buf); OLED_Chinese_PutText(Buf,0,CN_SAVE); Menu_WriteCells(4,Buf,CellMask4); return;
 }
 if(MenuPage==MENU_PAGE_ALARM)
 {
  Menu_BufferClear(Buf); OLED_Chinese_PutText(Buf,0,CN_ALARM);
  Buf[4]=(char)('0'+AlarmIndex); Buf[5]='/'; Buf[6]=(char)('0'+CLOCK_ALARM_NUM);
  OLED_Chinese_PutText(Buf,10,AlarmEditEn ? CN_ON : CN_OFF); Menu_WriteCells(2,Buf,CellMask2);
  Menu_BufferClear(Buf); Menu_BufferTime(Buf,AlarmEditH,AlarmEditM,AlarmEditS);
  OLED_Chinese_PutText(Buf,10,CN_TARGET); Menu_WriteCells(3,Buf,CellMask3);
  Menu_BufferClear(Buf);
  if(Clock_IsRinging())
  {
   OLED_Chinese_PutText(Buf,0,CN_RING);
   Buf[5]='K'; Buf[6]='1'; Buf[7]='+'; Buf[8]='K'; Buf[9]='2';
   OLED_Chinese_PutText(Buf,11,CN_OFF);
  }
  else
  {
   static const OLED_ChineseText Fields[5]={CN_ALARM,CN_HOUR,CN_MINUTE,CN_SECOND,CN_SWITCH};
   Buf[0]='*'; OLED_Chinese_PutText(Buf,1,Fields[AlarmCursor]);
   OLED_Chinese_PutText(Buf,6,CN_SAVE);
  }
  Menu_WriteCells(4,Buf,CellMask4); return;
 }
 if(MenuPage==MENU_PAGE_IMU)
 {
  int16_t Values[3];
  if(!MPU6050_IsReady())
  {
   Menu_BufferClear(Buf); OLED_Chinese_PutText(Buf,0,CN_IMU_FAULT); Menu_WriteCells(2,Buf,CellMask2);
   Menu_BufferClear(Buf); OLED_Chinese_PutText(Buf,0,CN_CHECK); Menu_WriteCells(3,Buf,CellMask3);
   Menu_BufferClear(Buf); Menu_WriteCells(4,Buf,CellMask4); return;
  }
  MPU6050_GetGyroDps10(&X,&Y,&Z); Values[0]=X; Values[1]=Y; Values[2]=Z;
  for(i=0;i<3;i++)
  {
   uint32_t Abs; Value=Values[i]; Abs=Value<0 ? (uint32_t)(-(int32_t)Value) : (uint32_t)Value;
   Menu_BufferClear(Buf); Buf[0]='G'; Buf[1]=(char)('X'+i); Buf[3]=Value<0 ? '-' : '+';
   Buf[4]=(char)('0'+Abs/1000%10); Buf[5]=(char)('0'+Abs/100%10); Buf[6]=(char)('0'+Abs/10%10);
   Buf[7]='.'; Buf[8]=(char)('0'+Abs%10);
   OLED_Chinese_PutDegree(Buf,11); Buf[12]='/'; Buf[13]='s';
   Menu_WriteCells((uint8_t)(i+2),Buf,i==0 ? CellMask2 : (i==1 ? CellMask3 : CellMask4));
  }
  return;
 }
 if(MenuPage==MENU_PAGE_BRIGHTNESS)
 {
  Menu_BufferClear(Buf);OLED_Chinese_PutText(Buf,0,LightSensor_GetAuto() ? CN_AUTO : CN_MANUAL);
  Buf[6]='K';Buf[7]='2';OLED_Chinese_PutText(Buf,8,CN_CHANGE);Menu_WriteCells(2,Buf,CellMask2);
  Menu_BufferBrightness(Buf,CN_CURRENT,LightSensor_GetContrast());Menu_WriteCells(3,Buf,CellMask3);
  if(LightSensor_GetAuto())
  {
   Menu_BufferClear(Buf);OLED_Chinese_PutText(Buf,0,CN_LIGHT);
   if(!LightSensor_IsReady()) OLED_Chinese_PutText(Buf,5,CN_FAULT);
   else {Value=(int16_t)LightSensor_GetFiltered();Buf[5]=(char)('0'+Value/1000);Buf[6]=(char)('0'+Value/100%10);Buf[7]=(char)('0'+Value/10%10);Buf[8]=(char)('0'+Value%10);}
  }
  else Menu_BufferBrightness(Buf,CN_SET_VALUE,LightSensor_GetManual());
  Menu_WriteCells(4,Buf,CellMask4);return;
 }
 if(MenuPage==MENU_PAGE_SERVO)
 {
  if(Servo_GetHold() || Servo_GetHoldFault())
  {
   Menu_BufferPitch(Buf,CN_CURRENT,Menu_ReadPitch10(),MPU6050_HasFreshSample());
   Menu_WriteCells(2,Buf,CellMask2);
   Menu_BufferPitch(Buf,CN_TARGET,HoldEditPending ? HoldEditAngle10 : Servo_GetHoldAngle10(),1);
   OLED_Chinese_PutText(Buf,12,HoldEditPending ? CN_EDIT : (Servo_GetHoldFault() ? CN_FAULT : CN_KEEP));
   Menu_WriteCells(3,Buf,CellMask3);
   Menu_BufferClear(Buf);
   if(Servo_GetHoldFault())
   {
    OLED_Chinese_PutText(Buf,0,CN_FAULT);Buf[4]=(char)('0'+Servo_GetHoldFaultCode()%10);
    OLED_Chinese_PutText(Buf,7,CN_CLEAR);
   }
   else {OLED_Chinese_PutText(Buf,0,CN_APPLY);OLED_Chinese_PutText(Buf,9,CN_LONG_OFF);}
   Menu_WriteCells(4,Buf,CellMask4);
  }
  else
  {
   Menu_BufferClear(Buf);OLED_Chinese_PutText(Buf,0,CN_SERVO_ANGLE);OLED_Chinese_PutText(Buf,12,CN_MANUAL);
   Menu_WriteCells(2,Buf,CellMask2);
   Menu_BufferClear(Buf);OLED_Chinese_PutText(Buf,0,CN_TARGET);
   Value=Servo_GetAngle();Buf[4]=(char)('0'+Value/100);Buf[5]=(char)('0'+Value/10%10);Buf[6]=(char)('0'+Value%10);
   OLED_Chinese_PutDegree(Buf,7);OLED_Chinese_PutText(Buf,10,CN_ANGLE);Menu_WriteCells(3,Buf,CellMask3);
   Menu_BufferClear(Buf);OLED_Chinese_PutText(Buf,0,CN_HOLD);OLED_Chinese_PutText(Buf,4,CN_SHORT_OFF);
   OLED_Chinese_PutText(Buf,8,CN_LONG_ON);Menu_WriteCells(4,Buf,CellMask4);
  }
 }
}


/**
  * @brief  绘制当前界面的主体内容（第 2~4 行）
  * @param  无
  * @retval 无
  */
void Menu_DrawPage(void)
{
 if(Standby){Menu_DrawStandby();return;}
	/* 【消屏闪】只重画"被标记的字符格"：
	   转编码器改哪两位，就只写哪两位；Edit 标签等内容一个字节都不发。
	   这里只做两件事：① 判断内容有没有变（没变直接返回）② 交给按格重画函数 */

	if((MenuPage == MENU_PAGE_IMU) && (MPU6050_IsReady() != 0))
	{
		int16_t gx,gy,gz;
		MPU6050_GetGyroDps10(&gx,&gy,&gz);
		if(PageDirty == 0 && gx == LastGyroX && gy == LastGyroY && gz == LastGyroZ)
		{
			PageDirty = 0;
			return;                         /* 三轴都没变：一个字都不写 */
		}

		/* 哪一轴变了就只重画哪一行的那几个数字格（第 1~3 行分别是 GX/GY/GZ） */
		if(gx != LastGyroX)
		{
			LineDirty |= LINE2_DIRTY;
			Menu_MarkCellRange(2,4,9);
		}
		if(gy != LastGyroY)
		{
			LineDirty |= LINE3_DIRTY;
			Menu_MarkCellRange(3,4,9);
		}
		if(gz != LastGyroZ)
		{
			LineDirty |= LINE4_DIRTY;
			Menu_MarkCellRange(4,4,9);
		}
	}
	else if(MenuPage == MENU_PAGE_SERVO)
 {
  int16_t Mode=(int16_t)(Servo_GetHoldFault() ? 2 : (Servo_GetHold() ? 1 : 0));
  Menu_SyncHoldEditor();
  if(Mode!=LastHoldMode) {CellMask2=CellMask3=CellMask4=0xFFFF;}
  if(Mode)
  {
   if(Menu_ReadPitch10()!=LastServoY10 || MPU6050_HasFreshSample()!=LastHoldFresh) CellMask2=0xFFFF;
   if((HoldEditPending ? HoldEditAngle10 : Servo_GetHoldAngle10())!=LastHoldAngle10 ||
      HoldEditPending!=LastHoldEdit) CellMask3=0xFFFF;
   if(Servo_GetHoldFaultCode()!=LastHoldFaultCode) CellMask4=0xFFFF;
  }
  else if(Servo_GetAngle()!=LastServoAngle) Menu_MarkCellRange(3,5,7);
  if(!PageDirty && !CellMask2 && !CellMask3 && !CellMask4) return;
 }
 else if(MenuPage == MENU_PAGE_BRIGHTNESS)
 {
  if(LightSensor_GetAuto()!=LastLightAuto) CellMask2=CellMask4=0xFFFF;
  if(LightSensor_GetContrast()!=LastLightContrast) CellMask3=0xFFFF;
  if(LightSensor_GetAuto())
  {if(LightSensor_GetFiltered()!=LastLightADC || LightSensor_IsReady()!=LastLightReady) CellMask4=0xFFFF;}
  else if(LightSensor_GetManual()!=LastLightManual) CellMask4=0xFFFF;
  if(!PageDirty && !CellMask2 && !CellMask3 && !CellMask4) return;
 }
	else if(MenuPage == MENU_PAGE_ALARM)
	{
		static uint8_t LastRing=0xFF;
		if(LastRing!=Clock_IsRinging())
		{
			LastRing=Clock_IsRinging(); PageDirty=1; CellMask4=0xFFFF;
		}
		if(PageDirty==0) return; /* 时间仅显示在顶栏，主体不按秒重画 */
	}
	else if(PageDirty == 0)
	{
		return;                             /* 静态页（主菜单/时间页）没变化就不重画 */
	}

	/* 只重画被标记的字符格 */
	Menu_DrawCellPage();

	/* 刷新各页的"上次显示值"缓存 */
	if(MenuPage == MENU_PAGE_IMU)
	{
		MPU6050_GetGyroDps10(&LastGyroX,&LastGyroY,&LastGyroZ);
	}
	else if(MenuPage == MENU_PAGE_SERVO)
 {
  LastServoAngle=Servo_GetAngle();
  LastHoldMode=(int16_t)(Servo_GetHoldFault() ? 2 : (Servo_GetHold() ? 1 : 0));
  LastHoldAngle10=HoldEditPending ? HoldEditAngle10 : Servo_GetHoldAngle10();
  LastServoY10=Menu_ReadPitch10();LastHoldEdit=HoldEditPending;
  LastHoldFresh=MPU6050_HasFreshSample();
  LastHoldFaultCode=Servo_GetHoldFaultCode();
 }

 if(MenuPage==MENU_PAGE_BRIGHTNESS)
 {
  LastLightAuto=LightSensor_GetAuto();LastLightContrast=LightSensor_GetContrast();
  LastLightManual=LightSensor_GetManual();LastLightADC=LightSensor_GetFiltered();LastLightReady=LightSensor_IsReady();
 }

	/* 本帧画完：清除所有标记 */
	LineDirty = 0;
	CellMask2 = 0;
	CellMask3 = 0;
	CellMask4 = 0;
	PageDirty = 0;
}

/**
  * @brief  当前界面的刷新周期
  * @param  无
  * @retval 毫秒。IMU 页要快一点才看得出数据在动，静态页慢一点省 IIC 时间
  */
uint16_t Menu_GetRefreshMs(void)
{
	if(MenuPage == MENU_PAGE_IMU)
	{
		return 200;                 /* IMU 页每 200ms 一次（配合"值没变不重画"，实际更少） */
	}

	if(MenuPage == MENU_PAGE_SERVO)
	{
		return 150;                 /* 舵机页要能看到定角角度的变化 */
	}

	return 200;
}

/*============================================================================
 * 五、操作任务
 *============================================================================*/

/**
  * @brief  操作任务：主循环每 10ms 调一次，处理按键事件与编码器格数
  * @param  无
  * @retval 无
  */
void Menu_Task(void)
{
	int8_t  Detent;
	int16_t Value;
	uint8_t EvtKey1;
	uint8_t EvtKey2;

	/* 开机动画期间不响应任何按键 */
	if(BootDone == 0)
	{
		return;
	}

	EvtKey1 = Key_GetEvent(KEY_ID_1);
	EvtKey2 = Key_GetEvent(KEY_ID_2);
	Detent  = Encoder_Get();
 /* 待机只改变显示。首次按压/旋转只唤醒，按住时吞掉后续长按与松手事件。 */
 if(Standby)
 {
  if(EvtKey1 || EvtKey2 || Detent || Key_GetPress(KEY_ID_1) || Key_GetPress(KEY_ID_2))
  {WakeRelease=(uint8_t)(Key_GetPress(KEY_ID_1)||Key_GetPress(KEY_ID_2));Menu_Wake();Menu_DrawHeader();Menu_DrawPage();}
  else Menu_DrawStandby();
  return;
 }
 if(WakeRelease)
 {
  LastActivity=Timer_GetTick();
  if(!Key_GetPress(KEY_ID_1) && !Key_GetPress(KEY_ID_2))WakeRelease=0;
  return;
 }
 if(EvtKey1 || EvtKey2 || Detent || Key_GetPress(KEY_ID_1) || Key_GetPress(KEY_ID_2))LastActivity=Timer_GetTick();
 else if(!Clock_IsRinging() && Timer_GetTick()-LastActivity>=MENU_IDLE_MS)
 {
  Standby=1;StandbyTime=0xFFFFFFFFu;OLED_Effects_Invalidate();Menu_DrawStandby();return;
 }


	/* 主菜单：先处理光标，再处理KEY1顺序入口与KEY2选择入口；同一轮的旋转不带入功能参数。 */
	if(MenuPage == MENU_PAGE_MAIN && Detent != 0)
	{
		Value=(int16_t)MainCursor+Detent;
		Value%=MENU_MAIN_ITEM_NUM;
		if(Value<0) Value+=MENU_MAIN_ITEM_NUM;
		if(MainCursor!=(uint8_t)Value)
		{
			MainCursor=(uint8_t)Value;
			if(MainCursor<MainTop) MainTop=MainCursor;
			if(MainCursor>=MainTop+3) MainTop=(uint8_t)(MainCursor-2);
			PageDirty=1; LineDirty=LINE234_DIRTY;
			CellMask2=0xFFFF; CellMask3=0xFFFF; CellMask4=0xFFFF;
		}
		Detent=0;
	}
	if(EvtKey1 == KEY_EVT_SHORT)
	{
		Menu_NextPage(); /* 原有入口：固定顺序，忽略主菜单光标。 */
		Detent=0; EvtKey2=KEY_EVT_NONE;
	}
	else if(MenuPage==MENU_PAGE_MAIN && EvtKey2==KEY_EVT_SHORT && !Clock_IsRinging())
	{
		Menu_EnterPage((uint8_t)(MainCursor+1)); /* 新增入口：主菜单KEY2确认，清除事件避免选中页再操作一次。 */
		Detent=0; EvtKey2=KEY_EVT_NONE;
	}

	/*==================================================================
	 * 二、按当前界面处理各自的按键 / 编码器 / 保存
	 *==================================================================*/
	switch(MenuPage)
	{
		/* ---------------- 主菜单：编码器移动列表光标 ---------------- */
		case MENU_PAGE_MAIN:
			break; /* 光标已在确认之前处理 */

		/* ---------------- 时间设置页 ----------------
		   编码器改的是【编辑副本】，系统时钟照常走；只有长按 KEY2 才写入系统时钟
		   消屏闪：改数值只重画第 2 行；改选择只重画第 3 行 */
		case MENU_PAGE_TIME:
			if(TimeLoaded == 0)                     //首次操作前先把当前时间取进编辑副本
			{
				TimeEditH  = Clock_GetHour();
				TimeEditM  = Clock_GetMinute();
				TimeEditS  = Clock_GetSecond();
				TimeLoaded = 1;
				PageDirty  = 1;
				LineDirty  = LINE234_DIRTY;
			}

			if(EvtKey2 == KEY_EVT_SHORT)            //选 时→分→秒→时
			{
				uint8_t OldCursor = TimeCursor;

				TimeCursor = (uint8_t)((TimeCursor + 1) % 3);
				PageDirty  = 1;
				LineDirty  = LINE3_DIRTY;
				/* 只重画"旧标记"和"新标记"那两个格子，其余 14 格不动 */
				Menu_MarkCell(3,(uint8_t)(1 + OldCursor * 5));
				Menu_MarkCell(3,(uint8_t)(1 + TimeCursor * 5));
			}

			/* 编码器加减当前选中项（只改副本）→ 只重画那两位数字，Edit 标签不动 */
			if(Detent != 0)
			{
				if(TimeCursor == 0)
				{
					Value = (int16_t)TimeEditH + Detent;
					while(Value < 0)   Value += 24;
					while(Value > 23)  Value -= 24;
					TimeEditH = (uint8_t)Value;
					Menu_MarkCellRange(2,1,2);       /* 时 = 第 1~2 列 */
				}
				else if(TimeCursor == 1)
				{
					Value = (int16_t)TimeEditM + Detent;
					while(Value < 0)   Value += 60;
					while(Value > 59)  Value -= 60;
					TimeEditM = (uint8_t)Value;
					Menu_MarkCellRange(2,4,5);       /* 分 = 第 4~5 列 */
				}
				else
				{
					Value = (int16_t)TimeEditS + Detent;
					while(Value < 0)   Value += 60;
					while(Value > 59)  Value -= 60;
					TimeEditS = (uint8_t)Value;
					Menu_MarkCellRange(2,7,8);       /* 秒 = 第 7~8 列 */
				}
				PageDirty = 1;
				LineDirty = LINE2_DIRTY;
			}

			if(EvtKey2 == KEY_EVT_LONG)             //长按 KEY2 保存：这时才写入系统时钟
			{
				Clock_SetTime(TimeEditH,TimeEditM,TimeEditS);
				PageDirty = 1;
				LineDirty = LINE234_DIRTY;
			}
			break;

		/* ---------------- 定时闹钟页 ---------------- */
		case MENU_PAGE_ALARM:
			if(!AlarmLoaded || AlarmEditRevision!=Clock_GetAlarmRevision(AlarmIndex)) Menu_LoadAlarmEditor();
			if(EvtKey2==KEY_EVT_SHORT)
			{
				AlarmCursor=(uint8_t)((AlarmCursor+1)%5);
				PageDirty=1; LineDirty=LINE4_DIRTY; CellMask4=0xFFFF;
			}
			if(Detent!=0)
			{
				if(AlarmCursor==0)
				{
					Value=(int16_t)AlarmIndex-1+Detent;
					Value%=CLOCK_ALARM_NUM;
					if(Value<0) Value+=CLOCK_ALARM_NUM;
					if(AlarmIndex!=(uint8_t)(Value+1))
					{
						AlarmIndex=(uint8_t)(Value+1);
						Menu_LoadAlarmEditor();
					}
				}
				else if(AlarmCursor<4)
				{
					uint8_t *Field=AlarmCursor==1 ? &AlarmEditH : (AlarmCursor==2 ? &AlarmEditM : &AlarmEditS);
					int16_t Limit=AlarmCursor==1 ? 24 : 60;
					Value=(int16_t)*Field+Detent;
					Value%=Limit;
					if(Value<0) Value+=Limit;
					*Field=(uint8_t)Value;
					PageDirty=1; LineDirty=LINE3_DIRTY;
					Menu_MarkCellRange(3,(uint8_t)(1+(AlarmCursor-1)*3),(uint8_t)(2+(AlarmCursor-1)*3));
				}
				else
				{
					AlarmEditEn=!AlarmEditEn;
					PageDirty=1; LineDirty=LINE2_DIRTY; Menu_MarkCellRange(2,11,14);
				}
			}
			if(EvtKey2==KEY_EVT_LONG)
			{
				Clock_SetAlarm(AlarmIndex,AlarmEditH,AlarmEditM,AlarmEditS);
				Clock_SetAlarmEnable(AlarmIndex,AlarmEditEn ? ALARM_ON : ALARM_OFF);
				AlarmEditRevision=Clock_GetAlarmRevision(AlarmIndex);
				Menu_SetDirty();
			}
			break;

		/* ---------------- IMU 数据页：本页无参数 ---------------- */
		case MENU_PAGE_IMU:
			break;

		/* ---------------- 舵机角度页 ---------------- */
  case MENU_PAGE_BRIGHTNESS:
   if(EvtKey2==KEY_EVT_SHORT)
   {LightSensor_SetAuto((uint8_t)!LightSensor_GetAuto());Menu_SetDirty();break;}
   if(!LightSensor_GetAuto() && Detent)
   {
    Value=(int16_t)LightSensor_GetManual()+(int16_t)Detent*8;
    if(Value<8) Value=8;
    if(Value>255) Value=255;
    LightSensor_SetManual((uint8_t)Value);Menu_SetDirty();
   }
   break;

		case MENU_PAGE_SERVO:
   Menu_SyncHoldEditor();
   if(Servo_GetHoldFault())
   {
    if(EvtKey2==KEY_EVT_LONG) {Servo_SetHold(0);HoldEditPending=0;Menu_SetDirty();}
    else if(EvtKey2==KEY_EVT_SHORT)
    {
     Value=Servo_GetHoldAngle10();Servo_SetHold(1);
     if(Servo_GetHold() && !Servo_GetHoldFault()) Servo_SetHoldAngle10(Value);
     Menu_SetDirty();
    }
    break;
   }
   /* 长按开关优先，不能把同次旋转误作手动动作或确认。 */
   if(EvtKey2==KEY_EVT_LONG)
   {
    if(Servo_GetHold()) {Servo_SetHold(0);HoldEditPending=0;}
    else
    {
     Servo_SetHold(1);
     if(Servo_GetHold() && !Servo_GetHoldFault())
     {HoldEditAngle10=0;HoldEditBase10=Servo_GetHoldAngle10();HoldEditPending=1;}
    }
    Menu_SetDirty();break;
   }
   if(Servo_GetHold())
   {
    if(Detent)
    {
     if(!HoldEditPending)
     {
      Value=Servo_GetHoldAngle10();HoldEditBase10=Value;
      HoldEditAngle10=(int16_t)(((Value>=0 ? Value+5 : Value-5)/10)*10);
     }
     Value=(int16_t)(HoldEditAngle10+(int16_t)Detent*10);
     if(Value>300) Value=300;
     if(Value< -300) Value=-300;
     HoldEditAngle10=Value;HoldEditPending=1;Menu_SetDirty();
    }
    if(EvtKey2==KEY_EVT_SHORT && HoldEditPending)
    {
     Servo_SetHoldAngle10(HoldEditAngle10);HoldEditPending=0;Menu_SetDirty();
    }
   }
   else if(Detent)
   {
    Value=(int16_t)Servo_GetAngle()+Detent;
    while(Value<0) Value+=181;
    while(Value>180) Value-=181;
    Servo_SetAngle((uint8_t)Value);Menu_SetDirty();
   }
   break;

		default:
			break;
	}

	/*==================================================================
	 * 三、绘制
	 *==================================================================*/
	Menu_DrawHeader();

	if(PageDirty == 1 || Menu_NeedRedraw() == 1)
	{
		Menu_DrawPage();
	}
}
