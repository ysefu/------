#include "stm32f10x.h"                  // Device header
#include "OLED.h"
#include "OLED_Chinese.h"
#include "OLED_ChineseFont.h"
/*----------------------------------------------------------------------------
 * 文件：OLED_Chinese.c / OLED_Chinese.h
 * 用处：16x16中文字库、中英文混排、度符号和字符范围检查。
 * 对应外设：PB8/PB9软件IIC；对应器件：128x64 OLED。
 * 备注：复用原驱动写屏路径，汉字成对刷新，避免只更新半个字。
 *----------------------------------------------------------------------------*/
/* 原驱动已有的非static函数；只复用写屏路径，不改IIC时序。 */
void OLED_SetCursor(uint8_t Y, uint8_t X);
void OLED_WriteData(uint8_t Data);
static const uint8_t Text_MAIN[] = {130,255,178,255,139,255};
static const uint8_t Text_TIME[] = {159,255,187,255};
static const uint8_t Text_ALARM[] = {188,255,184,255};
static const uint8_t Text_IMU[] = {143,255,150,255};
static const uint8_t Text_SERVO[] = {176,255,161,255};
static const uint8_t Text_TIME_SET[] = {159,255,187,255,181,255,175,255};
static const uint8_t Text_ALARM_SET[] = {145,255,159,255,188,255,184,255};
static const uint8_t Text_IMU_DATA[] = {143,255,150,255,158,255,155,255};
static const uint8_t Text_SERVO_ANGLE[] = {176,255,161,255,179,255,147,255};
static const uint8_t Text_NEXT[] = {75,49,129,255,128,255,191,255};
static const uint8_t Text_EDIT[] = {174,255,183,255};
static const uint8_t Text_HOUR[] = {159,255};
static const uint8_t Text_MINUTE[] = {133,255};
static const uint8_t Text_SECOND[] = {169,255};
static const uint8_t Text_SAVE[] = {75,50,185,255,154,255,131,255,144,255};
static const uint8_t Text_CURRENT[] = {149,255,135,255};
static const uint8_t Text_TARGET[] = {167,255,163,255};
static const uint8_t Text_SWITCH[] = {148,255,132,255};
static const uint8_t Text_ON[] = {148,255,140,255};
static const uint8_t Text_OFF[] = {132,255,186,255};
static const uint8_t Text_RING[] = {152,255,180,255};
static const uint8_t Text_IMU_FAULT[] = {143,255,150,255,157,255,190,255};
static const uint8_t Text_CHECK[] = {164,255,162,255,156,255,171,255};
static const uint8_t Text_ANGLE[] = {179,255,147,255};
static const uint8_t Text_HOLD[] = {145,255,179,255};
static const uint8_t Text_FAULT[] = {157,255,190,255};
static const uint8_t Text_CLEAR[] = {75,50,166,255,189,255};
static const uint8_t Text_MANUAL[] = {151,255,137,255};
static const uint8_t Text_SHORT_OFF[] = {132,255};
static const uint8_t Text_SHORT_ON[] = {148,255};
static const uint8_t Text_LONG_ON[] = {185,255,154,255,148,255};
static const uint8_t Text_KEEP[] = {131,255,153,255};
static const uint8_t Text_TITLE[] = {101,177,255,50,48,50,54};
static const uint8_t Text_INIT[] = {170,255,172,255,134,255,142,255,138,255};
static const uint8_t Text_OLED_READY[] = {160,255,168,255,146,255,173,255};
static const uint8_t Text_IMU_READY[] = {143,255,150,255,146,255,173,255};
static const uint8_t Text_LOADING[] = {165,255,141,255,136,255,182,255};
static const uint8_t Text_APPLY[]={75,50,131,255,144,255};
static const uint8_t Text_LONG_OFF[]={185,255,154,255,132,255};
static const uint8_t Text_BRIGHT[] = {192,255,147,255};
static const uint8_t Text_BRIGHT_SET[] = {192,255,147,255,181,255,175,255};
static const uint8_t Text_AUTO[] = {197,255,137,255};
static const uint8_t Text_CHANGE[] = {194,255,195,255};
static const uint8_t Text_LIGHT[] = {193,255,196,255};
static const uint8_t Text_SET_VALUE[] = {181,255,145,255};

typedef struct {const uint8_t *Data; uint8_t Length;}ChinesePhrase;
static const ChinesePhrase Phrases[CN_TEXT_COUNT] = {
	{Text_MAIN,sizeof(Text_MAIN)},
	{Text_TIME,sizeof(Text_TIME)},
	{Text_ALARM,sizeof(Text_ALARM)},
	{Text_IMU,sizeof(Text_IMU)},
	{Text_SERVO,sizeof(Text_SERVO)},
	{Text_TIME_SET,sizeof(Text_TIME_SET)},
	{Text_ALARM_SET,sizeof(Text_ALARM_SET)},
	{Text_IMU_DATA,sizeof(Text_IMU_DATA)},
	{Text_SERVO_ANGLE,sizeof(Text_SERVO_ANGLE)},
	{Text_NEXT,sizeof(Text_NEXT)},
	{Text_EDIT,sizeof(Text_EDIT)},
	{Text_HOUR,sizeof(Text_HOUR)},
	{Text_MINUTE,sizeof(Text_MINUTE)},
	{Text_SECOND,sizeof(Text_SECOND)},
	{Text_SAVE,sizeof(Text_SAVE)},
	{Text_CURRENT,sizeof(Text_CURRENT)},
	{Text_TARGET,sizeof(Text_TARGET)},
	{Text_SWITCH,sizeof(Text_SWITCH)},
	{Text_ON,sizeof(Text_ON)},
	{Text_OFF,sizeof(Text_OFF)},
	{Text_RING,sizeof(Text_RING)},
	{Text_IMU_FAULT,sizeof(Text_IMU_FAULT)},
	{Text_CHECK,sizeof(Text_CHECK)},
	{Text_ANGLE,sizeof(Text_ANGLE)},
	{Text_HOLD,sizeof(Text_HOLD)},
	{Text_FAULT,sizeof(Text_FAULT)},
	{Text_CLEAR,sizeof(Text_CLEAR)},
	{Text_MANUAL,sizeof(Text_MANUAL)},
	{Text_SHORT_OFF,sizeof(Text_SHORT_OFF)},
	{Text_SHORT_ON,sizeof(Text_SHORT_ON)},
	{Text_LONG_ON,sizeof(Text_LONG_ON)},
	{Text_KEEP,sizeof(Text_KEEP)},
	{Text_TITLE,sizeof(Text_TITLE)},
	{Text_INIT,sizeof(Text_INIT)},
	{Text_OLED_READY,sizeof(Text_OLED_READY)},
	{Text_IMU_READY,sizeof(Text_IMU_READY)},
	{Text_LOADING,sizeof(Text_LOADING)},
 {Text_APPLY,sizeof(Text_APPLY)},
 {Text_LONG_OFF,sizeof(Text_LONG_OFF)},
 {Text_BRIGHT,sizeof(Text_BRIGHT)},
 {Text_BRIGHT_SET,sizeof(Text_BRIGHT_SET)},
 {Text_AUTO,sizeof(Text_AUTO)},
 {Text_CHANGE,sizeof(Text_CHANGE)},
 {Text_LIGHT,sizeof(Text_LIGHT)},
 {Text_SET_VALUE,sizeof(Text_SET_VALUE)},
};

uint8_t OLED_Chinese_PutText(char *Buf, uint8_t Column, OLED_ChineseText Text)
{
 uint8_t i,Length;
 if(Buf == 0 || (unsigned int)Text >= CN_TEXT_COUNT) return Column;
 Length=Phrases[Text].Length;
 if(Column>16 || Length>16-Column) return Column;
 for(i=0;i<Length;i++) Buf[Column+i]=(char)Phrases[Text].Data[i];
 return (uint8_t)(Column+Length);
}
void OLED_Chinese_PutDegree(char *Buf, uint8_t Column)
{
 if(Buf != 0 && Column<16) Buf[Column]=(char)0xFE;
}
void OLED_Chinese_DrawCells(uint8_t Line, const char *Buf, uint16_t Mask, uint16_t Inverse)
{
 uint8_t i;
 if(Line<1 || Line>4 || Buf==0) return;
 for(i=0;i<16;i++)
 {
  uint8_t Code=(uint8_t)Buf[i];
  if(Code==0xFF) continue;
  if(Code>=0x80 && Code<0xFE)
  {
   uint8_t Glyph=(uint8_t)(Code-0x80),Page,x;
   uint16_t Pair=(uint16_t)(3u<<i);
   if(i>=15 || (uint8_t)Buf[i+1]!=0xFF || Glyph>=sizeof(OLED_ChineseFont)/32u) continue;
   if(Mask & Pair)
   {
    for(Page=0;Page<2;Page++)
    {
     OLED_SetCursor((uint8_t)((Line-1)*2+Page),(uint8_t)(i*8));
     for(x=0;x<16;x++)
     {
      uint8_t Pixel=OLED_ChineseFont[Glyph][Page*16+x];
      OLED_WriteData((Inverse & Pair) ? (uint8_t)~Pixel : Pixel);
     }
    }
   }
   i++;
  }
  else if(Mask & (uint16_t)(1u<<i))
  {
   if(Code==0xFE)
   {
    uint8_t Page,x;
    for(Page=0;Page<2;Page++)
    {
     OLED_SetCursor((uint8_t)((Line-1)*2+Page),(uint8_t)(i*8));
     for(x=0;x<8;x++)
     {
      uint8_t Pixel=0;
      if(Page==0) Pixel=(x==2 || x==5) ? 0x0C : ((x==3 || x==4) ? 0x12 : 0);
      OLED_WriteData(Pixel);
     }
    }
   }
   else OLED_ShowChar(Line,(uint8_t)(i+1),(Code>=32 && Code<=126) ? (char)Code : ' ');
  }
 }
}
void OLED_Chinese_ClearLine(uint8_t Line)
{
 char Buf[16]; uint8_t i;
 for(i=0;i<16;i++) Buf[i]=' ';
 OLED_Chinese_DrawCells(Line,Buf,0xFFFF,0);
}
void OLED_Chinese_ShowText(uint8_t Line, uint8_t Column, OLED_ChineseText Text)
{
 char Buf[16]; uint8_t i,End; uint16_t Mask=0;
 if(Column<1 || Column>16) return;
 for(i=0;i<16;i++) Buf[i]=' ';
 End=OLED_Chinese_PutText(Buf,(uint8_t)(Column-1),Text);
 for(i=(uint8_t)(Column-1);i<End;i++) Mask|=(uint16_t)(1u<<i);
 OLED_Chinese_DrawCells(Line,Buf,Mask,0);
}
