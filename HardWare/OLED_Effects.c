#include "OLED_Effects.h"
#include "OLED_BootAssets.h"
#include <string.h>
/* 保留OLED原有底层时序；此模块只生成128x64像素内容并按变化字节刷新。 */
void OLED_SetCursor(uint8_t Y,uint8_t X);
void OLED_WriteData(uint8_t Data);
static uint8_t Frame[1024],Previous[1024],Valid=0;
void OLED_Effects_Invalidate(void){Valid=0;}
static void Flush(void)
{
 uint16_t i;uint8_t p,x,end,last;
 for(p=0;p<8;p++)
 {
  x=0;
  while(x<128)
  {
   i=(uint16_t)p*128+x;
   if(Valid && Frame[i]==Previous[i]){x++;continue;}
   /* 合并间隔1~2字节的变化段，少发三条定位命令；保持原IIC时序。 */
   last=x;end=(uint8_t)(x+1);
   while(end<128)
   {
    i=(uint16_t)p*128+end;
    if(!Valid || Frame[i]!=Previous[i])last=end;
    else if(end-last>=3)break;
    end++;
   }
   OLED_SetCursor(p,x);
   do {i=(uint16_t)p*128+x;OLED_WriteData(Frame[i]);Previous[i]=Frame[i];x++;}while(x<=last);
  }
 }
 Valid=1;
}
static void Pixel(int X,int Y)
{
 if(X>=0 && X<128 && Y>=0 && Y<64) Frame[(Y/8)*128+X]|=(uint8_t)(1u<<(Y%8));
}
static void Line(int X1,int X2,int Y){int x;for(x=X1;x<=X2;x++)Pixel(x,Y);}
/* 5x7列字模，仅绘制参考图的TIME/24H/HOUR MIN SEC。 */
static const char Letters[]="TIME24HOURNSC ";
static const uint8_t Small[][5]={
 {1,1,127,1,1},{0,65,127,65,0},{127,2,12,2,127},{127,73,73,73,65},
 {66,97,81,73,70},{24,20,18,127,16},{127,8,8,8,127},{62,65,65,65,62},
 {63,64,64,64,63},{127,9,25,41,70},{127,4,8,16,127},{70,73,73,73,49},
 {62,65,65,65,34},{0,0,0,0,0}};
static void Text(int X,int Y,const char *S)
{
 int i,x,y;while(*S){for(i=0;Letters[i] && Letters[i]!=*S;i++);if(!Letters[i])i=13;
 for(x=0;x<5;x++)for(y=0;y<7;y++)if(Small[i][x]&(1u<<y))Pixel(X+x,Y+y);
 X+=6;S++;}
}
/* 斜体七段数码：含斜切端头，适配参考图六位大字。 */
static const uint8_t Segments[10]={0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};
static void Digit(int X,uint8_t N)
{
 int x,y;uint8_t mask=Segments[N%10],on;
 for(y=0;y<34;y++)for(x=0;x<15;x++)
 {
  on=0;
  if((mask&1) && y<4 && x>=2 && x<=12 && x>=3-y && x<=11+y)on=1;
  if((mask&64) && y>=15 && y<=18 && x>=2 && x<=12)on=1;
  if((mask&8) && y>=30 && x>=2 && x<=12 && x>=y-30 && x<=44-y)on=1;
  if((mask&32) && x<4 && y>=3 && y<=15 && y>=6-x)on=1;
  if((mask&2) && x>=11 && y>=3 && y<=15 && y>=x-9)on=1;
  if((mask&16) && x<4 && y>=18 && y<=30 && y<=27+x)on=1;
  if((mask&4) && x>=11 && y>=18 && y<=30 && y<=41-x)on=1;
  if(on)Pixel(X+x+(33-y)/11,18+y);
 }
}
void OLED_Effects_Boot(uint8_t Index)
{
 uint16_t pos,out=0;uint8_t count,tag,value;
 if(Index>=OLED_EFFECTS_BOOT_FRAMES)Index=OLED_EFFECTS_BOOT_FRAMES-1;
 pos=BootOffsets[Index];
 while(out<1024)
 {
  tag=BootPacked[pos++];count=(uint8_t)((tag&127u)+1u);
  if(tag&128u){value=BootPacked[pos++];while(count-- && out<1024)Frame[out++]=value;}
  else {while(count-- && out<1024)Frame[out++]=BootPacked[pos++];}
 }
 Flush();
}
void OLED_Effects_Clock(uint8_t Hour,uint8_t Minute,uint8_t Second)
{
 int y,x;memset(Frame,0,sizeof(Frame));
 Text(4,3,"TIME");Line(32,98,6);Text(106,3,"24H");
 Digit(0,Hour/10);Digit(19,Hour%10);Digit(45,Minute/10);Digit(64,Minute%10);Digit(90,Second/10);Digit(109,Second%10);
 for(x=40;x<=85;x+=45)for(y=28;y<=42;y+=14){Pixel(x,y);Pixel(x-1,y);Pixel(x+1,y);Pixel(x,y-1);Pixel(x,y+1);}
 Text(28,56,"HOUR MIN SEC");Line(4,23,59);Line(104,123,59);Flush();
}
