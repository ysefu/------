#include "stm32f10x.h"
#include "bsp_pin.h"
#include "Timer.h"
#include "OLED.h"
#include "LightSensor.h"

#define LIGHT_PERIOD_MS       50u
#define LIGHT_STALE_MS        500u
#define LIGHT_MIN_CONTRAST    8u
#define LIGHT_MAX_CONTRAST    255u
#define LIGHT_DEFAULT         128u
/* 用户实测：正常RAW1629/滤波1630，遮挡RAW3000/滤波3020。 */
#define LIGHT_ADC_LOW         1600u
#define LIGHT_ADC_HIGH        3000u
#define LIGHT_FADE_STEP       4u
#define LIGHT_DEADBAND        4u

static uint8_t HwReady=0,HasSample=0,HighIsBright=0;
static uint8_t Contrast=LIGHT_DEFAULT,AutoOn=1,ManualTarget=LIGHT_DEFAULT;
static uint16_t Raw=0;
static int32_t FilterQ8=0;
static uint32_t LastUpdate=0,LastSample=0;

/* 只在开机初始化时校准ADC，最长每步10ms且有循环次数上限。
 * 连续转换在硬件后台运行；周期任务仅读取已完成的样本。 */
static uint8_t Light_ADCInit(void)
{
 GPIO_InitTypeDef Gpio;
 ADC_InitTypeDef Adc;
 uint32_t Start,Limit;
 volatile uint32_t Stabilize;
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_ADC1,ENABLE);
 RCC_ADCCLKConfig(RCC_PCLK2_Div6); /* 72MHz/6=12MHz，低于14MHz上限。 */
 Gpio.GPIO_Pin=LIGHT_AO_PIN;Gpio.GPIO_Mode=GPIO_Mode_AIN;Gpio.GPIO_Speed=GPIO_Speed_50MHz;
 GPIO_Init(LIGHT_AO_PORT,&Gpio);
 ADC_StructInit(&Adc);
 Adc.ADC_Mode=ADC_Mode_Independent;Adc.ADC_ScanConvMode=DISABLE;
 Adc.ADC_ContinuousConvMode=ENABLE;Adc.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None;
 Adc.ADC_DataAlign=ADC_DataAlign_Right;Adc.ADC_NbrOfChannel=1;
 ADC_Init(LIGHT_ADC,&Adc);
 ADC_RegularChannelConfig(LIGHT_ADC,LIGHT_ADC_CHANNEL,1,ADC_SampleTime_239Cycles5);
 ADC_Cmd(LIGHT_ADC,ENABLE);
 for(Stabilize=0;Stabilize<128u;Stabilize++) {} /* ADC上电稳定时间，主动画开始前。 */
 ADC_ResetCalibration(LIGHT_ADC);Start=Timer_GetTick();Limit=0;
 while(ADC_GetResetCalibrationStatus(LIGHT_ADC)!=RESET)
 {
  if(Timer_GetTick()-Start>=10u || ++Limit>=100000u) {ADC_Cmd(LIGHT_ADC,DISABLE);return 0;}
 }
 ADC_StartCalibration(LIGHT_ADC);Start=Timer_GetTick();Limit=0;
 while(ADC_GetCalibrationStatus(LIGHT_ADC)!=RESET)
 {
  if(Timer_GetTick()-Start>=10u || ++Limit>=100000u) {ADC_Cmd(LIGHT_ADC,DISABLE);return 0;}
 }
 ADC_SoftwareStartConvCmd(LIGHT_ADC,ENABLE);
 return 1;
}
static uint8_t Light_ReadADC(uint16_t *Value)
{
 if(ADC_GetFlagStatus(LIGHT_ADC,ADC_FLAG_EOC)==RESET) return 0;
 *Value=ADC_GetConversionValue(LIGHT_ADC); /* 读取DR自动清除EOC，不等待下一次转换。 */
 return *Value<=4095u;
}
void LightSensor_Init(void)
{
 HasSample=0;Raw=0;FilterQ8=0;HighIsBright=0;Contrast=LIGHT_DEFAULT;AutoOn=1;ManualTarget=LIGHT_DEFAULT;
 LastUpdate=Timer_GetTick()-LIGHT_PERIOD_MS;LastSample=Timer_GetTick();
 HwReady=Light_ADCInit();
 OLED_SetContrast(Contrast); /* 保持可读，不因未接模块而主动关屏。 */
}
void LightSensor_Update(void)
{
 uint32_t Now=Timer_GetTick();
 uint16_t Value,Filtered,Bright,Target;
 uint8_t Threshold;
 if(Now-LastUpdate<LIGHT_PERIOD_MS) return;
 LastUpdate=Now;
 if(HwReady && Light_ReadADC(&Value))
 {
  Raw=Value;
  if(!HasSample || Now-LastSample>=LIGHT_STALE_MS) FilterQ8=(int32_t)Value*256;
  else FilterQ8+=((int32_t)Value*256-FilterQ8)/8; /* 约0.4秒平滑，保留小数避免量化卡住。 */
  LastSample=Now;HasSample=1;
 }
 Target=LIGHT_DEFAULT;
 if(AutoOn && HwReady && HasSample && Now-LastSample<LIGHT_STALE_MS)
 {
  Filtered=(uint16_t)((FilterQ8+128)/256);
  if(Filtered<LIGHT_ADC_LOW) Filtered=LIGHT_ADC_LOW;
  if(Filtered>LIGHT_ADC_HIGH) Filtered=LIGHT_ADC_HIGH;
  Bright=HighIsBright ? (uint16_t)(Filtered-LIGHT_ADC_LOW) : (uint16_t)(LIGHT_ADC_HIGH-Filtered);
  Target=(uint16_t)(LIGHT_MIN_CONTRAST+((uint32_t)Bright*(LIGHT_MAX_CONTRAST-LIGHT_MIN_CONTRAST)+(LIGHT_ADC_HIGH-LIGHT_ADC_LOW)/2u)/(LIGHT_ADC_HIGH-LIGHT_ADC_LOW));
 }
 if(!AutoOn) Target=ManualTarget;
 Threshold=AutoOn ? LIGHT_DEADBAND : 1u; /* 手动测试必须精确到所选值。 */
 /* 自动迟滞忽略微小变化，每50ms最多4级，覆盖整个亮度区间约3.1秒。 */
 if(Target>Contrast && Target-Contrast>=Threshold)
 {
  uint16_t Delta=Target-Contrast;
  Contrast=(uint8_t)(Contrast+(Delta>LIGHT_FADE_STEP ? LIGHT_FADE_STEP : Delta));
  OLED_SetContrast(Contrast);
 }
 else if(Target<Contrast && Contrast-Target>=Threshold)
 {
  uint16_t Delta=Contrast-Target;
  Contrast=(uint8_t)(Contrast-(Delta>LIGHT_FADE_STEP ? LIGHT_FADE_STEP : Delta));
  OLED_SetContrast(Contrast);
 }
}
uint16_t LightSensor_GetRaw(void) {return Raw;}
uint16_t LightSensor_GetFiltered(void) {return (uint16_t)((FilterQ8+128)/256);}
uint8_t LightSensor_GetContrast(void) {return Contrast;}
uint8_t LightSensor_IsReady(void) {return HwReady && HasSample && Timer_GetTick()-LastSample<LIGHT_STALE_MS;}
void LightSensor_SetDirection(uint8_t High) {HighIsBright=High ? 1 : 0;}
uint8_t LightSensor_GetDirection(void) {return HighIsBright;}

void LightSensor_SetAuto(uint8_t Enable)
{
 AutoOn=Enable ? 1 : 0;
 if(!AutoOn) ManualTarget=Contrast;
}
uint8_t LightSensor_GetAuto(void) {return AutoOn;}
void LightSensor_SetManual(uint8_t Level)
{
 ManualTarget=Level<LIGHT_MIN_CONTRAST ? LIGHT_MIN_CONTRAST : Level;
 AutoOn=0;
}

uint8_t LightSensor_GetManual(void) {return ManualTarget;}
