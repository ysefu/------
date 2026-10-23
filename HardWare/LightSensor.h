#ifndef __LIGHT_SENSOR_H
#define __LIGHT_SENSOR_H
#include "stm32f10x.h"

/* 光敏模块VCC=3.3V、GND共地、AO=PA0/ADC1_IN0，DO不接。
 * 所有接口仅由主循环调用；不占用定时器、不在中断中写OLED。 */
void LightSensor_Init(void);
void LightSensor_Update(void);
uint16_t LightSensor_GetRaw(void);
uint16_t LightSensor_GetFiltered(void);
uint8_t LightSensor_GetContrast(void);
uint8_t LightSensor_IsReady(void);
/* 0=光越强AO越低（默认），1=光越强AO越高。 */
void LightSensor_SetDirection(uint8_t HighIsBright);
uint8_t LightSensor_GetDirection(void);

/* 自动模式默认开启；手动测试不停止ADC采样，8~255级渐变到位。 */
void LightSensor_SetAuto(uint8_t Enable);
uint8_t LightSensor_GetAuto(void);
uint8_t LightSensor_GetManual(void);
void LightSensor_SetManual(uint8_t Level);
#endif
