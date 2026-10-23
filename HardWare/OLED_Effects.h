#ifndef OLED_EFFECTS_H
#define OLED_EFFECTS_H
#include "stm32f10x.h"
#define OLED_EFFECTS_BOOT_FRAMES 64u
void OLED_Effects_Invalidate(void);
void OLED_Effects_Boot(uint8_t Frame);
void OLED_Effects_Clock(uint8_t Hour,uint8_t Minute,uint8_t Second);
#endif
