#ifndef __LED_H
#define __LED_H
#include "stm32f10x.h"                  // Device header

void LED_Init(void);
void LED_ON(uint16_t num);
void LED_OFF(uint16_t num);
void LED_Turn(uint16_t num);

#endif
