#ifndef __TIM_H
#define __TIM_H
#include "stm32f10x.h"                  // Device header

void Timer_Init(TIM_TypeDef* timer, uint16_t period, uint16_t prescaler, 
                uint8_t interrupt_enable, uint8_t priority);
uint8_t Timer_Init_ByFreq(TIM_TypeDef* timer, float target_freq, uint32_t timer_freq, 
                          uint8_t interrupt_enable, uint8_t priority);
uint8_t Timer_Init_ByTime(TIM_TypeDef* timer, uint32_t time_ms, 
                          uint8_t interrupt_enable, uint8_t priority);

#endif
