#ifndef __SENSER_H
#define __SENSER_H
#include "stm32f10x.h"                  // Device header

void Sensor_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, GPIOMode_TypeDef input_mode, 
                 EXTITrigger_TypeDef trigger_mode, uint8_t priority) ;

uint8_t Sensor_Read(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) ;
#endif
