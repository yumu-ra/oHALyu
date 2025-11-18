#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f10x.h"
// 蜂鸣器控制函数
/**
 * @brief  蜂鸣器初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pins: GPIO引脚 (可以是多个引脚的或运算)
 * @param  speed: GPIO速度 (GPIO_Speed_2MHz, GPIO_Speed_10MHz, GPIO_Speed_50MHz)
 * @retval None
 */
void Buzzer_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pins, GPIOSpeed_TypeDef speed);
void Buzzer_ON(GPIO_TypeDef* gpio_port, uint16_t gpio_pins);
void Buzzer_OFF(GPIO_TypeDef* gpio_port, uint16_t gpio_pins);
/**
 * @brief  蜂鸣器状态翻转函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void Buzzer_Turn(GPIO_TypeDef* gpio_port, uint16_t gpio_pins);
/**
 * @brief  读取蜂鸣器状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚 (单个引脚)
 * @retval 1: 开启, 0: 关闭
 */
uint8_t Buzzer_Get(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) ;
#endif
