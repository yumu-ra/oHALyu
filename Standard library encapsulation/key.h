#ifndef __KEY_H
#define __KEY_H
#include "stm32f10x.h"                  // Device header


/**
 * @brief  按钮EXTI初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pin: GPIO引脚 (GPIO_Pin_0 到 GPIO_Pin_15)
 * @param  pull_mode: 上拉/下拉模式 (GPIO_Mode_IPU=上拉, GPIO_Mode_IPD=下拉, GPIO_Mode_IN_FLOATING=浮空)
 * @param  trigger_mode: 触发模式 (EXTI_Trigger_Rising, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling)
 * @param  priority: 中断优先级 (0-15)
 * @retval None
 */
void Button_EXTI_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, GPIOMode_TypeDef pull_mode, 
                      EXTITrigger_TypeDef trigger_mode, uint8_t priority);
/**
 * @brief  按钮GPIO初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pin: GPIO引脚 (GPIO_Pin_0 到 GPIO_Pin_15)
 * @param  pull_mode: 上拉/下拉模式 (GPIO_Mode_IPU=上拉, GPIO_Mode_IPD=下拉, GPIO_Mode_IN_FLOATING=浮空)
 * @retval None
 */
void Button_GPIO_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, GPIOMode_TypeDef pull_mode);
/**
 * @brief  读取按钮状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @retval 1: 按下, 0: 未按下
 */
uint8_t Button_Read(GPIO_TypeDef* gpio_port, uint16_t gpio_pin);
#endif
