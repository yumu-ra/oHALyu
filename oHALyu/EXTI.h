#ifndef __EXTI_H
#define __EXTI_H
#include "stm32f10x.h"                  // Device header
/**
 * @brief  通用EXTI配置函数（简化版）
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @param  trigger_mode: 触发模式
 * @param  priority: 中断优先级
 * @param  enable: 是否启用中断 (1=启用, 0=禁用)
 * @retval None
 */
void EXTI_Config(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, uint32_t trigger_mode, 
                 uint8_t priority, uint8_t enable);

#endif
