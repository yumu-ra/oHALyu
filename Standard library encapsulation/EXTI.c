#include "stm32f10x.h"
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
                 uint8_t priority, uint8_t enable) {
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint8_t exti_port_source;
    uint8_t exti_pin_source;
    IRQn_Type irq_channel;
    
    // 确定GPIO端口源
    if(gpio_port == GPIOA) exti_port_source = GPIO_PortSourceGPIOA;
    else if(gpio_port == GPIOB) exti_port_source = GPIO_PortSourceGPIOB;
    else if(gpio_port == GPIOC) exti_port_source = GPIO_PortSourceGPIOC;
    else if(gpio_port == GPIOD) exti_port_source = GPIO_PortSourceGPIOD;
    else if(gpio_port == GPIOE) exti_port_source = GPIO_PortSourceGPIOE;
    else if(gpio_port == GPIOF) exti_port_source = GPIO_PortSourceGPIOF;
    else if(gpio_port == GPIOG) exti_port_source = GPIO_PortSourceGPIOG;
    else return;
    
    // 确定GPIO引脚源和EXTI线
    if(gpio_pin == GPIO_Pin_0) {
        exti_pin_source = GPIO_PinSource0;
        irq_channel = EXTI0_IRQn;
    } else if(gpio_pin == GPIO_Pin_1) {
        exti_pin_source = GPIO_PinSource1;
        irq_channel = EXTI1_IRQn;
    } else if(gpio_pin == GPIO_Pin_2) {
        exti_pin_source = GPIO_PinSource2;
        irq_channel = EXTI2_IRQn;
    } else if(gpio_pin == GPIO_Pin_3) {
        exti_pin_source = GPIO_PinSource3;
        irq_channel = EXTI3_IRQn;
    } else if(gpio_pin == GPIO_Pin_4) {
        exti_pin_source = GPIO_PinSource4;
        irq_channel = EXTI4_IRQn;
    } else if(gpio_pin == GPIO_Pin_5) {
        exti_pin_source = GPIO_PinSource5;
        irq_channel = EXTI9_5_IRQn;
    } else if(gpio_pin == GPIO_Pin_6) {
        exti_pin_source = GPIO_PinSource6;
        irq_channel = EXTI9_5_IRQn;
    } else if(gpio_pin == GPIO_Pin_7) {
        exti_pin_source = GPIO_PinSource7;
        irq_channel = EXTI9_5_IRQn;
    } else if(gpio_pin == GPIO_Pin_8) {
        exti_pin_source = GPIO_PinSource8;
        irq_channel = EXTI9_5_IRQn;
    } else if(gpio_pin == GPIO_Pin_9) {
        exti_pin_source = GPIO_PinSource9;
        irq_channel = EXTI9_5_IRQn;
    } else if(gpio_pin == GPIO_Pin_10) {
        exti_pin_source = GPIO_PinSource10;
        irq_channel = EXTI15_10_IRQn;
    } else if(gpio_pin == GPIO_Pin_11) {
        exti_pin_source = GPIO_PinSource11;
        irq_channel = EXTI15_10_IRQn;
    } else if(gpio_pin == GPIO_Pin_12) {
        exti_pin_source = GPIO_PinSource12;
        irq_channel = EXTI15_10_IRQn;
    } else if(gpio_pin == GPIO_Pin_13) {
        exti_pin_source = GPIO_PinSource13;
        irq_channel = EXTI15_10_IRQn;
    } else if(gpio_pin == GPIO_Pin_14) {
        exti_pin_source = GPIO_PinSource14;
        irq_channel = EXTI15_10_IRQn;
    } else if(gpio_pin == GPIO_Pin_15) {
        exti_pin_source = GPIO_PinSource15;
        irq_channel = EXTI15_10_IRQn;
    } else {
        return; // 不支持的引脚
    }
    
    // 映射GPIO引脚到EXTI线
    GPIO_EXTILineConfig(exti_port_source, exti_pin_source);
    
    // EXTI配置
    EXTI_InitStructure.EXTI_Line = 1 << exti_pin_source; // 对应EXTI线
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = trigger_mode;
    EXTI_InitStructure.EXTI_LineCmd = (enable) ? ENABLE : DISABLE;
    EXTI_Init(&EXTI_InitStructure);
    
    // NVIC配置（只有在启用中断时才配置NVIC）
    if(enable) {
        NVIC_InitStructure.NVIC_IRQChannel = irq_channel;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
}

// 使用示例
void example_usage(void) {
    // 示例1: 配置PB0和PB1为下降沿触发中断
    EXTI_Line_Config(GPIOB, GPIO_Pin_0, EXTI_Trigger_Falling, 0);
    EXTI_Line_Config(GPIOB, GPIO_Pin_1, EXTI_Trigger_Falling, 0);
    
    // 示例2: 批量配置多个EXTI
    GPIO_TypeDef* ports[] = {GPIOB, GPIOB, GPIOC};
    uint16_t pins[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2};
    uint32_t triggers[] = {EXTI_Trigger_Falling, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling};
    uint8_t priorities[] = {0, 1, 2};
    
    EXTI_Multi_Config(ports, pins, triggers, priorities, 3);
    
    // 示例3: 使用通用函数配置
    EXTI_Config(GPIOB, GPIO_Pin_0, EXTI_Trigger_Falling, 0, 1); // 启用中断
    EXTI_Config(GPIOB, GPIO_Pin_1, EXTI_Trigger_Falling, 0, 1); // 启用中断
    EXTI_Config(GPIOC, GPIO_Pin_3, EXTI_Trigger_Rising, 1, 0);  // 不启用中断
}

// 中断服务函数示例
// void EXTI0_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
//         // 用户代码
//         EXTI_ClearITPendingBit(EXTI_Line0);
//     }
// }
// 
// void EXTI1_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
//         // 用户代码
//         EXTI_ClearITPendingBit(EXTI_Line1);
//     }
// }