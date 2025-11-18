#include "stm32f10x.h"

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
                      EXTITrigger_TypeDef trigger_mode, uint8_t priority) {
    // 开启时钟
    if(gpio_port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    else if(gpio_port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    else if(gpio_port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    else if(gpio_port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    else if(gpio_port == GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE);
    else if(gpio_port == GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO, ENABLE);
    else if(gpio_port == GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
    
    // GPIO初始化
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = gpio_pin;
    GPIO_InitStruct.GPIO_Mode = pull_mode;  // 上拉/下拉/浮空输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio_port, &GPIO_InitStruct);
    
    // 确定GPIO端口源和引脚源
    uint8_t exti_port_source;
    if(gpio_port == GPIOA) exti_port_source = GPIO_PortSourceGPIOA;
    else if(gpio_port == GPIOB) exti_port_source = GPIO_PortSourceGPIOB;
    else if(gpio_port == GPIOC) exti_port_source = GPIO_PortSourceGPIOC;
    else if(gpio_port == GPIOD) exti_port_source = GPIO_PortSourceGPIOD;
    else if(gpio_port == GPIOE) exti_port_source = GPIO_PortSourceGPIOE;
    else if(gpio_port == GPIOF) exti_port_source = GPIO_PortSourceGPIOF;
    else if(gpio_port == GPIOG) exti_port_source = GPIO_PortSourceGPIOG;
    else return;
    
    uint8_t exti_pin_source;
    if(gpio_pin == GPIO_Pin_0) exti_pin_source = GPIO_PinSource0;
    else if(gpio_pin == GPIO_Pin_1) exti_pin_source = GPIO_PinSource1;
    else if(gpio_pin == GPIO_Pin_2) exti_pin_source = GPIO_PinSource2;
    else if(gpio_pin == GPIO_Pin_3) exti_pin_source = GPIO_PinSource3;
    else if(gpio_pin == GPIO_Pin_4) exti_pin_source = GPIO_PinSource4;
    else if(gpio_pin == GPIO_Pin_5) exti_pin_source = GPIO_PinSource5;
    else if(gpio_pin == GPIO_Pin_6) exti_pin_source = GPIO_PinSource6;
    else if(gpio_pin == GPIO_Pin_7) exti_pin_source = GPIO_PinSource7;
    else if(gpio_pin == GPIO_Pin_8) exti_pin_source = GPIO_PinSource8;
    else if(gpio_pin == GPIO_Pin_9) exti_pin_source = GPIO_PinSource9;
    else if(gpio_pin == GPIO_Pin_10) exti_pin_source = GPIO_PinSource10;
    else if(gpio_pin == GPIO_Pin_11) exti_pin_source = GPIO_PinSource11;
    else if(gpio_pin == GPIO_Pin_12) exti_pin_source = GPIO_PinSource12;
    else if(gpio_pin == GPIO_Pin_13) exti_pin_source = GPIO_PinSource13;
    else if(gpio_pin == GPIO_Pin_14) exti_pin_source = GPIO_PinSource14;
    else if(gpio_pin == GPIO_Pin_15) exti_pin_source = GPIO_PinSource15;
    else return;
    
    // 映射GPIO引脚到EXTI线
    GPIO_EXTILineConfig(exti_port_source, exti_pin_source);
    
    // EXTI配置
    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = 1 << exti_pin_source;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = trigger_mode;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStruct);
    
    // 确定中断通道
    IRQn_Type irq_channel;
    if(exti_pin_source <= 4) {
        if(exti_pin_source == 0) irq_channel = EXTI0_IRQn;
        else if(exti_pin_source == 1) irq_channel = EXTI1_IRQn;
        else if(exti_pin_source == 2) irq_channel = EXTI2_IRQn;
        else if(exti_pin_source == 3) irq_channel = EXTI3_IRQn;
        else irq_channel = EXTI4_IRQn;
    } else if(exti_pin_source <= 9) {
        irq_channel = EXTI9_5_IRQn;
    } else {
        irq_channel = EXTI15_10_IRQn;
    }
    
    // NVIC配置
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = irq_channel;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
}

/**
 * @brief  读取按钮状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @retval 1: 按下, 0: 未按下
 */
uint8_t Button_Read(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) {
    if(GPIO_ReadInputDataBit(gpio_port, gpio_pin) == Bit_RESET) {
        return 1;  // 按下 (低电平)
    } else {
        return 0;  // 未按下 (高电平)
    }
}

// 中断服务函数示例
// void EXTI0_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line0) != RESET) {
//         // 按钮0中断处理
//         EXTI_ClearITPendingBit(EXTI_Line0);
//     }
// }
// 
// void EXTI1_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line1) != RESET) {
//         // 按钮1中断处理
//         EXTI_ClearITPendingBit(EXTI_Line1);
//     }
// }
// 
// void EXTI9_5_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
//         // 按钮5中断处理
//         EXTI_ClearITPendingBit(EXTI_Line5);
//     }
//     if(EXTI_GetITStatus(EXTI_Line6) != RESET) {
//         // 按钮6中断处理
//         EXTI_ClearITPendingBit(EXTI_Line6);
//     }
//     // ... 其他引脚
// }

/**
 * @brief  按钮GPIO初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pin: GPIO引脚 (GPIO_Pin_0 到 GPIO_Pin_15)
 * @param  pull_mode: 上拉/下拉模式 (GPIO_Mode_IPU=上拉, GPIO_Mode_IPD=下拉, GPIO_Mode_IN_FLOATING=浮空)
 * @retval None
 */
void Button_GPIO_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, GPIOMode_TypeDef pull_mode) {
    // 开启时钟
    if(gpio_port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if(gpio_port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if(gpio_port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if(gpio_port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if(gpio_port == GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    else if(gpio_port == GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    else if(gpio_port == GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    
    // GPIO初始化
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = gpio_pin;
    GPIO_InitStruct.GPIO_Mode = pull_mode;  // 上拉/下拉/浮空输入
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio_port, &GPIO_InitStruct);
}

/**
 * @brief  读取按钮状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @retval 1: 按下, 0: 未按下
 */
uint8_t Button_Read(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) {
    if(GPIO_ReadInputDataBit(gpio_port, gpio_pin) == Bit_RESET) {
        return 1;  // 按下 (低电平)
    } else {
        return 0;  // 未按下 (高电平)
    }
}

/**
 * @brief  按钮状态检测函数（带去抖动）
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @param  last_state: 上次状态指针
 * @param  last_time: 上次时间指针
 * @param  current_time: 当前时间
 * @param  debounce_time: 去抖动时间（毫秒）
 * @retval 0: 无变化, 1: 按下, 2: 释放
 */
uint8_t Button_Detect(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, 
                     uint8_t* last_state, uint32_t* last_time, 
                     uint32_t current_time, uint32_t debounce_time) {
    uint8_t current_input = Button_Read(gpio_port, gpio_pin);
    
    if(current_input != *last_state) {
        if(current_time - *last_time >= debounce_time) {
            *last_state = current_input;
            *last_time = current_time;
            if(current_input) {
                return 1; // 按下
            } else {
                return 2; // 释放
            }
        }
    } else {
        *last_time = current_time; // 重置时间
    }
    
    return 0; // 无变化
}

/**
 * @brief  按钮单击检测函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @param  last_state: 上次状态指针
 * @param  last_time: 上次时间指针
 * @param  current_time: 当前时间
 * @param  debounce_time: 去抖动时间
 * @retval 1: 单击, 0: 无操作
 */
uint8_t Button_Click(GPIO_TypeDef* gpio_port, uint16_t gpio_pin,
                    uint8_t* last_state, uint32_t* last_time,
                    uint32_t current_time, uint32_t debounce_time) {
    static uint8_t press_detected = 0;
    uint8_t current_input = Button_Read(gpio_port, gpio_pin);
    
    if(current_input != *last_state) {
        if(current_time - *last_time >= debounce_time) {
            *last_state = current_input;
            *last_time = current_time;
            
            if(current_input) {  // 按下
                press_detected = 1;
            } else if(press_detected) {  // 释放
                press_detected = 0;
                return 1; // 单击完成
            }
        }
    } else {
        *last_time = current_time; // 重置时间
    }
    
    return 0; // 无单击
}

/**
 * @brief  按钮长按检测函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @param  pressed_time: 按下时间指针
 * @param  current_time: 当前时间
 * @param  long_press_time: 长按时间阈值（毫秒）
 * @retval 0: 未长按, 1: 长按触发
 */
uint8_t Button_Long_Press(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, 
                         uint32_t* pressed_time, uint32_t current_time, 
                         uint32_t long_press_time) {
    if(Button_Read(gpio_port, gpio_pin)) {  // 按钮被按下
        if(*pressed_time == 0) {
            *pressed_time = current_time;  // 记录按下时间
        } else if(current_time - *pressed_time >= long_press_time) {
            return 1; // 长按触发
        }
    } else {
        *pressed_time = 0;  // 按钮释放，重置计时
    }
    
    return 0; // 未长按
}

/**
 * @brief  按钮双击检测函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @param  last_state: 上次状态指针
 * @param  last_time: 上次时间指针
 * @param  click_count: 点击计数指针
 * @param  last_click_time: 上次点击时间指针
 * @param  current_time: 当前时间
 * @param  debounce_time: 去抖动时间
 * @param  double_click_time: 双击间隔时间
 * @retval 0: 无操作, 1: 单击, 2: 双击
 */
uint8_t Button_Double_Click(GPIO_TypeDef* gpio_port, uint16_t gpio_pin,
                           uint8_t* last_state, uint32_t* last_time,
                           uint8_t* click_count, uint32_t* last_click_time,
                           uint32_t current_time, uint32_t debounce_time, uint32_t double_click_time) {
    uint8_t current_input = Button_Read(gpio_port, gpio_pin);
    
    if(current_input != *last_state) {
        if(current_time - *last_time >= debounce_time) {
            *last_state = current_input;
            *last_time = current_time;
            
            if(current_input) {  // 按下
                // 什么都不做，等待释放
            } else {  // 释放
                if(current_time - *last_click_time <= double_click_time) {
                    (*click_count)++;
                } else {
                    *click_count = 1;  // 重置为第一次点击
                }
                *last_click_time = current_time;
                
                if(*click_count >= 2) {
                    *click_count = 0;  // 重置计数
                    return 2; // 双击
                }
            }
        }
    } else {
        *last_time = current_time; // 重置时间
        
        // 检查是否超过双击时间间隔
        if(*click_count == 1 && current_time - *last_click_time >= double_click_time) {
            *click_count = 0;  // 重置计数
            return 1; // 单击
        }
    }
    
    return 0; // 无操作
}

// 使用示例
void example_usage(void) {
    // EXTI版本使用示例
    Button_EXTI_Init(GPIOA, GPIO_Pin_0, GPIO_Mode_IPU, EXTI_Trigger_Falling, 1);
    
    // GPIO版本使用示例
    Button_GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_IPU);
    
    // 变量定义
    static uint8_t last_state = 0;
    static uint32_t last_time = 0;
    static uint32_t pressed_time = 0;
    static uint8_t click_count = 0;
    static uint32_t last_click_time = 0;
    
    uint32_t current_time = 1000; // 假设当前时间
    
    // 检测按钮状态变化
    uint8_t status = Button_Detect(GPIOB, GPIO_Pin_1, &last_state, &last_time, current_time, 50);
    
    // 检测单击
    uint8_t click = Button_Click(GPIOB, GPIO_Pin_1, &last_state, &last_time, current_time, 50);
    
    // 检测长按
    uint8_t long_press = Button_Long_Press(GPIOB, GPIO_Pin_1, &pressed_time, current_time, 1000);
    
    // 检测双击
    uint8_t double_click = Button_Double_Click(GPIOB, GPIO_Pin_1, &last_state, &last_time,
                                              &click_count, &last_click_time, current_time, 50, 300);
}
