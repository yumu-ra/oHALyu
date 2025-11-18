#include "stm32f10x.h"

/**
 * @brief  传感器初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pin: GPIO引脚 (GPIO_Pin_0 到 GPIO_Pin_15)
 * @param  input_mode: 输入模式 (GPIO_Mode_IPD, GPIO_Mode_IPU, GPIO_Mode_IN_FLOATING)
 * @param  trigger_mode: 触发模式 (EXTI_Trigger_Rising, EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling)
 * @param  priority: 中断优先级 (0-15)
 * @retval None
 */
void Sensor_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, GPIOMode_TypeDef input_mode, 
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
    GPIO_InitStruct.GPIO_Mode = input_mode;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(gpio_port, &GPIO_InitStruct);
    
    // 确定GPIO端口源
    uint8_t exti_port_source;
    if(gpio_port == GPIOA) exti_port_source = GPIO_PortSourceGPIOA;
    else if(gpio_port == GPIOB) exti_port_source = GPIO_PortSourceGPIOB;
    else if(gpio_port == GPIOC) exti_port_source = GPIO_PortSourceGPIOC;
    else if(gpio_port == GPIOD) exti_port_source = GPIO_PortSourceGPIOD;
    else if(gpio_port == GPIOE) exti_port_source = GPIO_PortSourceGPIOE;
    else if(gpio_port == GPIOF) exti_port_source = GPIO_PortSourceGPIOF;
    else if(gpio_port == GPIOG) exti_port_source = GPIO_PortSourceGPIOG;
    else return;
    
    // 确定GPIO引脚源
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
    EXTI_InitStruct.EXTI_Line = 1 << exti_pin_source;  // 对应EXTI线
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
 * @brief  读取传感器状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚
 * @retval 1: 高电平, 0: 低电平
 */
uint8_t Sensor_Read(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) {
    if(GPIO_ReadInputDataBit(gpio_port, gpio_pin) == Bit_SET) {
        return 1;  // 高电平
    } else {
        return 0;  // 低电平
    }
}

uint8_t Sensor_Debounce(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, 
                       uint8_t* last_state, uint32_t* last_time, 
                       uint32_t current_time, uint32_t debounce_time) {
    uint8_t current_input = Sensor_Read(gpio_port, gpio_pin);
    
    if(current_input != *last_state) {
        if(current_time - *last_time >= debounce_time) {
            *last_state = current_input;
            *last_time = current_time;
            return 1; // 状态改变
        }
    } else {
        *last_time = current_time; // 重置时间，如果状态不一致
    }
    
    return 0; // 状态未改变
}

// 使用示例
void example_usage(void) {
    // 示例1: 光传感器初始化 (原功能)
    Sensor_Init(GPIOA, GPIO_Pin_5, GPIO_Mode_IPD, EXTI_Trigger_Rising, 1);
    
    // 示例2: 简化版初始化
   // Sensor_Init_Simple(GPIOA, GPIO_Pin_5, EXTI_Trigger_Rising, 1, 1);
    
    // 示例3: 不启用中断的传感器
    Sensor_Init_Simple(GPIOB, GPIO_Pin_0, EXTI_Trigger_Rising, 0, 0);
    
    // 示例4: 读取传感器状态
    uint8_t sensor_state = Sensor_Read(GPIOA, GPIO_Pin_5);
    
    // 示例5: 传感器去抖动
    static uint8_t last_state = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = 1000; // 假设当前时间
    uint8_t state_changed = Sensor_Debounce(GPIOA, GPIO_Pin_5, &last_state, &last_time, current_time, 50);
}

// 中断服务函数示例
// void EXTI9_5_IRQHandler(void) {
//     if(EXTI_GetITStatus(EXTI_Line5) != RESET) {
//         // 传感器触发事件处理
//         // 用户代码
//         EXTI_ClearITPendingBit(EXTI_Line5);
//     }
// }