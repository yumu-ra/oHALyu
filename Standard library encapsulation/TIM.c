#include "stm32f10x.h"

/**
 * @brief  通用定时器初始化函数
 * @param  timer: 定时器实例 (TIM2, TIM3, TIM4等)
 * @param  period: 自动重装载值 (0-65535)
 * @param  prescaler: 预分频值 (0-65535)
 * @param  interrupt_enable: 是否启用中断 (1=启用, 0=禁用)
 * @param  priority: 中断抢占优先级 (0-15)
 * @retval None
 */
void Timer_Init(TIM_TypeDef* timer, uint16_t period, uint16_t prescaler, 
                uint8_t interrupt_enable, uint8_t priority) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint8_t irq_channel;
    
    // 选择中断通道
    if(timer == TIM2) {
        irq_channel = TIM2_IRQn;
    } else if(timer == TIM3) {
        irq_channel = TIM3_IRQn;
    } else if(timer == TIM4) {
        irq_channel = TIM4_IRQn;
    } else {
        return; // 不支持的定时器
    }
    
    // 时基配置
    TIM_TimeBaseStructure.TIM_Period = period;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(timer, &TIM_TimeBaseStructure);
    
    // 配置中断
    if(interrupt_enable) {
        TIM_ITConfig(timer, TIM_IT_Update, ENABLE);
        
        // NVIC配置
        NVIC_InitStructure.NVIC_IRQChannel = irq_channel;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }
    
    TIM_Cmd(timer, ENABLE);  // 启动定时器
}

/**
 * @brief  基于目标频率的定时器初始化函数
 * @param  timer: 定时器实例 (TIM2, TIM3, TIM4等)
 * @param  target_freq: 目标频率 (Hz)
 * @param  timer_freq: 定时器时钟频率 (Hz, 默认72000000)
 * @param  interrupt_enable: 是否启用中断 (1=启用, 0=禁用)
 * @param  priority: 中断抢占优先级 (0-15)
 * @retval 0: 成功, 1: 失败
 */
uint8_t Timer_Init_ByFreq(TIM_TypeDef* timer, float target_freq, uint32_t timer_freq, 
                          uint8_t interrupt_enable, uint8_t priority) {
    uint32_t total_div = timer_freq / target_freq;
    uint16_t period, prescaler;
    
    if(total_div > 65535 * 65535) {
        return 1; // 超出范围
    }
    
    // 计算合适的分频值
    prescaler = (total_div + 65534) / 65535; // 向上取整
    if(prescaler > 65535) prescaler = 65535;
    
    period = total_div / (prescaler + 1);
    if(period > 65535) period = 65535;
    
    Timer_Init(timer, period, prescaler, interrupt_enable, priority);
    return 0;
}

/**
 * @brief  基于定时时间的定时器初始化函数
 * @param  timer: 定时器实例 (TIM2, TIM3, TIM4等)
 * @param  time_ms: 定时时间 (毫秒)
 * @param  interrupt_enable: 是否启用中断 (1=启用, 0=禁用)
 * @param  priority: 中断抢占优先级 (0-15)
 * @retval 0: 成功, 1: 失败
 */
uint8_t Timer_Init_ByTime(TIM_TypeDef* timer, uint32_t time_ms, 
                          uint8_t interrupt_enable, uint8_t priority) {
    // 使用10kHz基准，每100us计数一次
    uint32_t counts = time_ms * 10; // 毫秒转为100us单位
    uint16_t period, prescaler;
    
    if(counts > 65535) {
        // 需要更大的预分频
        prescaler = (counts + 65534) / 65535; // 向上取整
        if(prescaler > 65535) prescaler = 65535;
        period = counts / (prescaler + 1);
        if(period > 65535) period = 65535;
    } else {
        period = counts;
        prescaler = 0;
    }
    
    Timer_Init(timer, period, prescaler, interrupt_enable, priority);
    return 0;
}

// 使用示例
void example_usage(void) {
    // 示例1: 初始化1秒定时器 (原功能)
    TIM2_Init(9999, 7199, 1, 1); // period=9999, prescaler=7199, enable_int=1, priority=1
    
    // 示例2: 初始化100ms定时器
    TIM2_Init(999, 7199, 1, 2);  // 100ms定时器
    
    // 示例3: 使用通用定时器函数
    Timer_Init(TIM3, 4999, 14399, 1, 1); // TIM3配置
    
    // 示例4: 按频率初始化 (1Hz)
    Timer_Init_ByFreq(TIM4, 1.0f, 72000000, 1, 1);
    
    // 示例5: 按时间初始化 (500ms)
    Timer_Init_ByTime(TIM2, 500, 1, 1);
    
    // 示例6: 无中断模式
    TIM2_Init(9999, 7199, 0, 0); // 不启用中断
}

// 中断服务函数需要在main.c中定义
// void TIM2_IRQHandler(void) {
//     if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
//         // 用户代码
//         TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
//     }
// }