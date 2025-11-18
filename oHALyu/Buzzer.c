#include "stm32f10x.h"
#include <stdint.h>

/**
 * @brief  蜂鸣器初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pins: GPIO引脚 (可以是多个引脚的或运算)
 * @param  speed: GPIO速度 (GPIO_Speed_2MHz, GPIO_Speed_10MHz, GPIO_Speed_50MHz)
 * @retval None
 */
void Buzzer_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pins, GPIOSpeed_TypeDef speed) {
    /* 开启时钟 */
    if(gpio_port == GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    else if(gpio_port == GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    else if(gpio_port == GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    else if(gpio_port == GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    else if(gpio_port == GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
    else if(gpio_port == GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
    else if(gpio_port == GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);
    
    /* GPIO初始化 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;        // 推挽输出
    GPIO_InitStructure.GPIO_Pin = gpio_pins;
    GPIO_InitStructure.GPIO_Speed = speed;
    GPIO_Init(gpio_port, &GPIO_InitStructure);
    
    /* 设置GPIO初始化后的默认状态（关闭蜂鸣器） */
    GPIO_WriteBit(gpio_port, gpio_pins, Bit_RESET);
}
/**
 * @brief  蜂鸣器开启函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void Buzzer_ON(GPIO_TypeDef* gpio_port, uint16_t gpio_pins) {
    GPIO_SetBits(gpio_port, gpio_pins);  // 开启蜂鸣器
}

/**
 * @brief  蜂鸣器关闭函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void Buzzer_OFF(GPIO_TypeDef* gpio_port, uint16_t gpio_pins) {
    GPIO_ResetBits(gpio_port, gpio_pins);  // 关闭蜂鸣器
}

/**
 * @brief  蜂鸣器状态翻转函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void Buzzer_Turn(GPIO_TypeDef* gpio_port, uint16_t gpio_pins) {
    uint16_t current_state = GPIO_ReadOutputData(gpio_port);
    GPIO_Write(gpio_port, current_state ^ gpio_pins);
}

/**
 * @brief  读取蜂鸣器状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚 (单个引脚)
 * @retval 1: 开启, 0: 关闭
 */
uint8_t Buzzer_Get(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) {
    if(GPIO_ReadOutputDataBit(gpio_port, gpio_pin) == Bit_SET) {
        return 1;  // 蜂鸣器开启
    } else {
        return 0;  // 蜂鸣器关闭
    }
}
// 使用示例
void example_usage(void) {
    // 示例1: 初始化PB0为蜂鸣器
    Buzzer_Init(GPIOB, GPIO_Pin_0, GPIO_Speed_50MHz);
    
    // 示例2: 初始化PA0和PA1为蜂鸣器
    Buzzer_Init(GPIOA, GPIO_Pin_0 | GPIO_Pin_1, GPIO_Speed_10MHz);
    
    // 示例3: 设置蜂鸣器状态
    Buzzer_Set(GPIOB, GPIO_Pin_0, ENABLE);   // 开启蜂鸣器
    Buzzer_Set(GPIOB, GPIO_Pin_0, DISABLE);  // 关闭蜂鸣器
    
    // 示例4: 直接控制蜂鸣器
    Buzzer_ON(GPIOB, GPIO_Pin_0);   // 开启
    Buzzer_OFF(GPIOB, GPIO_Pin_0);  // 关闭
    Buzzer_Turn(GPIOB, GPIO_Pin_0); // 翻转状态
    
    // 示例5: 读取蜂鸣器状态
    uint8_t state = Buzzer_Get(GPIOB, GPIO_Pin_0);
    
    // 示例6: 蜂鸣指定时间
    // Buzzer_Beep(GPIOB, GPIO_Pin_0, 100);  // 蜂鸣100ms（需要延时函数）
}
