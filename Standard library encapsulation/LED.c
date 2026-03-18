#include "stm32f10x.h"

/**
 * @brief  LED初始化函数
 * @param  gpio_port: GPIO端口 (GPIOA, GPIOB, GPIOC等)
 * @param  gpio_pins: GPIO引脚 (可以是多个引脚的或运算，如GPIO_Pin_0 | GPIO_Pin_1)
 * @param  speed: GPIO速度 (GPIO_Speed_2MHz, GPIO_Speed_10MHz, GPIO_Speed_50MHz)
 * @retval None
 */
void LED_Init(GPIO_TypeDef* gpio_port, uint16_t gpio_pins, GPIOSpeed_TypeDef speed) {
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
    
    /* 设置GPIO初始化后的默认电平（关闭LED，设置为高电平） */
    GPIO_SetBits(gpio_port, gpio_pins);
}

/**
 * @brief  LED开启函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void LED_ON(GPIO_TypeDef* gpio_port, uint16_t gpio_pins) {
    GPIO_ResetBits(gpio_port, gpio_pins);  // 设置引脚为低电平（点亮LED）
}

/**
 * @brief  LED关闭函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void LED_OFF(GPIO_TypeDef* gpio_port, uint16_t gpio_pins) {
    GPIO_SetBits(gpio_port, gpio_pins);    // 设置引脚为高电平（关闭LED）
}

/**
 * @brief  LED状态翻转函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pins: GPIO引脚
 * @retval None
 */
void LED_Turn(GPIO_TypeDef* gpio_port, uint16_t gpio_pins) {
    // 读取当前引脚状态并翻转
    uint16_t current_state = GPIO_ReadOutputData(gpio_port);
    GPIO_Write(gpio_port, current_state ^ gpio_pins);
}
/**
 * @brief  读取LED状态函数
 * @param  gpio_port: GPIO端口
 * @param  gpio_pin: GPIO引脚 (单个引脚)
 * @retval 1: LED开启, 0: LED关闭
 */
uint8_t LED_Get(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) {
    if(GPIO_ReadOutputDataBit(gpio_port, gpio_pin) == 0) {
        return 1;  // 引脚为低电平，LED开启
    } else {
        return 0;  // 引脚为高电平，LED关闭
    }
}

/**
 * @brief  批量LED控制函数
 * @param  gpio_port: GPIO端口
 * @param  led_states: LED状态数组 (1=开启, 0=关闭)
 * @param  led_pins: LED引脚数组
 * @param  count: LED数量
 * @retval None
 */
void LED_Batch_Control(GPIO_TypeDef* gpio_port, uint8_t* led_states, uint16_t* led_pins, uint8_t count) {
    for(uint8_t i = 0; i < count; i++) {
        LED_Set(gpio_port, led_pins[i], led_states[i]);
    }
}

// 使用示例
void example_usage(void) {
    // 示例1: 初始化PA0-PA4为LED
    LED_Init(GPIOA, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4, GPIO_Speed_50MHz);
    
    // 示例2: 初始化PB0-PB3为LED
    LED_Init(GPIOB, GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, GPIO_Speed_2MHz);
    
    // 示例3: 控制单个LED
    LED_ON(GPIOA, GPIO_Pin_0);      // 开启PA0的LED
    LED_OFF(GPIOA, GPIO_Pin_0);     // 关闭PA0的LED
    
    // 示例4: 控制多个LED
    LED_ON(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);  // 开启PA0和PA1的LED
    
    // 示例5: 翻转LED状态
    LED_Turn(GPIOA, GPIO_Pin_2);    // 翻转PA2的LED状态
    
    // 示例6: 设置LED状态
    LED_Set(GPIOA, GPIO_Pin_3, 1);  // 设置PA3的LED为开启状态
    
    // 示例7: 读取LED状态
    uint8_t led_state = LED_Get(GPIOA, GPIO_Pin_0);
    
    // 示例8: 批量控制LED
    uint8_t states[] = {1, 0, 1, 0};  // 开启、关闭、开启、关闭
    uint16_t pins[] = {GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3};
    LED_Batch_Control(GPIOA, states, pins, 4);
}
