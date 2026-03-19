# 我搜集开源的robocom驱动库代码
核心：封装官方标准库，调用更简洁，搜集开源驱动代码
## 怎么用
1. 无额外依赖，直接引入项目
2. 示例看c源文件
## 
为了便于显示我将printf重定向代码直接放到这里,只要把此代码copy至main
```C
#include "stdio.h"

// 1. 禁用半主机模式（必加，无需开微库）
#if !defined(__MICROLIB)
#pragma import(__use_no_semihosting)
void _sys_exit(int x) {x = x;}
void _ttywrch(int x) {x = x;}
struct __FILE { int handle; };
FILE __stdout;
#endif

// 2. UART4 DMA 重定向printf（核心！）
int fputc(int ch, FILE *stream)
{
    // 等待上一次DMA发送完成（防止数据冲突）
    while(HAL_UART_GetState(&huart4) != HAL_UART_STATE_READY);
    
    // DMA发送单个字符（UART4 TX，超时10ms）
    HAL_UART_Transmit_DMA(&huart4, (uint8_t *)&ch, 1);
    
    return ch;
}
```
## 后续
不定时更新，只加高频实用封装

## 许可证
MIT - 详见 [LICENSE](LICENSE)
