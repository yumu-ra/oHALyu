# 我搜集开源的robocom驱动库代码
Time is life. If developers spend a lot of time repeating the process of "building wheels" and constantly searching for libraries, it will inevitably slow down the progress of the entire technical community. Based on work requirements, I have written and compiled a set of various peripheral driver libraries based on STM32 HAL. Now I am making it open source, hoping to save some time for everyone. At the same time, I sincerely invite developers with the same idea to participate and share their own driver libraries, so as to provide convenience for more colleagues.
时间就是生命。如果开发者将大量时间耗费在重复“造轮子”和四处寻找库上，势必会拖慢整个技术社区的进步。基于工作需求，我编写并整理了一套基于 STM32 HAL 的各种外设驱动库，现将其开源，希望能为大家节省一些时间。同时，我诚挚邀请有同样想法的开发者一起参与，将各自的驱动库开源共享，为更多同行提供便利。
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
