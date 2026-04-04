#ifndef PID_H
#define PID_H
#include "struct_typedef.h"

// PID模式枚举
enum PID_MODE
{
    PID_POSITION = 0,  // 位置式PID
    PID_DELTA          // 增量式PID
};

// PID控制结构体
typedef struct
{
    uint8_t mode;      // PID模式
    fp32 Kp;           // 比例系数
    fp32 Ki;           // 积分系数
    fp32 Kd;           // 微分系数

    fp32 max_out;      // PID输出限幅
    fp32 max_iout;     // 积分项限幅

    fp32 set;          // 设定值
    fp32 fdb;          // 反馈值

    fp32 out;          // PID总输出
    fp32 Pout;         // 比例项输出
    fp32 Iout;         // 积分项输出
    fp32 Dout;         // 微分项输出
    fp32 Dbuf[3];      // 微分缓冲（0:当前 1:上一帧 2:上两帧）
    fp32 error[3];     // 误差缓冲（0:当前 1:上一帧 2:上两帧）
} pid_type_def;

// PID函数声明
// PID初始化
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
// PID计算
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);
// PID清空
extern void PID_clear(pid_type_def *pid);

#endif
