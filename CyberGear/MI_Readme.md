# 小米电机驱动库 (XiaoMi Motor Driver)

## 简介
基于STM32 HAL库的小米CyberGear电机驱动库，支持多电机控制和多种控制模式。

## 功能特性
- ✅ 支持最多4台小米电机同时控制
- ✅ 支持4种控制模式：运动模式、位置模式、速度模式、电流模式
- ✅ 支持实时状态反馈（角度、速度、扭矩、温度）
- ✅ 支持动态电机ID映射
- ✅ 支持电机参数设置

## 硬件要求
- STM32微控制器（带CAN接口）
- 小米CyberGear电机
- CAN收发器

## 快速开始

### 1. 初始化配置
```c
// 配置CAN过滤器
CAN_Filter_Init();

// 初始化电机（示例）
init_cybergear(&mi_motor[0], 0x7F, Speed_mode);    // ID:0x7F, 速度模式
init_cybergear(&mi_motor[1], 0x7E, Position_mode); // ID:0x7E, 位置模式
init_cybergear(&mi_motor[2], 0x7D, Current_mode);  // ID:0x7D, 电流模式
```

### 2. 控制模式说明

#### 运动模式 (Motion_mode = 0)
```c
// 同时控制位置、速度、力矩等参数
motor_controlmode(&mi_motor[0], torque, position, speed, kp, kd);
```

#### 速度模式 (Speed_mode = 2)  
```c
// 先设置模式
set_mode_cybergear(&mi_motor[0], Speed_mode);
// 再设置目标速度
set_target_speed(&mi_motor[0], 15.0);  // 目标速度15 rad/s
```

#### 位置模式 (Position_mode = 1)
```c
// 先设置模式
set_mode_cybergear(&mi_motor[0], Position_mode);
// 再设置目标位置
set_target_position(&mi_motor[0], 3.14);  // 目标位置π rad
```

#### 电流模式 (Current_mode = 3)
```c
// 先设置模式
set_mode_cybergear(&mi_motor[0], Current_mode);
// 再设置目标电流
set_target_current(&mi_motor[0], 2.5);  // 目标电流2.5 A
```

### 3. 状态反馈
通过CAN中断回调函数自动更新电机状态：
```c
// 实时获取电机状态
float angle = mi_motor[0].Angle;    // 角度
float speed = mi_motor[0].Speed;    // 速度  
float torque = mi_motor[0].Torque;  // 扭矩
float temp = mi_motor[0].Temp;      // 温度
```

## 主要API函数

### 初始化函数
- `init_cybergear(Motor, Can_Id, mode)` - 初始化电机
- `CAN_Filter_Init()` - CAN过滤器初始化

### 控制函数
- `set_mode_cybergear(Motor, mode)` - 设置控制模式
- `set_target_speed(Motor, speed)` - 速度模式下设置目标速度
- `set_target_position(Motor, pos)` - 位置模式下设置目标位置  
- `set_target_current(Motor, current)` - 电流模式下设置目标电流
- `motor_controlmode(Motor, torque, pos, speed, kp, kd)` - 运动模式控制
- `start_cybergear(Motor)` - 使能电机
- `stop_cybergear(Motor, clear_error)` - 停止电机

### 辅助函数
- `set_zeropos_cybergear(Motor)` - 设置机械零点
- `set_CANID_cybergear(Motor, new_id)` - 修改电机ID
- `chack_cybergear(ID)` - 检查电机ID

## 参数范围
- 位置范围：[-12.5, 12.5] rad
- 速度范围：[-30, 30] rad/s  
- 扭矩范围：[-12, 12] N·m
- 电流范围：根据电机规格

## 注意事项
1. 模式切换必须在电机停止状态下进行
2. 初始化时注意电机ID不要重复
3. 运动模式下的参数协调控制效果最佳
4. 实时监控电机温度和状态防止过载

## 文件结构
- `XiaoMi.h` - 头文件，包含结构体定义和函数声明
- `XiaoMi.c` - 源文件，包含具体实现
- `main.h/can.h` - STM32 HAL库相关
