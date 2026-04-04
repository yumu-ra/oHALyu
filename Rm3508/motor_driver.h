/**
  ******************************************************************************
  * @file           : motor_driver.h
  * @brief          : Motor driver header
  * @author         : 未知
  * @version        : V1.0.0
  * @author         : yumu_Ra
  * @version        : V2.0.0
  ******************************************************************************
  * @attention
  * 这个头文件包含了电机3508的驱动函数声明。
  * 包括初始化、PID控制、设置目标速度和角度的函数。
  * motor_mode_t motor_mode_t 电机3508的模式，包括速度模式和角度模式。
  * 本程序来源于某位仁兄提供的代码，用于控制电机3508。并由yumu_Ra修改。
  * 注意：
  * 1. 电机3508的CAN ID为0x200 + 电机ID（0-3）。
  * 2. 因为假期原因此代码并未在硬件上实际测试，仅通过编译与AI检查。
  * 3.在角度模式下，目标角度为弧度制。
  * 4.在速度模式下，为转/秒。
  * 5.如何停止电机，需要设置目标速度为0。
  *
  ******************************************************************************
  */
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H
#include "main.h"
// 电机模式
typedef enum
{
    MOTOR_MODE_SPEED = 0,
    MOTOR_MODE_ANGLE = 1,
} motor_mode_t;

// CAN过滤器初始化函数声明
void CAN_Filter_Init(CAN_HandleTypeDef *hcan);
void Rm3508_Init(CAN_HandleTypeDef *hcan,motor_mode_t mode);
void RM_PID_Process(void);
// 设置电机目标速度
void RM_Set_Target_Speed(uint8_t motor_id, uint16_t speed);
// 设置电机目标角度
void RM_Set_Target_Angle(uint8_t motor_id, uint16_t angle);
#endif
