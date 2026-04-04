#include "struct_typedef.h"
#include "pid.h"
#include "CAN_receive.h"
#include "can.h"
#include <stdlib.h>
#include "motor_driver.h"
// PID参数 (底盘3508电机 双环PID：角度环 + 速度环)
// 角度环PID [Kp, Ki, Kd]
static const fp32 angle_pid_params[3] = {10.0f, 0.0f, 0.0f};
// 速度环PID [Kp, Ki, Kd]
static const fp32 pid_params[3]      = {5.0f,  0.1f, 0.0f};

// PID输出限幅 (3508电机最大电流：±16384)
#define max_out          10000.0f   // 速度环输出限幅（电机电流）
#define max_iout         5000.0f    // 速度环积分限幅
#define max_angle_out    5000.0f    // 角度环输出限幅（目标速度）
#define max_angle_iout   1000.0f    // 角度环积分限幅

// 电机目标角度 & 到位阈值 (3508编码器：0~8191)
//#define target_angle_val     4096   // 目标角度：半圈
#define angle_error_threshold 50    // 角度误差阈值（小于此值判定为到位）
// 电机目标角度 & 到位阈值 (3508编码器：0~8191)
#define ENCODER_RESOLUTION 8192  // 编码器分辨率
#define PI 3.14159265358979323846f

// 角度转换宏
#define RAD_TO_ENCODER(rad) ((uint16_t)((rad / (2 * PI)) * ENCODER_RESOLUTION))  // 弧度转编码器值
#define ENCODER_TO_RAD(enc) ((float)(enc) * (2 * PI) / ENCODER_RESOLUTION)  // 编码器值转弧度

// 全局变量
pid_type_def motor_pid[4];          // 4个电机速度环PID
pid_type_def angle_pid[4];          // 4个电机角度环PID
uint16_t target_angle[4];           // 电机目标角度
uint8_t  motor_reached[4];          // 电机到位标志（0=未到位，1=已到位）
uint16_t target_speed[4];           // 电机目标速度

static motor_mode_t motor_mode = MOTOR_MODE_SPEED;
//CAN句柄
CAN_HandleTypeDef *CHASSIS_CAN_PTR = NULL;

void Rm3508_Init(CAN_HandleTypeDef *hcan,motor_mode_t mode){
  motor_mode = mode;
	
  CHASSIS_CAN_PTR=hcan;
  // 电机ID重置
  CAN_cmd_chassis_reset_ID();
  HAL_Delay(50);

  // 初始化4个电机的双环PID
  for(uint8_t i = 0; i < 4; i++)
  {
    if (motor_mode == MOTOR_MODE_SPEED)
    {
      PID_init(&motor_pid[i], PID_POSITION, pid_params, max_out, max_iout);
    }else if (motor_mode == MOTOR_MODE_ANGLE)
    {
      PID_init(&motor_pid[i], PID_POSITION, pid_params, max_out, max_iout);
      PID_init(&angle_pid[i], PID_POSITION, angle_pid_params, max_angle_out, max_angle_iout);
      //target_angle[i] = target_angle_val;  // 统一设置目标角度
      //motor_reached[i] = 0;                // 初始化为未到位
    }
  }
}

// 设置电机目标角度
void RM_Set_Target_Angle(uint8_t motor_id, float angle){
  motor_mode = MOTOR_MODE_ANGLE;
  if(motor_id < 4){
    target_angle[motor_id] = RAD_TO_ENCODER(angle);
    motor_reached[motor_id] = 0;  // 重置到位标志
  }
}
// 设置电机目标速度
void RM_Set_Target_Speed(uint8_t motor_id, uint16_t speed){
  motor_mode = MOTOR_MODE_SPEED;
  if(motor_id < 4){
    target_speed[motor_id] = speed;
    motor_reached[motor_id] = 0;  // 重置到位标志
  }
}
/**
 * @brief 底盘电机PID控制处理函数
 * @details 实现4个底盘电机的双环PID控制（角度环+速度环）
 *          当电机达到目标角度时停止输出电流
 */

void RM_PID_Process(void){
  int16_t motor_current[4] = {0};


  // 4个底盘电机闭环控制
  for(uint8_t i = 0; i < 4; i++)
  {
    const motor_measure_t *motor_data = get_chassis_motor_measure_point(i);
    if(motor_data == NULL) continue;
    if(motor_mode == MOTOR_MODE_ANGLE)
    {
      // 未到达目标角度 → 双环PID计算
      if(motor_reached[i] == 0)
      {
        // 角度误差判断
        int32_t angle_error = abs((int32_t)motor_data->ecd - target_angle[i]);
        
        if(angle_error <= angle_error_threshold)
        {
          motor_reached[i] = 1;  // 标记已到位
          motor_current[i] = 0;
        }
        else
        {
          // 角度环PID → 输出目标转速
          fp32 target_speed = PID_calc(&angle_pid[i], motor_data->ecd, target_angle[i]);
          // 速度环PID → 输出控制电流
          fp32 current = PID_calc(&motor_pid[i], motor_data->speed_rpm, target_speed);
          motor_current[i] = (int16_t)current;
        }
      }
      else
      {
        motor_current[i] = 0;  // 到位后电流清零
      }
    }
    else if(motor_mode == MOTOR_MODE_SPEED)
    {
      // 速度环PID → 输出控制电流
      fp32 current = PID_calc(&motor_pid[i], motor_data->speed_rpm, target_speed[i]);
      motor_current[i] = (int16_t)current;
      }
    }
  // 发送CAN电流指令
  CAN_cmd_chassis(motor_current[0], motor_current[1], motor_current[2], motor_current[3]);
}

/**
 * @brief  CAN过滤器初始化（接收所有CAN报文，映射到FIFO0）
 * @note   为电机数据接收做准备
 * @retval None
 */
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);
    HAL_CAN_Start(hcan);
    // 开启FIFO0报文挂起中断（用于接收电机数据）
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}
