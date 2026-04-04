#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "struct_typedef.h"
#include "main.h"
// CAN收发ID枚举（仅保留底盘3508电机）
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,  // 底盘电机总控ID
    CAN_3508_M1_ID = 0x201,     // 3508电机1 ID
    CAN_3508_M2_ID = 0x202,     // 3508电机2 ID
    CAN_3508_M3_ID = 0x203,     // 3508电机3 ID
    CAN_3508_M4_ID = 0x204,     // 3508电机4 ID
} can_msg_id_e;

// 电机测量数据结构体（3508通用）
typedef struct
{
    uint16_t ecd;            // 电机编码器值
    int16_t speed_rpm;       // 电机转速(rpm)
    int16_t given_current;   // 电机给定电流
    uint8_t temperate;       // 电机温度
    int16_t last_ecd;        // 上一帧编码器值
} motor_measure_t;

// 函数声明（仅保留底盘电机控制）
extern void CAN_cmd_chassis_reset_ID(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);

#endif
