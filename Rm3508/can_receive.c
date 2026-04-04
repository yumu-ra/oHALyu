#include "can_receive.h"
#include "main.h"

//声明CAN句柄
extern CAN_HandleTypeDef *CHASSIS_CAN_PTR;

// 底盘4个3508电机数据缓存
motor_measure_t motor_chassis[4];

// CAN发送结构体（仅底盘）
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

// 电机数据解析宏
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);       \
        (ptr)->given_current = (int16_t)((data)[4] << 8 | (data)[5]);   \
        (ptr)->temperate = (data)[6];                                   \
    }

/**
 * @brief  CAN FIFO0中断回调函数（接收底盘电机数据）
 * @param  hcan: CAN句柄
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    // 读取接收到的CAN报文
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // 仅解析底盘3508电机数据
    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
            uint8_t motor_idx = rx_header.StdId - CAN_3508_M1_ID;
            get_motor_measure(&motor_chassis[motor_idx], rx_data);
            break;
        }
        default:
            break;
    }
}

/**
 * @brief  底盘3508电机ID重置（发送0x700指令）
 * @retval None
 */
void CAN_cmd_chassis_reset_ID()
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    // 清空发送数据
    for(uint8_t i=0; i<8; i++) chassis_can_send_data[i] = 0;
    HAL_CAN_AddTxMessage(CHASSIS_CAN_PTR, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
 * @brief  底盘3508电机电流控制指令发送
 * @param  motor1~4: 对应4个3508电机电流 (-16384~16384)
 * @retval None
 */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(CHASSIS_CAN_PTR, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

// 获取底盘电机数据（i:0-3对应4个3508电机）
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}
