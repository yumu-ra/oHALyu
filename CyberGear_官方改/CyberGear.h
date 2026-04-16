/**
* @brief 电机控制头文件
* @author yumu_Ra 注：原版为小米灵足官方驱动(cpp),现利用AI移植到c语言
* @version 0.1
*/
#ifndef __CYBERGEAR_H__
#define __CYBERGEAR_H__

#include "main.h"
#include "can.h"
#include <stdbool.h>
 
 #define Set_mode 		 'j'				//设置控制模式
 #define Set_parameter 'p'				//设置参数
 //各种控制模式
 #define move_control_mode  0	//运控模式
 #define Pos_control_mode   1	//PP位置模式
 #define Speed_control_mode 2 //速度模式
 #define Elect_control_mode 3 //电流模式
 #define Set_Zero_mode      4 //零点模式
 #define CSP_control_mode		5	//CSP位置模式
 //通信地址
#define Communication_Type_Get_ID 0x00      					//获取设备的ID和64位MCU唯一标识符`
#define Communication_Type_MotionControl 0x01  				//运控模式用来向主机发送控制指令
#define Communication_Type_MotorRequest 0x02				//用来向主机反馈电机运行状态
#define Communication_Type_MotorEnable 0x03					//电机使能运行
#define Communication_Type_MotorStop 0x04						//电机停止运行
#define Communication_Type_SetPosZero 0x06					//设置电机机械零位
#define Communication_Type_Can_ID 0x07							//更改当前电机CAN_ID
#define Communication_Type_Control_Mode 0x12				//设置电机模式
#define Communication_Type_GetSingleParameter 0x11		//读取单个参数
#define Communication_Type_SetSingleParameter 0x12		//设定单个参数
#define Communication_Type_ErrorFeedback 0x15				//故障反馈帧
//使用以下模式时请注意电机驱动版本号是否 >= 0.13.0		
#define Communication_Type_MotorDataSave 0x16				//电机数据保存帧
#define Communication_Type_BaudRateChange 0x17			//电机波特率修改帧，重新上电生效
#define Communication_Type_ProactiveEscalationSet 0x18	//电机主动上报
#define Communication_Type_MotorModeSet 0x19				//电机协议修改帧，重新上电生效


typedef struct
{
	uint16_t index;
	float data;
} data_read_write_one_t;

static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};

//18通信类型可以写入的参数列表
//参数变量名		参数地址		描述		类型		字节数		单位/说明
typedef struct		//可读写的参数
{
	data_read_write_one_t run_mode; 			//0:运控模式 1:位置模式 2:速度模式 3:电流模式 4:零点模式 uint8  1byte
	data_read_write_one_t iq_ref; 				//电流模式Iq指令    		float 	4byte  	-23~23A
	data_read_write_one_t spd_ref; 				//转速模式转速指令 			float 	4byte  	-30~30rad/s 
	data_read_write_one_t imit_torque; 		//转矩限制 					float 	4byte  	0~12Nm  
	data_read_write_one_t cur_kp; 				//电流的 Kp  				float 	4byte  	默认值 0.125  
	data_read_write_one_t cur_ki; 				//电流的 Ki  				float 	4byte  	默认值 0.0158  
	data_read_write_one_t cur_filt_gain; 	//电流滤波系数filt_gain  	float 	4byte  	0~1.0，默认值0.1  
	data_read_write_one_t loc_ref; 			//位置模式角度指令			float 	4byte  	rad  
	data_read_write_one_t limit_spd; 		//位置模式速度设置			float 	4byte  	0~30rad/s  
	data_read_write_one_t limit_cur; 		//速度位置模式电流设置 		float 	4byte  	0~23A
	//以下只可读
	data_read_write_one_t mechPos; 			//负载端计圈机械角度			float 	4byte  	rad
	data_read_write_one_t iqf; 				//iq 滤波值  				float 	4byte  	-23~23A
	data_read_write_one_t mechVel; 			//负载端转速					float 	4byte  	-30~30rad/s  
	data_read_write_one_t VBUS; 				//母线电压						float 	4byte  	V	
	data_read_write_one_t rotation; 			//圈数  						int16  	2byte   圈数
} data_read_write_t;

typedef struct
{
	float Angle;
	float Speed;
	float Torque;
	float Temp;
	int pattern; //电机模式（0复位1标定2运行）
} Motor_Pos_RobStride_Info;

typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_acceleration;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_limit_speed;
	float set_Kp;
	float set_Ki;
	float set_Kd;
} Motor_Set;

typedef enum
{
    operationControl = 0,
    positionControl = 1,
    speedControl = 2
} MIT_TYPE;

//RobStride_Motor电机
typedef struct
{
	// 私有成员
	uint8_t CAN_ID; 				//CAN ID   (默认127(0x7f) 可以通过上位机和通信类型1查看)
	uint64_t Unique_ID; 			//64位MCU唯一标识符
	uint16_t Master_CAN_ID; 	//主机ID  （会在初始化函数中设定为0x1F）
	float (*Motor_Offset_MotoFunc)(float Motor_Tar);

	Motor_Set Motor_Set_All; 	//设定值
	uint8_t error_code;

	bool MIT_Mode; 		//MIT模式
	MIT_TYPE MIT_Type; 	//MIT模式类型

	// 公共成员
	float output;
	int Can_Motor;
	Motor_Pos_RobStride_Info Pos_Info; 	//回传值
	data_read_write_t drw;       				//电机数据
} RobStride_Motor_t;

// 函数声明

// 初始化函数
void RobStride_Motor_init(RobStride_Motor_t *motor, uint8_t CAN_Id, bool MIT_mode);
void RobStride_Motor_init_with_offset(RobStride_Motor_t *motor, float (*Offset_MotoFunc)(float Motor_Tar), uint8_t CAN_Id, bool MIT_mode);

// 数据读取写入初始化
void data_read_write_init(data_read_write_t *drw, const uint16_t *index_list);

// 电机控制函数
void RobStride_Get_CAN_ID(RobStride_Motor_t *motor);
void Set_RobStride_Motor_parameter(RobStride_Motor_t *motor, uint16_t Index, float Value, char Value_mode);
void Get_RobStride_Motor_parameter(RobStride_Motor_t *motor, uint16_t Index);
void RobStride_Motor_Analysis(RobStride_Motor_t *motor, uint8_t *DataFrame, uint32_t ID_ExtId);
void RobStride_Motor_move_control(RobStride_Motor_t *motor, float Torque, float Angle, float Speed, float Kp, float Kd);
void RobStride_Motor_Pos_control(RobStride_Motor_t *motor, float Speed, float Angle);
void RobStride_Motor_CSP_control(RobStride_Motor_t *motor, float Angle, float limit_spd);
void RobStride_Motor_Speed_control(RobStride_Motor_t *motor, float Speed, float limit_cur);
void RobStride_Motor_current_control(RobStride_Motor_t *motor, float current);
void RobStride_Motor_Set_Zero_control(RobStride_Motor_t *motor);
void RobStride_Motor_MotorModeSet(RobStride_Motor_t *motor, uint8_t F_CMD);
void Enable_Motor(RobStride_Motor_t *motor);
void Disenable_Motor(RobStride_Motor_t *motor, uint8_t clear_error);
void Set_CAN_ID(RobStride_Motor_t *motor, uint8_t Set_CAN_ID);
void Set_ZeroPos(RobStride_Motor_t *motor);

// MIT模式相关函数
void RobStride_Motor_MIT_Control(RobStride_Motor_t *motor, float Angle, float Speed, float Kp, float Kd, float Torque);
void RobStride_Motor_MIT_PositionControl(RobStride_Motor_t *motor, float position_rad, float speed_rad_per_s);
void RobStride_Motor_MIT_SpeedControl(RobStride_Motor_t *motor, float speed_rad_per_s, float current_limit);
void RobStride_Motor_MIT_Enable(RobStride_Motor_t *motor);
void RobStride_Motor_MIT_Disable(RobStride_Motor_t *motor);
void RobStride_Motor_MIT_SetZeroPos(RobStride_Motor_t *motor);
void RobStride_Motor_MIT_ClearOrCheckError(RobStride_Motor_t *motor, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorType(RobStride_Motor_t *motor, uint8_t F_CMD);
void RobStride_Motor_MIT_SetMotorId(RobStride_Motor_t *motor, uint8_t F_CMD);
void RobStride_Motor_MIT_MotorModeSet(RobStride_Motor_t *motor, uint8_t F_CMD);

// 其他功能函数
void RobStride_Motor_MotorDataSave(RobStride_Motor_t *motor);
void RobStride_Motor_BaudRateChange(RobStride_Motor_t *motor, uint8_t F_CMD);
void RobStride_Motor_ProactiveEscalationSet(RobStride_Motor_t *motor, uint8_t F_CMD);

// 辅助函数
bool Get_MIT_Mode(RobStride_Motor_t *motor);
MIT_TYPE get_MIT_Type(RobStride_Motor_t *motor);
void Set_MIT_Mode(RobStride_Motor_t *motor, bool MIT_Mode);
void Set_MIT_Type(RobStride_Motor_t *motor, MIT_TYPE MIT_Type);

// 工具函数
float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
float Byte_to_float(uint8_t* bytedata);
uint8_t mapFaults(uint16_t fault16);

#endif
