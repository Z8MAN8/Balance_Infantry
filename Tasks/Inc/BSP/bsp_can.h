//
// Created by 14685 on 2022/7/15.
//

#ifndef HNU_RM_DOWN_BSP_CAN_H
#define HNU_RM_DOWN_BSP_CAN_H
#include "can.h"

# define CHASSIS_CAN hcan1
#define CAN_UP_TX_INFO 0x134
/* 上下板CAN通信的扩展标识符 */
#define CAN_RPY_TX          0x140   /* 底盘跟随云台相对角度 */
#define CAN_ODOM_TX_ONE     0x141   /* odom数据帧第一帧 */
#define CAN_ODOM_TX_TWO     0x142   /* odom数据帧第二帧 */
#define CAN_ODOM_TX_THREE   0x143   /* odom数据帧第二帧 */

#define CAN_GIM_STATE         0x145   /* 底盘状态数据 */

/**
 * @brief  CAN消息的ID
 */
typedef enum
{
    //接收ID
    CAN_3508_M1_ID       = 0x201,
    CAN_3508_M2_ID       = 0x202,
    CAN_3508_M3_ID       = 0x203,
    CAN_3508_M4_ID       = 0x204,
    CAN_YAW_MOTOR_ID     = 0x205,//ID 1 001
    CAN_PIT_MOTOR_ID     = 0x206,//ID 2 010
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_SUPERCAP_RECV    = 0x211,
    //发送ID
    CAN_CHASSIS_ID       = 0x200,
    CAN_SUPER_CAP_ID      = 0X210,
    CAN_GIMBAL_ID        = 0x1ff,
} CANMsgIDType;

/**
 * @brief    初始化CAN
 */
void CAN_Init();

/**
 * @brief             通过CAN发送数据
 * @param can         CAN1或CAN2
 * @param send_id     发送ID
 * @param send_data   发送的数据
 */
void CAN_Send(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]);

/**
 * @brief          CAN接收回调函数
 * @param hcan     CAN1或CAN2
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief         超级电容数据解算
 * @param data    接收到的数据
 */
void PowerDataResolve(uint8_t data[]);

/* 云台状态传输相关 */
/**
  * @brief     云台控制模式枚举
  */
typedef enum
{
    GIMBAL_INIT = 0,         //云台初始化
    GIMBAL_RELAX = 1,            //云台断电
    GIMBAL_CLOSE_LOOP_ZGYRO = 2, //云台跟随imu z轴角度
    GIMBAL_AUTO	= 3						 //云台自瞄模式
} GimbalModeType;

extern GimbalModeType gim_ctrl_mode;
extern float PowerData[4];
#endif //HNU_RM_DOWN_BSP_CAN_H
