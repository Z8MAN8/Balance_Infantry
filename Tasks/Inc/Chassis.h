//
// Created by 14685 on 2022/7/16.
//

#ifndef HNU_RM_DOWN_CHASSIS_H
#define HNU_RM_DOWN_CHASSIS_H



#include "stm32f4xx_hal.h"
#include "controller.h"
#include "motor.h"

/* 底盘控制周期 (ms) */
#define CHASSIS_PERIOD 2



/**
  * @brief     底盘控制模式枚举
  */
typedef enum
{
    CHASSIS_STOP,          //底盘停止
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_OPEN_LOOP,     //底盘开环
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_SPIN,          //底盘陀螺模式
    CHASSIS_FLY            //底盘飞坡模式
} ChassisModeType;

/**
  * @brief     底盘控制数据结构体
  */
typedef struct
{
    /* 底盘控制模式相关 */
    ChassisModeType  ctrl_mode;       //当前底盘控制模式
    ChassisModeType  last_mode;  //上次底盘控制模式

    /* 底盘移动速度相关数据 */
    float           vx;         //底盘前后速度
    float           vy;         //底盘左右速度
    float           vw;         //底盘旋转速度


    uint8_t         last_sw2;
} ChassisTypeDef;

/**
  * @brief     底盘控制参数初始化
  */
void Chassis_Init_param(void);

/**
  * @brief     底盘状态机，获取底盘控制模式
  */
void Chassis_Get_mode(void);

/**
 * @brief    底盘失能模式控制函数
 */
void Chassis_Relax_control(void);

/**
 * @brief    底盘静止模式控制函数
 */
void Chassis_Stop_control(void);

/**
 * @brief    底盘飞坡模式控制函数
 */
void Chassis_Fly_control(void);

/**
 * @brief    底盘不跟随云台模式控制函数
 */
void Chassis_Open_control(void);

/**
 * @brief    底盘跟随云台模式控制函数
 */
void Chassis_Follow_control(void);

/**
 * @brief    底盘失能模式控制函数
 */
void Chassis_Spin_control(void);

/**
  * @brief     底盘控制信息获取
  */
void Chassis_Get_control_information(void);

/**
 * @brief     底盘运动的速度分解，以及电机转速的闭环控制
 */
void Chassis_Custom_control(void);

/**
  * @brief     底盘速度分解，计算底盘每个轮子速度
  * @param     vx: 底盘前后速度
  * @param     vy: 底盘左右速度
  * @param     vw: 底盘旋转速度
  * @param     speed[]: 4 个轮子速度数组
  */
void Chassis_Calc_moto_speed(float vx, float vy, float vw, int16_t speed[]);

/**
  * @brief     底盘的运动逆运算求解实际速度
 */
void Chassis_Get_speed(void);

/**
  * @brief     底盘速度闭环处理计算函数
  */
void Chassis_Calculate_close_loop(void);

/**
 * @brief          发送底盘电机电流数据到电调
 * @param current  发送的电流数据
 */
void Chassis_Send_current(int16_t current[]);

/**
  * @brief     底盘陀螺处理函数
  */
void Chassis_Top_handle(void);




extern MotorTypeDef chassis_motor[4];
extern ChassisTypeDef chassis;
//底盘跟随云台PID结构体
extern PIDTypeDef rotate_follow;
extern float yaw_relative_angle;

extern unsigned char recv_flag;   //虚拟串口接收标志位


#endif //HNU_RM_DOWN_CHASSIS_H