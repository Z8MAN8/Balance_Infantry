/**
  ******************************************************************************
  * @file    Motor.h
  * @author  Hongxi Wong
  * @version V1.2.1
  * @date    2021/4/13
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */

#ifndef _MOTOR_H
#define _MOTOR_H

#include <stdint.h>
#include "controller.h"
#include "can.h"

#define ENCODERCOEF 0.0439453125f

//CAN Transmit ID
#define CAN_Transmit_1_4_ID 0x200
#define CAN_Transmit_5_8_ID 0x1ff
#define CAN_Transmit_6020_ID 0x2ff

//CAN Receive ID
#define CAN_Receive_1_ID 0x201
#define CAN_Receive_2_ID 0x202
#define CAN_Receive_3_ID 0x203
#define CAN_Receive_4_ID 0x204
#define CAN_Receive_5_ID 0x205
#define CAN_Receive_6_ID 0x206
#define CAN_Receive_7_ID 0x207
#define CAN_Receive_8_ID 0x208

//CAN Gimbal ID
#define CAN_GIMBAL_Info_ID 0x666
#define CAN_GIMBAL_Control_ID_1 0x233
#define CAN_GIMBAL_Control_ID_2 0x404

#define NEGATIVE 1
#define FILTER_BUF 5
/*moto information receive from CAN*/
typedef struct motor_t
{
    int16_t Velocity_RPM;
    float Real_Current;
    uint8_t Temperature;

    uint8_t Direction;

    float Ke;

    float Angle; //abs angle range:[0,8191]
    float AngleInDegree;
    uint16_t RawAngle; //abs angle range:[0,8191]
    uint16_t last_angle;

    int16_t offset_angle;
    int32_t round_cnt;
    int32_t total_angle;

    int16_t zero_offset;

    int32_t msg_cnt;

    uint16_t CAN_ID;

    float Output;
    float Max_Out;

    PIDTypeDef PID_Torque;
    PIDTypeDef PID_Velocity;
    PIDTypeDef PID_Angle;

    Feedforward_t FFC_Torque;
    Feedforward_t FFC_Velocity;
    Feedforward_t FFC_Angle;

    LDOB_t LDOB;

    void (*TorqueCtrl_User_Func_f)(struct motor_t *motor);
    void (*SpeedCtrl_User_Func_f)(struct motor_t *motor);
    void (*AngleCtrl_User_Func_f)(struct motor_t *motor);
} __attribute__((packed)) MotorTypeDef;

/**
  * @brief     电机参数结构体
  */
typedef struct
{
    /* 以下是电机电调直接回传的数据 */

    uint16_t ecd;         //电机的编码器数值
    uint16_t last_ecd;    //上一次电机的编码器数值
    int16_t  speed_rpm;   //电机的转速值

    /* 以下是计算出来的电机相关数据 */
    int32_t  round_cnt;   //电机旋转的总圈数
    int32_t  total_ecd;   //电机旋转的总编码器数值
    int32_t  total_angle; //电机旋转的总角度

    /* 以下电机计算相关数据时的中间变量，可以忽略 */
    uint16_t offset_ecd;
    uint32_t msg_cnt;
    int32_t  ecd_raw_rate;
    int32_t  rate_buf[FILTER_BUF];
    uint8_t  buf_cut;
    int32_t  filter_rate;
} moto_measure_t;


float Motor_Torque_Calculate(MotorTypeDef *motor, float torque, float target_torque);
float Motor_Speed_Calculate(MotorTypeDef *motor, float velocity, float target_speed);
float Motor_Angle_Calculate(MotorTypeDef *motor, float angle, float velocity, float target_angle);

/**
  * @Func	    void get_moto_info(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief      process data received from CAN
  * @Param	    MotorTypeDef *ptr  CAN_HandleTypeDef *_hcan
  * @Retval	    None
  * @Date       2019/11/5
 **/
void get_moto_info(MotorTypeDef *ptr, uint8_t *aData);

void get_motor_offset(MotorTypeDef *ptr, uint8_t *aData);
#endif
