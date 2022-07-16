/**
  ******************************************************************************
  * @file    Motor.c
  * @author  Hongxi Wong
  * @version V1.2.1
  * @date    2021/4/13
  * @brief   
  ******************************************************************************
  * @attention 
  *
  ******************************************************************************
  */
#include "motor.h"

float Motor_Torque_Calculate(MotorTypeDef *motor, float torque, float target_torque)
{
    // 前馈控制
    Feedforward_Calculate(&motor->FFC_Torque, target_torque);
    // 反馈控制
    PID_Calculate(&motor->PID_Torque, torque, target_torque);

    if (motor->TorqueCtrl_User_Func_f != NULL)
        motor->TorqueCtrl_User_Func_f(motor);

    if (motor->Direction != NEGATIVE)
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output + motor->Ke * motor->Velocity_RPM;
    else
        motor->Output = motor->FFC_Torque.Output + motor->PID_Torque.Output - motor->Ke * motor->Velocity_RPM;
    // 输出限幅
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Speed_Calculate(MotorTypeDef *motor, float velocity, float target_speed)
{
    // 前馈控制
    Feedforward_Calculate(&motor->FFC_Velocity, target_speed);
    // 反馈控制
    PID_Calculate(&motor->PID_Velocity, velocity, target_speed);
    // 线性扰动观测器
    LDOB_Calculate(&motor->LDOB, velocity, motor->Output);

    if (motor->SpeedCtrl_User_Func_f != NULL)
        motor->SpeedCtrl_User_Func_f(motor);

    // 扰动补偿
    motor->Output = motor->FFC_Velocity.Output + motor->PID_Velocity.Output - motor->LDOB.Disturbance;
    // 输出限幅
    motor->Output = float_constrain(motor->Output, -motor->Max_Out, motor->Max_Out);

    return motor->Output;
}

float Motor_Angle_Calculate(MotorTypeDef *motor, float angle, float velocity, float target_angle)
{
    // 外环前馈控制
    Feedforward_Calculate(&motor->FFC_Angle, target_angle);
    // 外环反馈控制
    PID_Calculate(&motor->PID_Angle, angle, target_angle);

    if (motor->AngleCtrl_User_Func_f != NULL)
        motor->AngleCtrl_User_Func_f(motor);

    // 内环
    Motor_Speed_Calculate(motor, velocity, motor->FFC_Angle.Output + motor->PID_Angle.Output);

    return motor->Output;
}

void get_moto_info(MotorTypeDef *ptr, uint8_t *aData)
{
    // 详见C620电调手册
    if (ptr->Direction != NEGATIVE)
    {
        ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = (int16_t)(aData[2] << 8 | aData[3]);
    }
    else
    {
        ptr->RawAngle = 8191 - (uint16_t)(aData[0] << 8 | aData[1]);
        ptr->Velocity_RPM = -(int16_t)(aData[2] << 8 | aData[3]);
    }

    ptr->Real_Current = (int16_t)(aData[4] << 8 | aData[5]);
    ptr->Temperature = aData[6];

    if (ptr->RawAngle - ptr->last_angle > 4096)
        ptr->round_cnt--;
    else if (ptr->RawAngle - ptr->last_angle < -4096)
        ptr->round_cnt++;

    ptr->Angle = loop_float_constrain(ptr->RawAngle - ptr->zero_offset, -4095.5, 4095.5);

    ptr->AngleInDegree = ptr->Angle * 0.0439507f;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->RawAngle - ptr->offset_angle;

    ptr->last_angle = ptr->RawAngle; //update last_angle
}

void get_motor_offset(MotorTypeDef *ptr, uint8_t *aData)
{
    ptr->RawAngle = (uint16_t)(aData[0] << 8 | aData[1]);
    ptr->offset_angle = ptr->RawAngle;
    ptr->last_angle = ptr->RawAngle; //update last_angle
}
