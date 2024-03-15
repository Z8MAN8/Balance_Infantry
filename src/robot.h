#ifndef ROBOT_H
#define ROBOT_H

#include "rm_module.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"

/**
 * @brief 机器人初始化,请在开启rtos之前调用
 * 
 */
void robot_init();

/**
 * @brief 机器人任务,放入实时系统以一定频率运行,内部会调用各个应用的任务
 * 
 */
void robot_task();

/* ------------------------------- ipc uMCN 相关 ------------------------------ */
struct ins_msg
{
    // IMU量测值
    float gyro[3];  // 角速度
    float accel[3]; // 加速度
    float motion_accel_b[3]; // 机体坐标加速度
    // 位姿
    float roll;
    float pitch;
    float yaw;
    float yaw_total_angle;

    float gyro_gim[3];  // 角速度
    float accel_gim[3]; // 加速度
    float motion_accel_b_gim[3]; // 机体坐标加速度
    // 位姿
    float roll_gim;
    float pitch_gim;
    float yaw_gim;
    float yaw_total_angle_gim;
};

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief cmd发布的底盘控制数据,由chassis订阅
 */
struct chassis_cmd_msg
{
    float vx;                  // 前进方向速度
    float vy;                  // 横移方向速度
    float vw;                  // 旋转速度
    // TODO: 轮腿前期调试使用
    float leg_length;          // 腿长
    float leg_angle;           // 腿角度
    float offset_angle;        // 底盘和归中位置的夹角
    leg_state_e leg_state;     // 腿部归中初始化情况
    chassis_mode_e ctrl_mode;  // 当前底盘控制模式
    chassis_mode_e last_mode;  // 上一次底盘控制模式
};

/**
 * @brief cmd发布的云台控制数据,由gimbal订阅
 */
struct gimbal_cmd_msg
{ // 云台期望角度控制
    float yaw;
    float pitch;
    gimbal_mode_e ctrl_mode;  // 当前云台控制模式
    gimbal_mode_e last_mode;  // 上一次云台控制模式
};

/**
 * @brief cmd发布的云台控制数据,由shoot订阅
 */
struct shoot_cmd_msg
{ // 发射器
    shoot_mode_e ctrl_mode;  // 当前发射器控制模式
    shoot_mode_e last_mode;  // 上一次发射器控制模式
    trigger_mode_e trigger_status;
    int16_t shoot_freq;      // 发射弹频
    // TODO: 添加发射弹速控制
    int16_t shoot_speed;     // 发射弹速
    uint8_t cover_open;      // 弹仓盖开关
};

/* ------------------------------ gimbal反馈状态数据 ------------------------------ */
/**
 * @brief 云台真实反馈状态数据,由gimbal发布
 */
struct gimbal_fdb_msg
{
    gimbal_back_e back_mode;  // 云台归中情况

    float yaw_offset_angle_total;    //云台初始 yaw 轴角度 （由imu得）
    float yaw_offset_angle;    //云台初始 yaw 轴角度 （由imu得）
    float pit_offset_angle;    //云台初始 pit 轴角度 （由imu得）
    float yaw_relative_angle;  //云台相对于初始位置的yaw轴角度
};

/* ------------------------------ shoot反馈状态数据 ------------------------------ */
/**
 * @brief 发射机真实反馈状态数据,由shoot发布
 */
struct shoot_fdb_msg
{
    shoot_back_e shoot_mode;  // shoot状态反馈
    int16_t trigger_motor_current; //拨弹电机电流，传给cmd控制反转
};


/* ------------------------------ chassis反馈状态数据 ------------------------------ */
/**
 * @brief 底盘真实反馈状态数据,由chassis发布
 */
struct chassis_fdb_msg
{
    enum leg_state_e leg_state;  // 腿部归中初始化情况
    /*  底盘任务使用到的电机句柄,仅能对其 measure 成员当作传感器数据读取，禁止改写 */
    lk_motor_measure_t lk_l;   // 左轮毂电机
    lk_motor_measure_t lk_r;   // 右轮毂电机
    bool touch_ground;         // 是否触地
};

/* ------------------------------ trans解析自瞄数据 ------------------------------ */
/**
 * @brief 上位机自瞄数据,由trans发布
 */
struct trans_fdb_msg
{
   float yaw;
   float pitch;
};

#endif