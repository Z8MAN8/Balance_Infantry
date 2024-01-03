/**
 * @file chassis_task.h
 * @brief  底盘控制任务，在 robot 线程中调用
 * @date 2023-12-29
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

/**
 * @brief 底盘模式
 */
typedef enum
{
    CHASSIS_RELAX,         //底盘失能
    CHASSIS_INIT,           //底盘归中初始化
    CHASSIS_STOP,          //底盘停止
    CHASSIS_OPEN_LOOP,     //底盘开环
    CHASSIS_RECOVERY,      //底盘倒地自起
    CHASSIS_STAND_LOW,     //底盘站立低姿态
    CHASSIS_STAND_MID,     //底盘站立中姿态
    CHASSIS_STAND_HIG,     //底盘站立高姿态
    CHASSIS_OFF_GROUND,    //底盘离地
    CHASSIS_FOLLOW_GIMBAL, //底盘跟随云台
    CHASSIS_SPIN,          //底盘陀螺模式
    CHASSIS_FLY,           //底盘飞坡模式
    CHASSIS_AUTO           //底盘自动模式
} chassis_mode_e;

/**
  * @brief     底盘回中状态枚举
  */
typedef enum
{
    LEG_BACK_STEP = 0,          //腿部正在回中
    LEG_BACK_IS_OK,             //腿部回中完毕
}leg_state_e;

/**
 * @brief 底盘控制任务初始化
 */
void chassis_task_init(void);

/**
 * @brief 底盘控制任务,在RTOS中应该设定为200hz运行
 */
void chassis_control_task(void);

#endif // CHASSIS_TASK_H
