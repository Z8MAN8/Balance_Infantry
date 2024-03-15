/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H



/**
 * @brief 云台模式
 */
typedef enum
{
    GIMBAL_RELAX = 0,        //云台断电
    GIMBAL_INIT = 1,         //云台初始化
    GIMBAL_GYRO = 2,         //云台跟随imu闭环
    GIMBAL_AUTO = 3          //云台自瞄
} gimbal_mode_e;

/**
  * @brief     云台回中状态枚举
  */
typedef enum
{
    BACK_STEP = 0,             //云台正在回中
    BACK_IS_OK = 1,            //云台回中完毕
} gimbal_back_e;
/**
 * @brief 底盘控制任务初始化
 */
void gimbal_task_init(void);
/**
 * @brief 底盘控制任务,在RTOS中应该设定为200hz运行
 */
void gimbal_control_task(void);

#endif /* _GIMBAL_TASK_H */
