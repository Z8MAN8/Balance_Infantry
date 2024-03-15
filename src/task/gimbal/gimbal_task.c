/*
* Change Logs:
* Date            Author          Notes
* 2023-09-25      ChuShicheng     first version
*/
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"

#define GIM_MOTOR_NUM 2
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins_data;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gim_cmd;

// 发布
MCN_DECLARE(gimbal_fdb);
static struct gimbal_fdb_msg gim_fdb;



static void gimbal_sub_init(void);
static void gimbal_pub_push(void);
static void gimbal_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */
/* [0]为yaw，[1]为pitch */
#define YAW 0
#define PITCH 1
/* gyro三轴：[0]为X，[1]为Y，[2]为Z */
#define X 0
#define Y 1
#define Z 2
static struct gimbal_controller_t{
    /* 基于imu数据闭环，主要用于手动模式 */
    pid_obj_t *pid_speed_imu;
    pid_obj_t *pid_angle_imu;
    /* 基于imu数据闭环，主要用于自动模式 */
    pid_obj_t *pid_speed_auto;
    pid_obj_t *pid_angle_auto;
}gim_controller[GIM_MOTOR_NUM];

motor_config_t gimbal_motor_config[GIM_MOTOR_NUM] = {
        {
                .motor_type = GM6020,
                .can_id =2,
                .rx_id = YAW_MOTOR_ID,
                .controller = &gim_controller[YAW],
        },
        {
                .motor_type = GM6020,
                .can_id =2,
                .rx_id = PITCH_MOTOR_ID,
                .controller = &gim_controller[PITCH],
        }
};

static int16_t yaw_motor_relive, pitch_motor_relive;  // 电机相对于归中值的角度

static ramp_obj_t *yaw_ramp;//yaw 轴云台控制斜坡
static ramp_obj_t *pit_ramp;//pitch 轴云台控制斜坡

static dji_motor_object_t *gim_motor[GIM_MOTOR_NUM];  // 底盘电机实例
 float gim_motor_ref[GIM_MOTOR_NUM]; // 电机控制期望值

static void gimbal_motor_init();
static int16_t motor_control_yaw(dji_motor_measure_t measure);
static int16_t motor_control_pitch(dji_motor_measure_t measure);
static int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset);


/* --------------------------------- 云台线程入口 --------------------------------- */
static float gim_dt;

void gimbal_task_init(void) {
    gimbal_sub_init();
    gimbal_motor_init();
    yaw_ramp = ramp_register(0, BACK_CENTER_TIME/GIMBAL_PERIOD);
    pit_ramp = ramp_register(0, BACK_CENTER_TIME/GIMBAL_PERIOD);
}
void gimbal_control_task(void){

        static float gim_start;
        gim_start = dwt_get_time_ms();
        /* 更新该线程所有的订阅者 */
        gimbal_sub_pull();
        //因为平步和全向轮云台的C板安装方向不同，故传感器数据需要进行改动
        ins_data.gyro_gim[Y]=-ins_data.gyro_gim[Y];
        ins_data.gyro_gim[Z]=ins_data.gyro_gim[Z];
        ins_data.yaw_gim=ins_data.yaw_gim;
        ins_data.pitch_gim=-(ins_data.pitch_gim);
        ins_data.yaw_total_angle_gim=ins_data.yaw_total_angle_gim;
//    ins.gyro[Y]=-ins.gyro[Y];
//    ins.gyro[Z]=ins.gyro[Z];
//    ins.yaw=ins.yaw;
//    ins.pitch=-(ins.pitch);
//    ins.yaw_total_angle=ins.yaw_total_angle;


        // 云台本身相对于归中值的角度，加负号
//        yaw_motor_relive = -get_relative_pos(gim_motor[YAW]->measure.ecd, CENTER_ECD_YAW) / 22.75f;
//        pitch_motor_relive = get_relative_pos(gim_motor[PITCH]->measure.ecd, CENTER_ECD_PITCH) / 22.75f;
    yaw_motor_relive = -get_relative_pos(gim_motor[YAW]->measure.ecd, CENTER_ECD_YAW) / 22.75f;
    pitch_motor_relive = -get_relative_pos(gim_motor[PITCH]->measure.ecd, CENTER_ECD_PITCH) / 22.75f;

        for (uint8_t i = 0; i < GIM_MOTOR_NUM; i++)
        {
            dji_motor_enable(gim_motor[i]);
        }

        switch (gim_cmd.ctrl_mode)
        {
        case GIMBAL_RELAX:
            for (uint8_t i = 0; i < GIM_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor[i]);
            }
            gim_fdb.back_mode = BACK_STEP;
            yaw_ramp->reset(yaw_ramp, 0, BACK_CENTER_TIME/GIMBAL_PERIOD);
            pit_ramp->reset(pit_ramp, 0, BACK_CENTER_TIME/GIMBAL_PERIOD);

            break;
        case GIMBAL_INIT:
            // TODO：加入斜坡算法，可以控制归中时间
            // TODO: 将编码器值转化为角度值
            // TODO: 优化归中逻辑，yaw轴选取最近的方向
            gim_motor_ref[YAW] = yaw_motor_relive * ( 1 - yaw_ramp->calc(yaw_ramp));
            gim_motor_ref[PITCH] = pitch_motor_relive* ( 1 - pit_ramp->calc(pit_ramp));


                if((abs(gim_motor[YAW]->measure.ecd - CENTER_ECD_YAW) <= 80))
                    if((abs(gim_motor[PITCH]->measure.ecd - CENTER_ECD_PITCH) <= 10))
            {
                gim_fdb.back_mode = BACK_IS_OK;
                gim_fdb.yaw_offset_angle_total = ins_data.yaw_total_angle;/*云台抽风的原因，期望应该为总角度。抽风原因：不应该用ins_data.yaw*/
                gim_fdb.yaw_offset_angle=ins_data.yaw;
                gim_fdb.pit_offset_angle = ins_data.pitch;

            }
            else
            {
                gim_fdb.back_mode = BACK_STEP;
            }
            break;
        case GIMBAL_GYRO:
            gim_motor_ref[YAW] = gim_cmd.yaw;
            gim_motor_ref[PITCH] = gim_cmd.pitch;
            // 底盘相对于云台归中值的角度，取负
            gim_fdb.yaw_relative_angle = -yaw_motor_relive;
            break;
        // TODO: add auto mode
        case GIMBAL_AUTO:
                /*gim_motor_ref[YAW] = gim_cmd.yaw_auto;*/
            gim_motor_ref[YAW] =gim_cmd.yaw;
            gim_motor_ref[PITCH] =gim_cmd.pitch;
            // 底盘相对于云台归中值的角度，取负
            gim_fdb.yaw_relative_angle = -yaw_motor_relive;
            break;
        default:
            for (uint8_t i = 0; i < GIM_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor[i]);
            }
            break;
        }

        /* 更新发布该线程的msg */
        gimbal_pub_push();

        /* 用于调试监测线程调度使用 */
        gim_dt = dwt_get_time_ms() - gim_start;
        if (gim_dt > 1) {

//        rt_thread_delay(1);
//            dwt_delay_s(0.001);
        }

}


/**
 * @brief gimbal 线程中所有订阅者初始化
 */
static void gimbal_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
}

/**
 * @brief gimbal 线程中所有发布者推送更新话题
 */
static void gimbal_pub_push(void)
{
    mcn_publish(MCN_HUB(gimbal_fdb), &gim_fdb);
}

/**
 * @brief gimbal 线程中所有订阅者获取更新话题
 */
static void gimbal_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins_data);
    }

    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gim_cmd);
    }
}

/**
 * @brief 注册云台电机及其控制器初始化
 */
static void gimbal_motor_init()
{
/* ----------------------------------- yaw ---------------------------------- */
    pid_config_t yaw_speed_imu_config = INIT_PID_CONFIG(YAW_KP_V_IMU, YAW_KI_V_IMU, YAW_KD_V_IMU, YAW_INTEGRAL_V_IMU, YAW_MAX_V_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_imu_config = INIT_PID_CONFIG(YAW_KP_A_IMU, YAW_KI_A_IMU, YAW_KD_A_IMU, YAW_INTEGRAL_A_IMU, YAW_MAX_A_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: 自瞄模式参数待调
    pid_config_t yaw_speed_auto_config = INIT_PID_CONFIG(YAW_KP_V_AUTO, YAW_KI_V_AUTO, YAW_KD_V_AUTO, YAW_INTEGRAL_V_AUTO, YAW_MAX_V_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_auto_config = INIT_PID_CONFIG(YAW_KP_A_AUTO, YAW_KI_A_AUTO, YAW_KD_A_AUTO, YAW_INTEGRAL_A_AUTO, YAW_MAX_A_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[YAW].pid_speed_imu = pid_register(&yaw_speed_imu_config);
    gim_controller[YAW].pid_angle_imu = pid_register(&yaw_angle_imu_config);
    gim_controller[YAW].pid_speed_auto = pid_register(&yaw_speed_auto_config);
    gim_controller[YAW].pid_angle_auto = pid_register(&yaw_angle_auto_config);
    gim_motor[YAW] = dji_motor_register(&gimbal_motor_config[YAW], motor_control_yaw);

/* ---------------------------------- pitch --------------------------------- */
    pid_config_t pitch_speed_imu_config = INIT_PID_CONFIG(PITCH_KP_V_IMU, PITCH_KI_V_IMU, PITCH_KD_V_IMU, PITCH_INTEGRAL_V_IMU, PITCH_MAX_V_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_imu_config = INIT_PID_CONFIG(PITCH_KP_A_IMU, PITCH_KI_A_IMU, PITCH_KD_A_IMU, PITCH_INTEGRAL_A_IMU, PITCH_MAX_A_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: 自瞄模式参数待调
    pid_config_t pitch_speed_auto_config = INIT_PID_CONFIG(PITCH_KP_V_AUTO, PITCH_KI_V_AUTO, PITCH_KD_V_AUTO, PITCH_INTEGRAL_V_AUTO, PITCH_MAX_V_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_auto_config = INIT_PID_CONFIG(PITCH_KP_A_AUTO, PITCH_KI_A_AUTO, PITCH_KD_A_AUTO, PITCH_INTEGRAL_A_AUTO, PITCH_MAX_A_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[PITCH].pid_speed_imu = pid_register(&pitch_speed_imu_config);
    gim_controller[PITCH].pid_angle_imu = pid_register(&pitch_angle_imu_config);
    gim_controller[PITCH].pid_speed_auto = pid_register(&pitch_speed_auto_config);
    gim_controller[PITCH].pid_angle_auto = pid_register(&pitch_angle_auto_config);
    gim_motor[PITCH] = dji_motor_register(&gimbal_motor_config[PITCH], motor_control_pitch);
}


int16_t send_data_temp_yaw;
float get_speed_yaw, get_angle_yaw;
float pid_out_angle_yaw;
static int16_t motor_control_yaw(dji_motor_measure_t measure){
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static int16_t send_data;        // 最终发送给电调的数据

    switch (gim_cmd.ctrl_mode)
    {
        // TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
        case GIMBAL_INIT:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_speed = ins_data.gyro[Z];//不用变
            get_angle = yaw_motor_relive;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_speed = ins_data.gyro[Z];
            get_angle = ins_data.yaw_total_angle - gim_fdb.yaw_offset_angle_total;

            get_speed_yaw=get_speed;
            get_angle_yaw=get_angle;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[YAW].pid_speed_auto;
            pid_angle = gim_controller[YAW].pid_angle_auto;
            get_speed = ins_data.gyro[Z];
            get_angle = ins_data.yaw_total_angle - gim_fdb.yaw_offset_angle_total;

            break;
        default:
            break;
    }
    /* 切换模式需要清空控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    if(gim_cmd.ctrl_mode == GIMBAL_INIT)  // 编码器闭环
    {
        /* 注意负号 */
//        send_data = 5000;//往左转

        pid_out_angle = -pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);//转向不对？
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);      // 电机转动正方向与imu相反


    }
    else if(gim_cmd.ctrl_mode == GIMBAL_GYRO || gim_cmd.ctrl_mode == GIMBAL_AUTO ) /* imu闭环 */
    {
        /* 注意负号 */
        pid_out_angle = -pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);      // 电机转动正方向与imu相反


        pid_out_angle_yaw=pid_out_angle;

    }
    else{
        send_data=0;
    }

    send_data_temp_yaw=send_data;
    return send_data;
}

//前期调试用

int16_t send_data_temp_pitch;
float get_speed_pitch, get_angle_pitch;
float pid_out_angle_pitch;
float ref=0;
static int16_t motor_control_pitch(dji_motor_measure_t measure){
    /* PID局部指针，切换不同模式下PID控制器 */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // 闭环反馈量
    static float pid_out_angle;         // 角度环输出
    static int16_t send_data;        // 最终发送给电调的数据

    switch (gim_cmd.ctrl_mode)
    {
        /* 根据云台模式，切换对应的控制器及观测量 */
        case GIMBAL_INIT:// TODO: 云台初始化模式加入斜坡算法，可以控制归中时间
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            get_speed = ins_data.gyro[Y];

            get_angle = pitch_motor_relive;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            get_speed = ins_data.gyro[Y];
            get_angle = ins_data.pitch;


            get_speed_pitch=get_speed;
            get_angle_pitch=get_angle;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[PITCH].pid_speed_auto;
            pid_angle = gim_controller[PITCH].pid_angle_auto;
            get_speed = ins_data.gyro[Y];
            get_angle = ins_data.pitch;

            break;
        default:
            break;
    }
    /* 切换模式需要清空控制器历史状态 */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        pid_clear(pid_angle);
        pid_clear(pid_speed);
    }

    if(gim_cmd.ctrl_mode == GIMBAL_INIT)  // 编码器闭环
    {
        /*串级pid的使用，角度环套在速度环上面*/
        /* 注意负号 */
//                send_data = 5000;//往下

       pid_out_angle = -pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);//方向错误
       send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);      // 电机转动正方向与imu相反

    }
    else if(gim_cmd.ctrl_mode == GIMBAL_GYRO || gim_cmd.ctrl_mode == GIMBAL_AUTO )  /* imu闭环 */
    {
        /* 限制云台俯仰角度 */
        VAL_LIMIT(gim_motor_ref[PITCH], PIT_ANGLE_MIN, PIT_ANGLE_MAX);
        /* 注意负号 */

        pid_out_angle = -pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);      // 电机转动正方向与imu相反


    }
    else{
        send_data=1000;
    }

    pid_out_angle_pitch=pid_out_angle;
    send_data_temp_pitch=send_data;
    return send_data;
}

/**
 * @brief Get the relative pos object
 *
 * @param raw_ecd  实际的编码器值
 * @param center_offset 相对的参考编码器值
 * @return rt_int16_t
 */

//GM6020编码器范围:0~8191
int16_t get_relative_pos(int16_t raw_ecd, int16_t center_offset)
{
    int16_t tmp = 0;
    if (center_offset >= 4096){
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else{
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}
