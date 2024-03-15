/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
*/
#include "chassis_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"

//#define CLOSE_ALL_MOTOR
//#define SLIP_DETECTION // 是否使能打滑检测
/* -------------------------------- 线程间通讯话题相关 ------------------------------- */
// 订阅
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chassis_cmd;
// 发布
MCN_DECLARE(chassis_fdb);
static struct chassis_fdb_msg chassis_fdb_data;

static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);

/* --------------------------------- 电机控制相关 --------------------------------- */
#define LEFT    0
#define RIGHT   1
#define FRONT   0
#define BACK    1
#define LEFT_FRONT  0
#define RIGHT_FRONT 1
#define RIGHT_BACK  2
#define LEFT_BACK   3
#define LEN_LOW     0.16f // 单位：m
#define LEN_MID     0.24f // 单位：m
#define LEN_HIG     0.32f // 单位：m
/* 髋关节电机零点偏移 需要提前测出 */
#define HT_OFFSET_LF -0.46485f  // 左上电机 -0.3279
#define HT_OFFSET_RF  0.44445f  // 右上电机 0.3366
#define HT_OFFSET_LB  0.44154f  // 左下电机 0.3221
#define HT_OFFSET_RB -0.45280f  // 右下电机 -0.3366
#define ROLL_OFFSET  -0.28f     // TODO：目前C板陀螺仪安装误差，后续可能变化
//#define LEG_LENGHT_F  15.0f     // TODO: 腿长控制前馈，根据机体质量调整，加云台前数据
#define LEG_LENGHT_F  20.0f     // TODO: 腿长控制前馈，根据机体质量调整
#define LEG_MASS      0.824f    // 腿部质量
/*跳跃相关*/
#define JUMP_TORQUE_PRESS    30.0f   // 跳跃时下压扭矩，正负需对应各电机确定
#define JUMP_TORQUE_SHRINK   22.0f   // 跳跃时回缩扭矩，正负需对应各电机确定
/* 髋关节电机扭矩限制 */
#define HT_OUTPUT_LIMIT 35.0f//当时最后用的
//#define HT_OUTPUT_LIMIT 20.0f//加云台前
//#define HT_OUTPUT_LIMIT 0//当时最后用的
#define HT_INIT_OUT 2.0f   // 电机初始化时的输出扭矩,确保能撞到限位
#define LK_OUTPUT_LIMIT 4.5f
//#define LK_OUTPUT_LIMIT 0
#define LK_TOR_TO_CUR 406.21344  // LK9025 扭矩转换电流系数
/* 整体运动限制 */
#define POSITION_X_LIMIT 0.5f  // LQR控制时的位移限制 m
#define VELOCITY_X_LIMIT 4.0f  // LQR控制时的位移速度限制 m/s
/* 当前lk9025使用多电机模式控制 */

static pid_obj_t *follow_pid; // 用于底盘跟随云台计算vw
static pid_obj_t *theta_pid;  // 双腿角度协调控制
static pid_obj_t *yaw_pid;    // 航向角控制， 输出为vx的补偿，与vx期望累积
static pid_obj_t *roll_pid;   // 横滚角控制
static float yaw_target;
static float pos_x_offset[2]; // 底盘x位移的偏置，倒地自起后重置

static void leg_calc();
static float vx_change_limit(float vx_cmd, float vx_now);

/* 没有加入LQR前测试VMC用 */
static struct leg_controller_t{
    pid_obj_t *length_pid;
    pid_obj_t *angle_pid;
}leg_controller[2];

static leg_obj_t *leg[2] = {NULL};
static float ft_l[2], ft_r[2];  // FT = [PendulumForce PendulumTorque]，[沿腿的推力 沿五连杆中心轴的力矩]
static float vmc_out_l[2];  // vmc计算得出的扭矩值 左边腿
static float vmc_out_r[2];  // vmc计算得出的扭矩值 右边腿
static float jump_out_l[2]; // 跳跃附加扭矩值 左边腿
static float jump_out_r[2]; // 跳跃附加扭矩值 右边腿
static float jump_start_time;  // 跳跃开始时间
static float jump_dt_time;     // 起跳后的间隔时间
static float is_jumping;       // 跳跃标志位
static float jump_flag;        // 切入跳跃模式只跳一次
static float support_force[2];  //左右腿的地面支持力 用于离地检测
static float length_ref = 0.16f;
static float angle_ref = /*1.57f*/1.57f;

/* 髋关节电机 HT04 实例 */
static ht_motor_object_t *ht_motor[4];
/* 驱动电机 LK9025 实例 */
static lk_motor_object_t *lk_motor[2];

static int chassis_motor_init(void);
/* 髋关节电机初始化，撞限位，设置零点 */
/**
 * @brief 髋关节电机初始化,零点获取
 * @note ht04为单编码器，每次上电通过撞限位设置零点并减去固定偏移量
 */
static void leg_init_get_zero();
/* 使能底盘所有电机 */
static void motor_enable();
/* 失能底盘所有电机 */
static void motor_relax();

/* --------------------------------- LQR控制相关 -------------------------------- */
/* LQR 反馈矩阵（由 matlab 生成），3组分别对应不同腿长 */
//Bot :LQRKbuf[0][]
//Mid :LQRKbuf[1][]
//Bot :LQRKbuf[2][]
/*static float LQR_k[3][12]=
        {
                //Order: K00 K01 K02 K03 K04 K05 K10 K11 K12 K13 K14 K15
                {-10.3992,  -0.9331,  -0.4549,  -1.7546,   7.2587,   0.9680,   15.3052,   1.1763,   0.9337,   3.4995,  13.9384,   0.9096},	//K BOT 160
                {-13.7472,  -1.4827,  -0.5207,  -2.0187,   5.9456,   0.8669,   15.6191,   1.2992,   0.6813,   2.5643,  17.3396,   1.3307},	//K MID 240
                {-16.1640,  -2.0019,  -0.5512,  -2.1502,   4.9823,   0.7846,   15.2672,   1.3208,   0.5151,   1.9467,  19.1084,   1.56559},	//K TOP 320
        }; // 700hz Q=diag([100 1 100 1000 5000 1]) R=[240 0;0 25]*/
//static float LQR_k[3][12]=
//        {
//                {-13.5435,  -0.9888,  -1.0915,  -1.7668,   8.2780,   1.1123,   22.4086,   1.4836,   2.6957,   4.0920,  14.7195,   1.1273},	//K BOT 160
//                {-17.8858,  -1.5890,  -1.2950,  -2.1111,   6.7203,   0.9709,   22.3946,   1.6363,   1.9748,   3.0152,  19.5775,   1.7395},	//K MID 240
//                {-20.6655,  -2.1190,  -1.3653,  -2.2479,   5.5022,   0.8466,   21.0502,   1.6104,   1.4511,   2.2234,  21.9892,   2.0712},	//K TOP 320
//        }; // Ts=1.424ms Q=diag([1 1 500 100 5000 1]) R=[240 0;0 25]
static float LQR_k[3][12]=
        {
                {-14.8670,   -1.0361,   -1.1047,   -1.8466,    8.7726,    1.1942,   25.5826,    1.6609,    2.8783,    4.5272,   14.7643,    1.1838},	//K BOT 160
                {-19.3345,   -1.6403,   -1.2814,   -2.1545,    7.0545,    1.0217,   24.8938,    1.7897,    2.0581,    3.2504,   20.0129,    1.8501},	//K MID 240
                {-22.3391,   -2.1924,   -1.3580,   -2.3039,    5.7755,    0.8876,   23.3362,    1.7671,    1.5156,    2.4006,   22.6439,    2.2150},	//K TOP 320
        }; // Ts=1.424ms Q=diag([1 1 500 100 5000 5]) R=[240 0;0 25]//只改了k0
//static float LQR_k[3][12]=
//        {
//                {-14.8835,   -1.0376,   -1.1069,   -1.8502,    8.7680,    1.1975,   25.5319,    1.6562,    2.8701,    4.5137,   14.8088,    1.2065},	//K BOT 160
//                {-19.3345,   -1.6403,   -1.2814,   -2.1545,    7.0545,    1.0217,   25.2985,    1.6381,    2.8471,    4.4824,   15.8202,    1.2645},	//K MID 240
//                {-22.3391,   -2.1924,   -1.3580,   -2.3039,    5.7755,    0.8876,   23.3362,    1.7671,    1.5156,    2.4006,   22.6439,    2.2150},	//K TOP 320
//        }; // Ts=1.424ms Q=diag([1 1 500 100 5000 5]) R=[240 0;0 25]//只改了k0


/* [T Tp(髋)] */
static float LQROutBuf[2][2]={0};
static float LQRXerrorBuf[2][6]={0};
static float LQRXObsBuf[2][6]={0};
static float LQRXRefBuf[2][6]={0};

static arm_matrix_instance_f32 MatLQRObs_L  = {6, 1, LQRXObsBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRObs_R  = {6, 1, LQRXObsBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRRef_L  = {6, 1, LQRXRefBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRRef_R  = {6, 1, LQRXRefBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRNegK = {2, 6, (float*)LQR_k[0]};
static arm_matrix_instance_f32 MatLQRErrX_L = {6, 1, LQRXerrorBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRErrX_R = {6, 1, LQRXerrorBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQROutU_L = {2, 1, LQROutBuf[LEFT]};
static arm_matrix_instance_f32 MatLQROutU_R = {2, 1, LQROutBuf[RIGHT]};

/*Calculate X. Output is u (T,Tp)`*/
static void LQR_cal()
{
    //Calculate error
    arm_mat_sub_f32(&MatLQRObs_L, &MatLQRRef_L, &MatLQRErrX_L);
    //Calculate error
    arm_mat_sub_f32(&MatLQRObs_R, &MatLQRRef_R, &MatLQRErrX_R);

    // TODO:加入斜坡控制，避免较大振荡, 目前的限制是不合理的，反而会让系统无法快速收敛
    // 对期望值进行限幅
/*    LIMIT_MIN_MAX(LQRXerrorBuf[LEFT][2], -POSITION_X_LIMIT, POSITION_X_LIMIT);
    LIMIT_MIN_MAX(LQRXerrorBuf[RIGHT][2], -POSITION_X_LIMIT, POSITION_X_LIMIT);
    LIMIT_MIN_MAX(LQRXerrorBuf[LEFT][3], -VELOCITY_X_LIMIT, VELOCITY_X_LIMIT);
    LIMIT_MIN_MAX(LQRXerrorBuf[RIGHT][3], -VELOCITY_X_LIMIT, VELOCITY_X_LIMIT);*/

    //Calculate output value
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_L, &MatLQROutU_L);
    //Calculate output value
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_R, &MatLQROutU_R);
}

static pid_obj_t *length_pid[2];

/* ------------------------------- 打滑检测卡尔曼滤波部分 ------------------------------ */
//TODO:后续考虑移入观测线程，建立整车观测器
static KalmanFilter_t chassis_kf[2];
static float *observe_data[2]; // 卡尔曼输出矩阵的地址
static float observe_vx[2], observe_ax; // 观测后得到的数据

/**
*   |    vx     |
*   |   dvx     |
*/
static void chassis_kf_init(void)
{
    static float P_Init[4] =
            {
                    10, 0,
                    0, 30
            };
    static float F_Init[4] =
            {
                    1, 0.001,
                    0, 1
            };
    static float Q_Init[4] =
            {
                    1, 0,
                    0, 1,
            };

    // 设置最小方差
    static float state_min_variance[2] = {0.005, 0.1};

    // 开启自动调整
    chassis_kf[LEFT].UseAutoAdjustment = 1;

    // 电机编码器测得速度，加速度计测得x轴运动加速度
    static uint8_t measurement_reference[2] = {1,2};

    static float measurement_degree[2] = {1,1};
    // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）

    static float mat_R_diagonal_elements[2] = {800000,2000}; // 方差即为影响置信度，方差越大置信度越低  /* 打滑检测：800000 1000 */1
    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）

    Kalman_Filter_Init(&chassis_kf[LEFT], 2, 0, 2);

    // 设置矩阵值
    memcpy(chassis_kf[LEFT].P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf[LEFT].F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf[LEFT].Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf[LEFT].MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf[LEFT].MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf[LEFT].MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf[LEFT].StateMinVariance, state_min_variance, sizeof(state_min_variance));

    chassis_kf[RIGHT].UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf[RIGHT], 2, 0, 2);

    // 设置矩阵值
    memcpy(chassis_kf[RIGHT].P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf[RIGHT].F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf[RIGHT].Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf[RIGHT].MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf[RIGHT].MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf[RIGHT].MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf[RIGHT].StateMinVariance, state_min_variance, sizeof(state_min_variance));
}

static void chassis_kf_update(void)
{
    chassis_kf[LEFT].MeasuredVector[0] = lk_motor[LEFT]->measure.speed_rads * WHEEL_RADIUS + leg[LEFT]->d_phi0*leg[LEFT]->PendulumLength - ins.gyro[2] * 0.25f;
    chassis_kf[LEFT].MeasuredVector[1] = ins.motion_accel_b[0];
    observe_data[LEFT] = Kalman_Filter_Update(&chassis_kf[LEFT]);
    observe_vx[LEFT] = *observe_data[LEFT];
//    observe_ax = *(observe_data + 1);

    chassis_kf[RIGHT].MeasuredVector[0] = lk_motor[RIGHT]->measure.speed_rads * WHEEL_RADIUS + leg[RIGHT]->d_phi0*leg[LEFT]->PendulumLength + ins.gyro[2] * 0.25f;
    chassis_kf[RIGHT].MeasuredVector[1] = ins.motion_accel_b[0];
    observe_data[RIGHT] = Kalman_Filter_Update(&chassis_kf[RIGHT]);
    observe_vx[RIGHT] = *observe_data[RIGHT];
}

/* --------------------------------- 底盘运动学解算 --------------------------------- */
/* 根据宏定义选择的底盘类型使能对应的解算函数 */
#ifdef BSP_CHASSIS_OMNI_MODE
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
static void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = omni_calc;
#endif /* BSP_CHASSIS_OMNI_MODE */
#ifdef BSP_CHASSIS_MECANUM_MODE
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed);
void (*chassis_calc_moto_speed)(struct chassis_cmd_msg *cmd, int16_t* out_speed) = mecanum_calc;
#endif /* BSP_CHASSIS_MECANUM_MODE */
static void absolute_cal(struct chassis_cmd_msg *cmd, float angle);
/**
 * @brief 底盘跳跃处理
 */
static void jumping_control(void);

/* --------------------------------- 底盘线程入口 --------------------------------- */
static float chassis_dt;

static void update_LQR_obs() {
    /*  更新观测矩阵 */
    LQRXObsBuf[LEFT][0] = PI / 2 - leg[LEFT]->PendulumRadian - ins.pitch * DEGREE_2_RAD;
    LQRXObsBuf[LEFT][1] = -leg[LEFT]->d_phi0 - ins.gyro[1] * DEGREE_2_RAD;
    LQRXObsBuf[LEFT][2] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[LEFT];
#ifdef SLIP_DETECTION
    LQRXObsBuf[LEFT][3] = observe_vx[LEFT];
#else
    LQRXObsBuf[LEFT][3] = lk_motor[LEFT]->measure.speed_rads * WHEEL_RADIUS;
#endif /*SLIP_DETECTION*/
    LQRXObsBuf[LEFT][4] = ins.pitch * DEGREE_2_RAD/* + 0.064*/;
    LQRXObsBuf[LEFT][5] = ins.gyro[1] * DEGREE_2_RAD;

    LQRXObsBuf[RIGHT][0] = PI / 2 - leg[RIGHT]->PendulumRadian - ins.pitch * DEGREE_2_RAD;;
    LQRXObsBuf[RIGHT][1] = -leg[RIGHT]->d_phi0 - ins.gyro[1] * DEGREE_2_RAD;
    LQRXObsBuf[RIGHT][2] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[RIGHT];
#ifdef SLIP_DETECTION
    LQRXObsBuf[RIGHT][3] = observe_vx[RIGHT];
#else
    LQRXObsBuf[RIGHT][3] = lk_motor[RIGHT]->measure.speed_rads * WHEEL_RADIUS;
#endif /*SLIP_DETECTION*/
    LQRXObsBuf[RIGHT][4] = ins.pitch * DEGREE_2_RAD;
    LQRXObsBuf[RIGHT][5] = ins.gyro[1] * DEGREE_2_RAD;

    //计算位置目标
    //  TODO：目前位移项error一直为0，并没有有效利用，需要完善
/*    LQRXRefBuf[LEFT][2] += chassis_cmd.vx * 0.001f * 0.001f;  // cmd更新频率为1ms,单位为米
    LQRXRefBuf[RIGHT][2] += chassis_cmd.vx * 0.001f * 0.001f;  // cmd更新频率为1ms,单位为米*/
    LQRXRefBuf[LEFT][2] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[LEFT];
    LQRXRefBuf[RIGHT][2] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS - pos_x_offset[RIGHT];
    // 期望线速度
/*    LQRXRefBuf[LEFT][3]  = vx_change_limit((chassis_cmd.vx/1000), LQRXObsBuf[LEFT][3]) +yaw_pid->Output;   // 米每秒
    LQRXRefBuf[RIGHT][3] = vx_change_limit((chassis_cmd.vx/1000), LQRXObsBuf[RIGHT][3]) -yaw_pid->Output;  // 米每秒*/
    LQRXRefBuf[LEFT][3]  = (chassis_cmd.vx/1000) +yaw_pid->Output;   // 米每秒
    LQRXRefBuf[RIGHT][3] = (chassis_cmd.vx/1000) -yaw_pid->Output;  // 米每秒

    //更新航向角期望
    yaw_target += chassis_cmd.vw * 0.001f; // cmd更新频率为1ms,单位为度
}

void chassis_task_init(void)
{
    chassis_sub_init();
    chassis_motor_init();
#ifdef SLIP_DETECTION
    chassis_kf_init();
#endif /* SLIP_DETECTION */
}

/**
 * @brief 底盘控制任务,在RTOS中应该设定为200hz运行
 */
void chassis_control_task(void)
{
    /* 更新该线程所有的订阅者 */
    chassis_sub_pull();

    switch (chassis_cmd.ctrl_mode)
    {
        case CHASSIS_RELAX:
            motor_relax();
            break;
        case CHASSIS_INIT:
            motor_enable();
            leg_init_get_zero();
            break;
        case CHASSIS_RECOVERY:
            motor_enable();
            /* 保持腿长，便于倒地自起 */
            leg[LEFT]->length_ref = /*leg[LEFT]->PendulumLength*/0.1;
            leg[RIGHT]->length_ref = /*leg[RIGHT]->PendulumLength*/0.1;
            /* 倒地自起后重置yaw期望和x位移的offset */
            yaw_target = ins.yaw_total_angle;
            pos_x_offset[LEFT] = lk_motor[LEFT]->measure.total_angle * WHEEL_RADIUS;
            pos_x_offset[RIGHT] = lk_motor[RIGHT]->measure.total_angle * WHEEL_RADIUS;
            //TODO: 处于该模式下，应该屏蔽遥控器等控制
            break;
        case CHASSIS_FOLLOW_GIMBAL:
            // motor_enable();
            break;
        case CHASSIS_SPIN:

            break;
        case CHASSIS_OPEN_LOOP:
            motor_enable();
            /* 更改腿长 */
            leg[LEFT]->length_ref = LEN_LOW;
            leg[RIGHT]->length_ref = LEN_LOW;
            /* 切换对应的 K 矩阵 */
            MatLQRNegK.pData = (float*)LQR_k[0];
            break;
        case CHASSIS_STAND_MID:
            motor_enable();
            leg[LEFT]->length_ref = LEN_MID;
            leg[RIGHT]->length_ref = LEN_MID;
            MatLQRNegK.pData = (float*)LQR_k[1];
            break;
        case CHASSIS_STAND_HIG:
            //TODO: 添加对应的遥控切换键位
            motor_enable();
            leg[LEFT]->length_ref = LEN_HIG;
            leg[RIGHT]->length_ref = LEN_HIG;
            MatLQRNegK.pData = (float*)LQR_k[2];
            break;
        case CHASSIS_JUMP:
            motor_enable();
            jumping_control();
            break;
        case CHASSIS_STOP:
            ht_motor_disable_all();//此时ht不会再响应指令
            motor_relax();//relax内部只是给电机发送的命令为零，电机仍具有响应命令的能力
            break;
        case CHASSIS_FLY:
            break;
        case CHASSIS_AUTO:
            break;
        default:
            motor_relax();
            break;
    }

#ifdef SLIP_DETECTION
    chassis_kf_update();
#endif /*SLIP_DETECTION*/
    leg_calc(); // 保证稳定的运算频率，不受模式影响
    chassis_fdb_data.lk_l = lk_motor[LEFT]->measure;
    chassis_fdb_data.lk_r = lk_motor[RIGHT]->measure;
    /* 更新发布该线程的msg */
    chassis_pub_push();
}

/* --------------------------------- 电机控制相关 --------------------------------- */
static void leg_init_get_zero()
{
    // 首先各个电机给定一个适当的力矩，并持续，确保撞到限位
    chassis_fdb_data.leg_state = LEG_BACK_STEP;
    osDelay(2000);
    // 撞到限位后，控制电机在此处设置零点
    for (uint8_t i = 0; i < 4; i++)
    {
        ht_motor[i]->set_mode(ht_motor[i], CMD_ZERO_POSITION);
    }
    // 电机零点设置完成，正常零点为减去各偏移量
    chassis_fdb_data.leg_state = LEG_BACK_IS_OK;
    // 该函数仅在每次重新上电执行
}
static void motor_enable()
{
    ht_motor_enable_all();  // 海泰所有电机进入 motor 模式
    for (uint8_t i = 0; i < 4; i++)
    {
        ht_motor_set_type(ht_motor[i], MOTOR_ENALBED);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        lk_motor_enable(lk_motor[i]);
    }
}

static void motor_relax()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        ht_motor_set_type(ht_motor[i], MOTOR_STOP);
    }
    for (uint8_t i = 0; i < 2; i++)
    {
        lk_motor_relax(lk_motor[i]);
    }
}

static float control_dt[4];
static float control_start[4];

/* 1 号电机 */
static ht_motor_para_t ht_control_1(ht_motor_measure_t measure)
{
    control_dt[0] = dwt_get_time_us() - control_start[0];
    control_start[0] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if((chassis_cmd.ctrl_mode == CHASSIS_INIT) || (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY))
    {
        send_t = HT_INIT_OUT;
    }
    else
    {
        send_t = vmc_out_l[FRONT] + jump_out_l[FRONT];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -HT_OUTPUT_LIMIT, HT_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 2 号电机 */
static ht_motor_para_t ht_control_2(ht_motor_measure_t measure)
{
    control_dt[1] = dwt_get_time_us() - control_start[1];
    control_start[1] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if((chassis_cmd.ctrl_mode == CHASSIS_INIT) || (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY))
    {
        send_t = -HT_INIT_OUT;
    }
    else
    {
        send_t = -vmc_out_r[FRONT] + jump_out_r[FRONT];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -HT_OUTPUT_LIMIT, HT_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 3 号电机 */
static ht_motor_para_t ht_control_3(ht_motor_measure_t measure)
{
    control_dt[2] = dwt_get_time_us() - control_start[2];
    control_start[2] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if((chassis_cmd.ctrl_mode == CHASSIS_INIT) || (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY))
    {
        send_t = HT_INIT_OUT;
    }
    else
    {
        send_t = -vmc_out_r[BACK] + jump_out_r[BACK];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -HT_OUTPUT_LIMIT, HT_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 4 号电机 */
static ht_motor_para_t ht_control_4(ht_motor_measure_t measure)
{
    control_dt[3] = dwt_get_time_us() - control_start[3];
    control_start[3] = dwt_get_time_us();
    static ht_motor_para_t set;
    float send_t;

    // 每次上电归中电机给定一个适当的力矩，并持续，确保撞到限位
    if((chassis_cmd.ctrl_mode == CHASSIS_INIT) || (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY))
    {
        send_t = -HT_INIT_OUT;
    }
    else
    {
        send_t = vmc_out_l[BACK] + jump_out_l[BACK];
#ifdef CLOSE_ALL_MOTOR
        send_t = 0;
#endif
    }

    LIMIT_MIN_MAX(send_t, -HT_OUTPUT_LIMIT, HT_OUTPUT_LIMIT);
    {
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
        set.t = send_t; // 正负没问题
    }
    return set;
}
/* 底盘每个电机对应的控制函数 */
static void *ht_control[4] =
        {
                ht_control_1,
                ht_control_2,
                ht_control_3,
                ht_control_4,
        };

static void ht_motor_init()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        motor_config_t ht_motor_config = {
                .motor_type = HT04,
                .can_id = 1,
                .tx_id = i+1,
                .rx_id = 0x10, // TODO
        };
        ht_motor[i] = ht_motor_register(&ht_motor_config, ht_control[i]);
    }
/*    motor_config_t ht_motor_config = {
            .motor_type = HT04,
            .can_name = CAN_CHASSIS,
            .tx_id = 1,
            .rx_id = 0x10, // TODO
    };
    ht_motor[0] = ht_motor_register(&ht_motor_config, ht_control[0]);
    ht_motor_config.tx_id = 2;
    ht_motor_config.can_name = CAN_GIMBAL,
    ht_motor[1] = ht_motor_register(&ht_motor_config, ht_control[1]);
    ht_motor_config.tx_id = 3;
    ht_motor_config.can_name = CAN_GIMBAL,
    ht_motor[2] = ht_motor_register(&ht_motor_config, ht_control[2]);
    ht_motor_config.tx_id = 4;
    ht_motor_config.can_name = CAN_CHASSIS,
    ht_motor[3] = ht_motor_register(&ht_motor_config, ht_control[3]);*/
}

static int16_t lk_control_l(lk_motor_measure_t measure){
    static int16_t set;

    LIMIT_MIN_MAX(LQROutBuf[LEFT][0], -LK_OUTPUT_LIMIT, LK_OUTPUT_LIMIT);
#ifndef CLOSE_ALL_MOTOR
    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        set = 0;
    }
    else
    {
        set = (int16_t)(LQROutBuf[LEFT][0] * LK_TOR_TO_CUR);
    }
#endif
    return set;
}

static int16_t lk_control_r(lk_motor_measure_t measure){
    static int16_t set;

    LIMIT_MIN_MAX(LQROutBuf[RIGHT][0], -LK_OUTPUT_LIMIT, LK_OUTPUT_LIMIT);
#ifndef CLOSE_ALL_MOTOR
    if(chassis_cmd.ctrl_mode == CHASSIS_INIT)
    {
        set = 0;
    }
    else
    {
        set = (int16_t)(LQROutBuf[RIGHT][0] * LK_TOR_TO_CUR);
    }
#endif
    return set;
}

static void *lk_control[2] =
        {
                lk_control_l,
                lk_control_r,
        };

static void lk_motor_init()
{
    motor_config_t motor_config = {
            .motor_type = MF9025,
            .can_id = 1,
            .tx_id = 0x141,
            .rx_id = 0x141,
    };
    lk_motor[RIGHT] = lk_motor_register(&motor_config, lk_control[RIGHT]);

    motor_config_t motor_config_2 = {
            .motor_type = MF9025,
            .can_id = 1,
            .tx_id = 0x142,
            .rx_id = 0x142,
    };
    lk_motor[LEFT] = lk_motor_register(&motor_config_2, lk_control[LEFT]);
}

/**
 * @brief 底盘初始化（注册底盘电机及其控制器初始化等）
 */
static int chassis_motor_init(void)
{
    ht_motor_init();

    leg_config_t leg_config =
            {
                    /*单位m*/
                    0.15f,  // l4=l1
                    0.250f, // l3=l2
                    0.11f   //电机间距
                    /* TODO: 改为宏定义 */
            };
    leg[LEFT] = leg_register(&leg_config);
    leg[RIGHT] = leg_register(&leg_config);

    //TODO
    lk_motor_init();

    pid_config_t length_pid_config = INIT_PID_CONFIG(500, 0, 200, 0, 500,
                                                     (PID_Integral_Limit | PID_DerivativeFilter));
    length_pid[LEFT] = pid_register(&length_pid_config);
    length_pid[RIGHT] = pid_register(&length_pid_config);

    // TODO: 两腿协调 PD 控制，航向角控制，横滚角补偿控制
    /* 两腿协调 PD 控制 */
//    pid_config_t theta_pid_config = INIT_PID_CONFIG(15, 0, 0.2, 0, 2, PID_Integral_Limit);//加云台前
    pid_config_t theta_pid_config = INIT_PID_CONFIG(15, 0, 0.2, 0, 5, PID_Integral_Limit);//加云台后，输出不够会劈叉
    theta_pid = pid_register(&theta_pid_config);
    /* 航向角 PD 控制 */
    pid_config_t yaw_pid_config = INIT_PID_CONFIG(0.25, 0, 0.01, 0, 2, PID_Integral_Limit);
    yaw_pid = pid_register(&yaw_pid_config);
    /* 横滚角 PD 控制 */
    pid_config_t roll_pid_config = INIT_PID_CONFIG(15, 0, 0.5, 0, 100, PID_Integral_Limit);
    roll_pid = pid_register(&roll_pid_config);

    return 0;
}

/* --------------------------------- 底盘解算控制 --------------------------------- */
#ifdef BSP_CHASSIS_OMNI_MODE
/**
 * @brief 全向轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
static void omni_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    int16_t wheel_rpm[4];
    float wheel_rpm_ratio;

    wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * 3.14159f) * CHASSIS_DECELE_RATIO * 1000;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//left//x，y方向速度,w底盘转动速度
    wheel_rpm[1] = ( cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//forward
    wheel_rpm[2] = (-cmd->vx - cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//right
    wheel_rpm[3] = (-cmd->vx + cmd->vy + cmd->vw * (LENGTH_A + LENGTH_B)) * wheel_rpm_ratio;//back

    memcpy(out_speed, wheel_rpm, 4*sizeof(int16_t));//copy the rpm to out_speed
}
#endif /* BSP_CHASSIS_OMNI_MODE */

#ifdef BSP_CHASSIS_MECANUM_MODE

/**
 * @brief 麦克纳姆轮底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮速度
 */
void mecanum_calc(struct chassis_cmd_msg *cmd, int16_t* out_speed)
{
    static float rotate_ratio_f = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float rotate_ratio_b = ((WHEELBASE + WHEELTRACK) / 2.0f) / RADIAN_COEF;
    static float wheel_rpm_ratio = 60.0f / (WHEEL_PERIMETER * CHASSIS_DECELE_RATIO);

    int16_t wheel_rpm[4];
    float max = 0;

    //限制底盘各方向速度
    VAL_LIMIT(cmd->vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);  //mm/s
    VAL_LIMIT(cmd->vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);  //mm/s
    VAL_LIMIT(cmd->vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);  //deg/s

    wheel_rpm[0] = ( cmd->vx - cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[1] = ( cmd->vx + cmd->vy + cmd->vw * rotate_ratio_f) * wheel_rpm_ratio;
    wheel_rpm[2] = (-cmd->vx + cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;
    wheel_rpm[3] = (-cmd->vx - cmd->vy + cmd->vw * rotate_ratio_b) * wheel_rpm_ratio;

    memcpy(out_speed, wheel_rpm, 4 * sizeof(int16_t));
}
#endif /* BSP_CHASSIS_MECANUM_MODE */

#ifdef BSP_CHASSIS_LEG_MODE
static uint8_t isTouchingGround;
/**
 * @brief 轮腿底盘运动解算
 *
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮力矩
 */
static void leg_calc()
{
    float len_pid_out;
    // 左腿解算
    //更新phi1、phi4并通过判断phi1和phi4的大小判断腿的状态是否正常
    leg[LEFT]->input_leg_angle(leg[LEFT], /*-0.42*/(ht_motor[LEFT_BACK]->measure.total_angle - HT_OFFSET_LB), /*3.6*/PI + (ht_motor[LEFT_FRONT]->measure.total_angle - HT_OFFSET_LF));
    leg[LEFT]->resolve(leg[LEFT]);//求得摆角、摆长 [phi0; l0]，并更新phi2、phi3和倒立摆模型各点坐标
    //求得腿部运动速度 [dl0; dphi0; ddl0]
    leg[LEFT]->get_leg_spd(leg[LEFT], ht_motor[LEFT_FRONT]->measure.speed_rads, ht_motor[LEFT_BACK]->measure.speed_rads);

    // 右腿解算
    leg[RIGHT]->input_leg_angle(leg[RIGHT], -(ht_motor[RIGHT_BACK]->measure.total_angle - HT_OFFSET_RB),  PI - (ht_motor[RIGHT_FRONT]->measure.total_angle - HT_OFFSET_RF));
    leg[RIGHT]->resolve(leg[RIGHT]);
    leg[RIGHT]->get_leg_spd(leg[RIGHT], -ht_motor[RIGHT_FRONT]->measure.speed_rads, -ht_motor[RIGHT_BACK]->measure.speed_rads);

    //更新观测量和从cmd得到的目标期望，目前x的期望和观测的值始终一致，后续视情况可能需要更改
    update_LQR_obs();
    //通过观测值和期望值的差得到驱动轮电机的输出扭矩，对应lk，在LQROutBuf里更新
    LQR_cal();

    /* 双腿角度协调控制 */
    pid_calculate(theta_pid, leg[LEFT]->PendulumRadian - leg[RIGHT]->PendulumRadian, 0);
    /* 航向角控制 */
    pid_calculate(yaw_pid, ins.yaw_total_angle, yaw_target);
    /* 横滚角控制 */
    pid_calculate(roll_pid, ins.roll, chassis_cmd.vy + ROLL_OFFSET);

    len_pid_out = pid_calculate(length_pid[LEFT], leg[LEFT]->PendulumLength, leg[LEFT]->length_ref);
    LIMIT_MIN_MAX(len_pid_out, -300, 300);
    //根据离地状态计算左右腿推力，若离地则不考虑roll轴PID输出和前馈量
//    leftForce = legLengthPID.output + ((groundDetector.isTouchingGround && !groundDetector.isCuchioning) ? 6-rollPID.output : 0);
    ft_l[0] = len_pid_out + roll_pid->Output + LEG_LENGHT_F;
    ft_l[1] = LQROutBuf[LEFT][1] + theta_pid->Output;

    len_pid_out = pid_calculate(length_pid[RIGHT], leg[RIGHT]->PendulumLength, leg[RIGHT]->length_ref);
    LIMIT_MIN_MAX(len_pid_out, -300, 300);
    ft_r[0] = len_pid_out - roll_pid->Output + LEG_LENGHT_F;
    ft_r[1] = LQROutBuf[RIGHT][1] - theta_pid->Output;

    /* 离地检测，计算两腿地面支持力 */
//    TODO：目前离地检测还存在问题 ins.acc 存在问题，可能需要卡尔曼滤波
    leg[LEFT]->support_force = ft_l[0] + LEG_MASS * 9.8f + LEG_MASS * (ins.motion_accel_b[2] - leg[LEFT]->dd_l0);
    leg[RIGHT]->support_force = ft_r[0] + LEG_MASS * 9.8f + LEG_MASS * (ins.motion_accel_b[2] - leg[RIGHT]->dd_l0);
    isTouchingGround = (leg[LEFT]->support_force < 15) && (leg[RIGHT]->support_force < 15); //判断当前瞬间是否接地

    //解算得到对应腿前后两关节电机的输出力矩,对应HT电机，在vmc_out_l/r里更新
    leg[LEFT]->VMC_cal(leg[LEFT], ft_l, vmc_out_l);
    leg[RIGHT]->VMC_cal(leg[RIGHT], ft_r, vmc_out_r);
}

/**
 * @brief 底盘跳跃处理
 * @note 进入跳跃模式时记录当前时间，并将jump输出力矩加上跳跃下压力矩值，持续0.2s,开始跳跃0，15s后加上跳跃回缩力矩值，持续0.4s,落地后跳跃结束
 */
// TODO: 结合离地检测进行优化（进阶玩法：空中轮子充当动量轮调整姿态
// TODO: 跳跃力矩和持续时间还需继续优化
// TODO: 跳跃前需要增加机体pitch轴范围判断，或起跳前通过改写k矩阵，使得机体pitch轴更快收敛，充分准备起跳，减少空中发撒
// TODO：目前通过多状态量判断跳跃是否结束，以及切入跳跃模式仅跳跃一次，需要进一步优化，可以结合cmd线程
static void jumping_control(void)
{
    float current_time = dwt_get_time_ms();
    float dt;
    if(chassis_cmd.last_mode != CHASSIS_JUMP)
    {
        jump_flag = 1;
    }

    if(jump_flag)
    {

        if(!is_jumping)
        {
            is_jumping = 1;
            jump_start_time = current_time;
        }
        dt = current_time - jump_start_time;

        jump_out_l[FRONT] = -JUMP_TORQUE_PRESS * (dt<200) + JUMP_TORQUE_SHRINK * (dt>150 && dt<300);
        jump_out_l[BACK]  =  JUMP_TORQUE_PRESS * (dt<200) - JUMP_TORQUE_SHRINK * (dt>150 && dt<300);
        jump_out_r[FRONT] =  JUMP_TORQUE_PRESS * (dt<200) - JUMP_TORQUE_SHRINK * (dt>150 && dt<300);
        jump_out_r[BACK]  = -JUMP_TORQUE_PRESS * (dt<200) + JUMP_TORQUE_SHRINK * (dt>150 && dt<300);

        if (dt > 400)
        {
            is_jumping = 0;  // 跳跃结束
            jump_flag = 0;  // 跳跃结束
        }
    }
}
#endif /* BSP_CHASSIS_MECANUM_MODE */

/**
 * @brief  根据当前腿长限幅加速度,添加斜坡
 * @param  vx_cmd: 期望速度
 * @param  vx_now: 当前速度
 */
static float vx_change_limit(float vx_cmd, float vx_now)
{
    //根据当前腿长计算速度斜坡步长(腿越短越稳定，加减速斜率越大)
    float legLength = (leg[LEFT]->PendulumLength + leg[RIGHT]->PendulumLength) / 2;
    float speed_step = legLength * (-2.5) + 1;
    float vx_change = vx_cmd - vx_now;
    float vx_limited; // 限制后的速度,返回值

    //计算速度斜坡，斜坡值更新到target.speed
    if(fabsf(vx_change) < fabsf(speed_step))
    {
        vx_limited = vx_cmd;
    }
    else
    {
        if(vx_change > 0)
            vx_limited = vx_now + speed_step;
        else
            vx_limited = vx_now - speed_step;
    }

    return vx_limited;
}

/**
 * @brief chassis 线程中所有订阅者初始化
 */
static void chassis_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb_data);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_cmd);
    }
}
