#include "cmd_task.h"
#include "rm_module.h"
#include "robot.h"

static rc_obj_t *rc_now, *rc_last;

/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(chassis_fdb);
static McnNode_t chassis_fdb_node;
static struct chassis_fdb_msg chassis_fdb;
// 发布
MCN_DECLARE(chassis_cmd);
static struct chassis_cmd_msg chassis_cmd_data;

static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/* ------------------------------- 遥控数据转换为控制指令 ------------------------------ */
static void remote_to_cmd(void);
//TODO: 添加图传链路的自定义控制器控制方式和键鼠控制方式

/* -------------------------------- cmd 线程主体 -------------------------------- */
void cmd_task_init(void)
{
    cmd_sub_init();
    rc_now = sbus_rc_init();
    rc_last = (rc_now + 1);   // rc_obj[0]:当前数据NOW,[1]:上一次的数据LAST
}

void cmd_control_task(void)
{
    cmd_sub_pull();

    remote_to_cmd();

    cmd_pub_push();
}

/**
 * @brief 将遥控器数据转换为控制指令
 */
static void remote_to_cmd(void)
{
// TODO: 目前状态机转换较为简单，有很多优化和改进空间
   //遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
   chassis_cmd_data.vx = rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED;
   chassis_cmd_data.vy = rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED;
   chassis_cmd_data.vw = rc_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED;
   // TODO: 轮腿前期调试
/*    chassis_cmd_data.leg_length = rc_now->ch2 * ratio + base;          // 腿长
   chassis_cmd_data.leg_angle = rc_now->ch1 * ratio + base;           // 腿角度*/
   // chassis_cmd_data.offset_angle = gim_fdb.yaw_relative_angle;
   // gim_cmd.yaw += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;
   // gim_cmd.pitch += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT;
   // /* 限制云台角度 */
   // VAL_LIMIT(gim_cmd.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);

   // 左拨杆sw2为上时，底盘和云台均REALX；为中时，云台为GYRO；为下时，云台为AUTO。
   // 右拨杆sw1为上时，底盘为FOLLOW；为中时，底盘为OPEN；为下时，底盘为SPIN。
   switch (rc_now->sw2)
   {
   case RC_UP:
       chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
       break;

   case RC_DN:
       if(chassis_fdb.leg_state == LEG_BACK_IS_OK)
       {
           if(chassis_cmd_data.last_mode == CHASSIS_INIT || chassis_cmd_data.last_mode == CHASSIS_RELAX)
           {
               chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
           }
           else if(chassis_cmd_data.last_mode == CHASSIS_RECOVERY)
           {
               /*if(usr_abs(obs_data.phi) <= 0.05)
               {*/
                   chassis_cmd_data.ctrl_mode = CHASSIS_OPEN_LOOP;
                   if (rc_now->sw4 == RC_DN)
                   {
                       chassis_cmd_data.ctrl_mode = CHASSIS_STAND_MID;
                   }
               /*}
               else
               {
                   chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
               }*/
           }
           else
           {
               chassis_cmd_data.ctrl_mode = CHASSIS_OPEN_LOOP;
               if (rc_now->sw4 == RC_DN)
               {
                   chassis_cmd_data.ctrl_mode = CHASSIS_STAND_MID;
               }
           }

       }
       else
       {
           chassis_cmd_data.ctrl_mode = CHASSIS_INIT;
       }
       break;
   }

   if(rc_now->sw3 == RC_DN)
   {
       chassis_cmd_data.ctrl_mode = CHASSIS_STOP;
   }
   /* 因为左拨杆值会影响到底盘RELAX状态，所以后判断 */
   /*switch(rc_now->sw3)
   {
   case RC_UP:
       gim_cmd.ctrl_mode = GIMBAL_RELAX;
       chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
       break;
   case RC_MI:
       if(gim_cmd.last_mode == GIMBAL_RELAX)
       {*//* 判断上次状态是否为RELAX，是则先归中 *//*
           gim_cmd.ctrl_mode = GIMBAL_INIT;
       }
       else
       {
           if(gim_fdb.back_mode == BACK_IS_OK)
           {
               gim_cmd.ctrl_mode = GIMBAL_GYRO;
           }
       }
       break;
   case RC_DN:
       if(gim_cmd.last_mode == GIMBAL_RELAX)
       {*//* 判断上次状态是否为RELAX，是则先归中 *//*
           gim_cmd.ctrl_mode = GIMBAL_INIT;
       }
       else
       {
           if(gim_fdb.back_mode == BACK_IS_OK)
           {*//* 判断归中是否完成 *//*
               gim_cmd.ctrl_mode = GIMBAL_AUTO;
           }
       }
       break;
   }*/

   /* 保存上一次数据 */
   // gim_cmd.last_mode = gim_cmd.ctrl_mode;
   chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
   *rc_last = *rc_now;
}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);
}


/**
 * @brief cmd 线程中所有订阅者获取更新话题
 */
static void cmd_sub_pull(void)
{
    if (mcn_poll(chassis_fdb_node))
    {
        mcn_copy(MCN_HUB(chassis_fdb), chassis_fdb_node, &chassis_fdb);
    }
}
