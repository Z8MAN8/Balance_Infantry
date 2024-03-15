#include "cmd_task.h"
#include "rm_module.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "robot.h"

static rc_obj_t *rc_now, *rc_last;

/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
// 订阅
MCN_DECLARE(chassis_fdb);
static McnNode_t chassis_fdb_node;
static struct chassis_fdb_msg chassis_fdb;
MCN_DECLARE(gimbal_fdb);
static McnNode_t gimbal_fdb_node;
static struct gimbal_fdb_msg gimbal_fdb;
MCN_DECLARE(shoot_fdb);
static McnNode_t shoot_fdb_node;
static struct shoot_fdb_msg shoot_fdb;
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(trans_fdb);
static McnNode_t trans_fdb_node;
static struct trans_fdb_msg trans_fdb;
// 发布
MCN_DECLARE(chassis_cmd);
static struct chassis_cmd_msg chassis_cmd_data;
MCN_DECLARE(shoot_cmd);
static struct shoot_cmd_msg shoot_cmd_data;
MCN_DECLARE(gimbal_cmd);
static struct gimbal_cmd_msg gimbal_cmd_data;

static void cmd_pub_push(void);
static void cmd_sub_init(void);
static void cmd_sub_pull(void);

/*发射停止标志位*/
static int trigger_flag=0;
/*堵转电流反转记次*/
static int reverse_cnt;
static float gyro_yaw_inherit;
static float gyro_pitch_inherit;

static float cmd_flag=0;//发射保护,防止刚开始发射机构就启动

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
    /* 保存上一次数据 */
    // gim_cmd.last_mode = gim_cmd.ctrl_mode;
    chassis_cmd_data.last_mode = chassis_cmd_data.ctrl_mode;
    *rc_last = *rc_now;
    // TODO: 目前状态机转换较为简单，有很多优化和改进空间
    //遥控器的控制信息转化为标准单位，平移为(mm/s)旋转为(degree/s)
    chassis_cmd_data.vx = rc_now->ch2 * CHASSIS_RC_MOVE_RATIO_X / RC_MAX_VALUE * MAX_CHASSIS_VX_SPEED;
    chassis_cmd_data.vy = rc_now->ch1 * CHASSIS_RC_MOVE_RATIO_Y / RC_MAX_VALUE * MAX_CHASSIS_VY_SPEED;
    chassis_cmd_data.vw = rc_now->ch4 * CHASSIS_RC_MOVE_RATIO_R / RC_MAX_VALUE * MAX_CHASSIS_VR_SPEED;

    if (gimbal_cmd_data.ctrl_mode==GIMBAL_GYRO)
    {
        gimbal_cmd_data.yaw += rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW /*-fx * KB_RATIO * GIMBAL_PC_MOVE_RATIO_YAW*/;
        gimbal_cmd_data.pitch += rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT /*- fy * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT*/;
        gyro_yaw_inherit =gimbal_cmd_data.yaw;
        gyro_pitch_inherit =-(ins.pitch);


    }
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_AUTO) {

        gimbal_cmd_data.yaw = trans_fdb.yaw + gyro_yaw_inherit + 150 * rc_now->ch3 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_YAW;//上位机自瞄
        gimbal_cmd_data.pitch = trans_fdb.pitch + 100* rc_now->ch4 * RC_RATIO * GIMBAL_RC_MOVE_RATIO_PIT /*- Ballistic * KB_RATIO * GIMBAL_PC_MOVE_RATIO_PIT*/;//上位机自瞄

    }
    /* 限制云台角度 */

    VAL_LIMIT(gimbal_cmd_data.pitch, PIT_ANGLE_MIN, PIT_ANGLE_MAX);
    VAL_LIMIT(gimbal_cmd_data.yaw, -30,30);
    /*开环状态和遥控器归中*/
    if (gimbal_cmd_data.ctrl_mode==GIMBAL_INIT||gimbal_cmd_data.ctrl_mode==GIMBAL_RELAX)
    {
        gimbal_cmd_data.pitch=0;
        gimbal_cmd_data.yaw=0;

    }
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
   switch (rc_now->sw2) {
       case RC_UP:
           chassis_cmd_data.ctrl_mode = CHASSIS_RELAX;
           gimbal_cmd_data.ctrl_mode = GIMBAL_RELAX;
           shoot_cmd_data.ctrl_mode = SHOOT_STOP;
           /*放开状态下，gim不接收值*/
           gimbal_cmd_data.pitch = 0;
           gimbal_cmd_data.yaw = 0;

           break;

       case RC_DN:

//           if (gimbal_cmd_data.last_mode == GIMBAL_RELAX || gimbal_fdb.back_mode==BACK_STEP)
//           {/* 判断上次状态是否为RELAX，是则先归中 */
//               gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
//           }
//           else
//           {
//               if (gimbal_fdb.back_mode == BACK_IS_OK) {/* 判断归中是否完成 */
//                   gimbal_cmd_data.ctrl_mode = GIMBAL_GYRO;
//               }
//           }

//           if (gimbal_fdb.back_mode == BACK_IS_OK){
           if (1){
               if (chassis_fdb.leg_state == LEG_BACK_IS_OK)
               {
                   if (chassis_cmd_data.last_mode == CHASSIS_INIT || chassis_cmd_data.last_mode == CHASSIS_RELAX) {
                       chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
                   } else if (chassis_cmd_data.last_mode == CHASSIS_RECOVERY) {
                       /*if(usr_abs(obs_data.phi) <= 0.05)
                       {*/
                       chassis_cmd_data.ctrl_mode = CHASSIS_OPEN_LOOP;
                       if (rc_now->sw4 == RC_DN) {
                           chassis_cmd_data.ctrl_mode = CHASSIS_STAND_MID;
                       }
                       /*}
                       else
                       {
                           chassis_cmd_data.ctrl_mode = CHASSIS_RECOVERY;
                       }*/
                   } else {
                       chassis_cmd_data.ctrl_mode = CHASSIS_OPEN_LOOP;
                       if (rc_now->sw4 == RC_DN) {
                           chassis_cmd_data.ctrl_mode = CHASSIS_STAND_MID;
                       }

                       if (rc_now->sw1 == RC_DN) {   //TODO：增加按钮是否切换判断，切换才触发一次，维持不触发（参考老代码 keyboard 驱动）
//                           chassis_cmd_data.ctrl_mode = CHASSIS_JUMP;
//                           chassis_cmd_data.ctrl_mode = CHASSIS_SPIN;
                       }
                   }

//                   if (gimbal_cmd_data.last_mode == GIMBAL_RELAX || gimbal_fdb.back_mode==BACK_STEP)
//                   {/* 判断上次状态是否为RELAX，是则先归中 */
//                       gimbal_cmd_data.ctrl_mode = GIMBAL_INIT;
//                   }
//                   else
//                   {
//                       if (gimbal_fdb.back_mode == BACK_IS_OK) {/* 判断归中是否完成 */
//                           gimbal_cmd_data.ctrl_mode = GIMBAL_GYRO;
//                       }
//                   }

               } else {
                   chassis_cmd_data.ctrl_mode = CHASSIS_INIT;
               }

           }
           else{
               gimbal_cmd_data.ctrl_mode=GIMBAL_INIT;
           }

       break;
   }

   if(rc_now->sw3 == RC_DN)
   {
       chassis_cmd_data.ctrl_mode = CHASSIS_STOP;
       gimbal_cmd_data.ctrl_mode=GIMBAL_RELAX;
       shoot_cmd_data.trigger_status = TRIGGER_OFF;
       shoot_cmd_data.ctrl_mode = SHOOT_STOP;
   }
   else if(rc_now->sw3 == RC_MI){
       if (gimbal_fdb.back_mode == BACK_IS_OK&&chassis_fdb.leg_state == LEG_BACK_IS_OK) {
           cmd_flag = 1;
           shoot_cmd_data.trigger_status = TRIGGER_OFF;
           shoot_cmd_data.ctrl_mode = SHOOT_STOP;
       }
   }
   else if(rc_now->sw3 == RC_UP){
       if(gimbal_fdb.back_mode == BACK_IS_OK && chassis_fdb.leg_state == LEG_BACK_IS_OK){
           if((cmd_flag == 1)&& !(gimbal_cmd_data.last_mode==GIMBAL_RELAX)){
               shoot_cmd_data.trigger_status = TRIGGER_ON;
               shoot_cmd_data.ctrl_mode=SHOOT_COUNTINUE;
               cmd_flag=0;
           }
           else{
               shoot_cmd_data.trigger_status = TRIGGER_OFF;
               shoot_cmd_data.ctrl_mode = SHOOT_STOP;
               cmd_flag=0;
           }
       }
   }

    /*堵弹反转检测*/
    if (shoot_fdb.trigger_motor_current>=9500||reverse_cnt!=0)
    {
        shoot_cmd_data.ctrl_mode=SHOOT_REVERSE;
        if (reverse_cnt<120)
            reverse_cnt++;
        else
            reverse_cnt=0;
    }
    if (chassis_cmd_data.ctrl_mode==CHASSIS_SPIN)
    {
        chassis_cmd_data.vw=3;
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

}

/* --------------------------------- 线程间通讯相关 -------------------------------- */
/**
 * @brief cmd 线程中所有发布者推送更新话题
 */
static void cmd_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(chassis_cmd), &chassis_cmd_data);
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_cmd_data);
    mcn_publish(MCN_HUB(shoot_cmd), &shoot_cmd_data);
}

/**
 * @brief cmd 线程中所有订阅者初始化
 */
static void cmd_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_fdb_node = mcn_subscribe(MCN_HUB(chassis_fdb), NULL, NULL);
    gimbal_fdb_node = mcn_subscribe(MCN_HUB(gimbal_fdb), NULL, NULL);
    trans_fdb_node = mcn_subscribe(MCN_HUB(trans_fdb), NULL, NULL);
    shoot_fdb_node = mcn_subscribe(MCN_HUB(shoot_fdb), NULL, NULL);
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
    if (mcn_poll(gimbal_fdb_node))
    {
        mcn_copy(MCN_HUB(gimbal_fdb), gimbal_fdb_node, &gimbal_fdb);
    }
    if (mcn_poll(shoot_fdb_node))
    {
        mcn_copy(MCN_HUB(shoot_fdb), shoot_fdb_node, &shoot_fdb);
    }
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }
    if (mcn_poll(trans_fdb_node))
    {
        mcn_copy(MCN_HUB(trans_fdb), trans_fdb_node, &trans_fdb);
    }
}
