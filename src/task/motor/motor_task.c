#include "motor_task.h"
#include "rm_module.h"
#include "robot.h"
#include "shoot_task.h"
static osMutexId semMotorHandle; // 触发CAN消息发送的信号量

void motor_task_init(void)
{
    osMutexDef(motor_Sem);
    semMotorHandle = osMutexCreate(osMutex(motor_Sem));  // 初始化信号量
}

//float lk_dt, lk_start;
extern CAN_HandleTypeDef hcan2;
extern  dji_motor_object_t *sht_motor[3];
void motor_control_task(void)
{
//    static osEvent evt;
    // static uint8_t cnt = 0; 设定不同电机的任务频率
    // if(cnt%5==0) //200hz
    // if(cnt%10==0) //100hz
//    dji_motor_control();
//    osSemaphoreWait(semMotorHandle, osWaitForever); // 等待信号量可用
/*    for (uint8_t i=0; i < 5; ++i)
    {
        evt = osSignalWait (0x01, osWaitForever);
        if (evt.status == osEventSignal)  {
            if(i==4){
                lk_motor_control();
            }
            else {
                ht_controller(i);
            }
        }*/

//前期测试
//    int16_t motor=5000;
//    uint8_t data[8]={0,};
//
//    data[0]=motor>>8;
//    data[1]=motor;
//    data[2]=motor>>8;
//    data[3]=motor;
//    data[4]=motor>>8;
//    data[5]=motor;
//    data[6]=motor>>8;
//    data[7]=motor;
//    data[0]=motor>>8;
//    data[1]=motor;
//    data[2]=motor>>8;
//    data[3]=motor;
//    CAN_send(&hcan2,0x2ff,data);


int16_t  send_shoot_right;
int16_t  send_shoot_left;
int16_t  send_shoot_trigger;

//调试用
//    send_shoot_right=sht_motor[0]->control(sht_motor[0]->measure);
//    send_shoot_left=sht_motor[1]->control(sht_motor[0]->measure);
//    send_shoot_trigger=sht_motor[2]->control(sht_motor[0]->measure);
//
//    uint8_t data_shoot[8];
//
//    data_shoot[0]=send_shoot_right>>8;
//    data_shoot[1]=send_shoot_right;
//    data_shoot[2]=send_shoot_left>>8;
//    data_shoot[3]=send_shoot_left;
//    data_shoot[4]=send_shoot_trigger>>8;
//    data_shoot[5]=send_shoot_trigger;
//    data_shoot[6]=0;
//    data_shoot[7]=0;
//    data_shoot[8]=0;
//
//    CAN_send(&hcan2, 0x200, data_shoot);


    dji_motor_control();
    lk_motor_control();
}

static float can_tim_dt, can_tim_start;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    static uint8_t i = 0;
    if (htim->Instance == htim3.Instance)
    {
        can_tim_dt = dwt_get_time_us() - can_tim_start;
        can_tim_start = dwt_get_time_us();
        ht_controll_all_poll();
    }
}
