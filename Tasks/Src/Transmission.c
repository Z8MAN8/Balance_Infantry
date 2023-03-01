//
// Created by 14685 on 2022/7/16.
//

#include <usbd_cdc_if.h>
#include <BMI088driver.h>
#include "Transmission.h"
#include "Chassis.h"
#include "bsp_can.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "Referee_system.h"
#include "Ins.h"

BCPFrameTypeDef upper_tx_all_data[FRAME_NUM];
BCPFrameTypeDef upper_rx_data;
BCPFrameTypeDef upper_tx_data;
BCPRpyTypeDef rpy_rx_data;
BCPImuTypeDef imu_tx_data;
BCPCtrlTypeDef ctrl_tx_data;
BCPCtrlTypeDef ctrl_rx_data;
BCPCtrlTypeDef odom_tx_data;

extern float chassis_vx;
extern float chassis_vy;
extern float chassis_vw;

extern uint8_t USB_SEND_OK;

void Transmission_Task(void const * argument){
    int8_t imu_tx_buffer[FRAME_IMU_LEN] = {0} ;
    int32_t imu_data = ins.q[0] * 10000;
    uint32_t *chassis_i = (uint32_t *)&imu_data;

    int8_t odom_tx_buffer[FRAME_ODOM_LEN] = {0} ;
    int32_t odom_data = chassis_vx * 10000;
    uint32_t *chassis_o = (uint32_t *)&odom_data;

    int8_t test_buffer[1] = {0} ;

    uint32_t trans_wake_time = osKernelSysTick();
    while (1){
        Chassis_Send_supercap();
        Chassis_Send_shoot();

        if(!USB_CONNECT_OK){
            CDC_Transmit_FS(test_buffer,sizeof(test_buffer));
        }  /*一直发送测试USB连接，防止不进发送回调*/

        /* USB发送imu帧 */
        imu_data = ins.q[0] * 10000;
        imu_tx_buffer[0] = *chassis_i;
        imu_tx_buffer[1] = *chassis_i >> 8;
        imu_tx_buffer[2] = *chassis_i >> 16;
        imu_tx_buffer[3] = *chassis_i >> 24;
        imu_data = ins.q[1] * 10000;
        imu_tx_buffer[4] = *chassis_i;
        imu_tx_buffer[5] = *chassis_i >> 8;
        imu_tx_buffer[6] = *chassis_i >> 16;
        imu_tx_buffer[7] = *chassis_i >> 24;
        imu_data = ins.q[2] * 10000;
        imu_tx_buffer[8] = *chassis_i;
        imu_tx_buffer[9] = *chassis_i >> 8;
        imu_tx_buffer[10] = *chassis_i >> 16;
        imu_tx_buffer[11] = *chassis_i >> 24;
        imu_data = ins.q[3] * 10000;
        imu_tx_buffer[12] = *chassis_i;
        imu_tx_buffer[13] = *chassis_i >> 8;
        imu_tx_buffer[14] = *chassis_i >> 16;
        imu_tx_buffer[15] = *chassis_i >> 24;
        imu_data = BMI088.gyro[0] * 10000;
        imu_tx_buffer[16] = *chassis_i;
        imu_tx_buffer[17] = *chassis_i >> 8;
        imu_tx_buffer[18] = *chassis_i >> 16;
        imu_tx_buffer[19] = *chassis_i >> 24;
        imu_data = BMI088.gyro[1] * 10000;
        imu_tx_buffer[20] = *chassis_i;
        imu_tx_buffer[21] = *chassis_i >> 8;
        imu_tx_buffer[22] = *chassis_i >> 16;
        imu_tx_buffer[23] = *chassis_i >> 24;
        imu_data = BMI088.gyro[2] * 10000;
        imu_tx_buffer[24] = *chassis_i;
        imu_tx_buffer[25] = *chassis_i >> 8;
        imu_tx_buffer[26] = *chassis_i >> 16;
        imu_tx_buffer[27] = *chassis_i >> 24;
        imu_data = BMI088.accel[0] * 10000;
        imu_tx_buffer[28] = *chassis_i;
        imu_tx_buffer[29] = *chassis_i >> 8;
        imu_tx_buffer[30] = *chassis_i >> 16;
        imu_tx_buffer[31] = *chassis_i >> 24;
        imu_data = BMI088.accel[1] * 10000;
        imu_tx_buffer[32] = *chassis_i;
        imu_tx_buffer[33] = *chassis_i >> 8;
        imu_tx_buffer[34] = *chassis_i >> 16;
        imu_tx_buffer[35] = *chassis_i >> 24;
        imu_data = BMI088.accel[2] * 10000;
        imu_tx_buffer[36] = *chassis_i;
        imu_tx_buffer[37] = *chassis_i >> 8;
        imu_tx_buffer[38] = *chassis_i >> 16;
        imu_tx_buffer[39] = *chassis_i >> 24;

        Add_Frame_To_Upper(CHASSIS_IMU, imu_tx_buffer);
        if((USB_SEND_OK % 2) == 1){
            CDC_Transmit_FS((uint8_t*)&imu_tx_data, sizeof(imu_tx_data));
        }
//        CDC_Transmit_FS((uint8_t*)&imu_tx_data, sizeof(imu_tx_data));

        /* USB发送角/线速度方式控制帧 */
        odom_data = chassis_vx * 10;
        odom_tx_buffer[0] = *chassis_o;
        odom_tx_buffer[1] = *chassis_o >> 8;
        odom_tx_buffer[2] = *chassis_o >> 16;
        odom_tx_buffer[3] = *chassis_o >> 24;
        odom_data = chassis_vy * 10;
        odom_tx_buffer[4] = *chassis_o;
        odom_tx_buffer[5] = *chassis_o >> 8;
        odom_tx_buffer[6] = *chassis_o >> 16;
        odom_tx_buffer[7] = *chassis_o >> 24;
        odom_data = chassis_vw * 10000;
        odom_tx_buffer[8] = *chassis_o;
        odom_tx_buffer[9] = *chassis_o >> 8;
        odom_tx_buffer[10] = *chassis_o >> 16;
        odom_tx_buffer[11] = *chassis_o >> 24;
        odom_data = chassis_total_x * 10;
        odom_tx_buffer[12] = *chassis_o;
        odom_tx_buffer[13] = *chassis_o >> 8;
        odom_tx_buffer[14] = *chassis_o >> 16;
        odom_tx_buffer[15] = *chassis_o >> 24;
        odom_data = chassis_total_y * 10;
        odom_tx_buffer[16] = *chassis_o;
        odom_tx_buffer[17] = *chassis_o >> 8;
        odom_tx_buffer[18] = *chassis_o >> 16;
        odom_tx_buffer[19] = *chassis_o >> 24;
        odom_data = chassis_total_w * 10000;
        odom_tx_buffer[20] = *chassis_o;
        odom_tx_buffer[21] = *chassis_o >> 8;
        odom_tx_buffer[22] = *chassis_o >> 16;
        odom_tx_buffer[23] = *chassis_o >> 24;

        Add_Frame_To_Upper(CHASSIS_ODOM, odom_tx_buffer);
        if((USB_SEND_OK % 2) == 0){
            CDC_Transmit_FS((uint8_t*)&odom_tx_data, sizeof(odom_tx_data));
        }

        //TODO:研究USB虚拟串口的工作原理
//        memcpy(&upper_tx_all_data[0], &imu_tx_data, sizeof(imu_tx_data));
//        memcpy(&upper_tx_all_data[1], &ctrl_tx_data, sizeof(ctrl_tx_data));
//        CDC_Transmit_FS((uint8_t*)upper_tx_all_data, sizeof(upper_tx_all_data));

        vTaskDelayUntil(&trans_wake_time,1);
    }
}

void Chassis_Send_supercap(void)
{
    uint16_t temPower =(robot_status.chassis_power_limit)*100;//功率设定步进0.01W，范围为3000-13000（30W-130W）
//    uint16_t temPower =5000;//功率设定步进0.01W，范围为3000-13000（30W-130W）
    uint8_t sendbuf[8];//发送的数据内容
    sendbuf[0]=temPower >> 8;
    sendbuf[1]=temPower;
    CAN_Send(hcan1, CAN_SUPER_CAP_ID, sendbuf);

}

void Chassis_Send_shoot(void) //发送裁判系统对SHOOT的供电情况
{
    uint8_t sendbuf[8];//发送的数据内容
    sendbuf[0]=robot_status.mains_power_shooter_output;
    CAN_Send(hcan2,0x134, sendbuf);

}

uint16_t Sumcheck_Cal(BCPFrameTypeDef frame){
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint16_t allcheck = 0;

    sumcheck += frame.HEAD;
    addcheck += sumcheck;
    sumcheck += frame.D_ADDR;
    addcheck += sumcheck;
    sumcheck += frame.ID;
    addcheck += sumcheck;
    sumcheck += frame.LEN;
    addcheck += sumcheck;

    for(int i = 0; i<frame.LEN; i++){
        sumcheck += frame.DATA[i];
        addcheck += sumcheck;
    }
    allcheck = (uint16_t)(sumcheck << 8 | addcheck);
    return allcheck;
}

void Add_Frame_To_Upper(uint16_t send_mode, int8_t* data_buf){
    switch (send_mode) {
        case CHASSIS_IMU:{
            imu_tx_data.HEAD = 0XFF;
            imu_tx_data.D_ADDR = MAINFLOD;
            imu_tx_data.ID = CHASSIS_IMU;
            imu_tx_data.LEN = FRAME_IMU_LEN;
            memcpy(&imu_tx_data.DATA, data_buf, sizeof(imu_tx_data.DATA));

            /* 将 odom 帧先转存到中转帧中做数据校验计算 */
            memcpy(&upper_tx_data, &imu_tx_data, sizeof(imu_tx_data));
            imu_tx_data.SC = (uint8_t)Sumcheck_Cal(upper_tx_data) >> 8;
            imu_tx_data.AC = (uint8_t)Sumcheck_Cal(upper_tx_data);
            memset(&upper_tx_data, 0, sizeof(upper_tx_data));
        }break;
        case CHASSIS_CTRL:{
            ctrl_tx_data.HEAD = 0XFF;
            ctrl_tx_data.D_ADDR = MAINFLOD;
            ctrl_tx_data.ID = CHASSIS_CTRL;
            ctrl_tx_data.LEN = FRAME_CTRL_LEN;
            memcpy(&ctrl_tx_data.DATA, data_buf, sizeof(ctrl_tx_data.DATA));

            /* 将 odom 帧先转存到中转帧中做数据校验计算 */
            memcpy(&upper_tx_data, &ctrl_tx_data, sizeof(ctrl_tx_data));
            ctrl_tx_data.SC = (uint8_t)Sumcheck_Cal(upper_tx_data) >> 8;
            ctrl_tx_data.AC = (uint8_t)Sumcheck_Cal(upper_tx_data);
            memset(&upper_tx_data, 0, sizeof(upper_tx_data));
        }break;
        case CHASSIS_ODOM:{
            ctrl_tx_data.HEAD = 0XFF;
            ctrl_tx_data.D_ADDR = MAINFLOD;
            ctrl_tx_data.ID = CHASSIS_ODOM;
            ctrl_tx_data.LEN = FRAME_ODOM_LEN;
            memcpy(&odom_tx_data.DATA, data_buf, sizeof(odom_tx_data.DATA));

            /* 将 odom 帧先转存到中转帧中做数据校验计算 */
            memcpy(&upper_tx_data, &odom_tx_data, sizeof(odom_tx_data));
            odom_tx_data.SC = (uint8_t)Sumcheck_Cal(upper_tx_data) >> 8;
            odom_tx_data.AC = (uint8_t)Sumcheck_Cal(upper_tx_data);
            memset(&upper_tx_data, 0, sizeof(upper_tx_data));
        }break;
        default:break;
    }
}