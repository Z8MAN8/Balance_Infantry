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

BCPFrameTypeDef upper_rx_data;
BCPFrameTypeDef upper_tx_data;
BCPRpyTypeDef rpy_rx_data;
BCPImuTypeDef imu_tx_data;


void Transmission_Task(void const * argument){
    int8_t imu_tx_buffer[FRAME_IMU_LEN] = {0} ;
    int32_t imu_data = BMI088.accel[0] * 10000;
    uint32_t *chassis_i = (uint32_t *)&imu_data;

    uint32_t trans_wake_time = osKernelSysTick();
    while (1){
        Chassis_Send_supercap();
        Chassis_Send_shoot();

        /* USB发送imu帧 */
        imu_data = BMI088.accel[0] * 10000;
        imu_tx_buffer[0] = *chassis_i;
        imu_tx_buffer[1] = *chassis_i >> 8;
        imu_tx_buffer[2] = *chassis_i >> 16;
        imu_tx_buffer[3] = *chassis_i >> 24;
        imu_data = BMI088.accel[1] * 10000;
        imu_tx_buffer[4] = *chassis_i;
        imu_tx_buffer[5] = *chassis_i >> 8;
        imu_tx_buffer[6] = *chassis_i >> 16;
        imu_tx_buffer[7] = *chassis_i >> 24;
        imu_data = BMI088.accel[2] * 10000;
        imu_tx_buffer[8] = *chassis_i;
        imu_tx_buffer[9] = *chassis_i >> 8;
        imu_tx_buffer[10] = *chassis_i >> 16;
        imu_tx_buffer[11] = *chassis_i >> 24;
        imu_data = BMI088.gyro[0] * 10000;
        imu_tx_buffer[12] = *chassis_i;
        imu_tx_buffer[13] = *chassis_i >> 8;
        imu_tx_buffer[14] = *chassis_i >> 16;
        imu_tx_buffer[15] = *chassis_i >> 24;
        imu_data = BMI088.gyro[1] * 10000;
        imu_tx_buffer[16] = *chassis_i;
        imu_tx_buffer[17] = *chassis_i >> 8;
        imu_tx_buffer[18] = *chassis_i >> 16;
        imu_tx_buffer[19] = *chassis_i >> 24;
        imu_data = BMI088.gyro[2] * 10000;
        imu_tx_buffer[20] = *chassis_i;
        imu_tx_buffer[21] = *chassis_i >> 8;
        imu_tx_buffer[22] = *chassis_i >> 16;
        imu_tx_buffer[23] = *chassis_i >> 24;

        Add_Frame_To_Upper(CHASSIS_IMU, imu_tx_buffer);
        CDC_Transmit_FS((uint8_t*)&imu_tx_data, sizeof(imu_tx_data));

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
        default:break;
    }
}