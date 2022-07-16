//
// Created by 14685 on 2022/7/15.
//

#include "../../Inc/BSP/bsp_can.h"
#include "can.h"
#include "Chassis.h"

static CAN_TxHeaderTypeDef  tx_message;
float PowerData [4]__attribute__ ((section(".ccmram")));


void CAN_Init(){
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN_Send(CAN_HandleTypeDef can, uint32_t send_id, uint8_t send_data[]){
    uint32_t send_mail_box;
    tx_message.StdId = send_id;
    tx_message.IDE = CAN_ID_STD;
    tx_message.RTR = CAN_RTR_DATA;
    tx_message.DLC = 0x08;
    HAL_CAN_AddTxMessage(&can, &tx_message, send_data, &send_mail_box);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    if (hcan == &hcan1) {
        switch (rx_header.StdId) {
            case CAN_3508_M1_ID: {
                chassis_motor[0].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[0], rx_data) : \
      get_moto_info(&chassis_motor[0], rx_data);
            }
                break;
            case CAN_3508_M2_ID: {
                chassis_motor[1].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[1], rx_data) : \
      get_moto_info(&chassis_motor[1], rx_data);
            }
                break;

            case CAN_3508_M3_ID: {
                chassis_motor[2].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[2], rx_data) : \
      get_moto_info(&chassis_motor[2], rx_data);
            }
                break;
            case CAN_3508_M4_ID: {
                chassis_motor[3].msg_cnt++ <= 50 ? get_motor_offset(&chassis_motor[3], rx_data) : \
      get_moto_info(&chassis_motor[3], rx_data);
            }
                break;
            case CAN_SUPERCAP_RECV: {
                PowerDataResolve(rx_data);
            }
                break;
            default: {
            }
                break;
        }
    }
    if (hcan == &hcan2) {
        switch (rx_header.StdId) {
            case CAN_UP_TX_INFO: {
                //gim.ctrl_mode= rx_data[0];
                yaw_relative_angle=*(float*)&rx_data[0];
//                yaw_angle_ref=*(float*)&rx_data[4];
                /*yaw_angle_ref=0;
                cap_open_flag=0x1&rx_data[6];
                fric_wheel_run=(0x2&rx_data[6])>>1;
                fric_flag=(0xC&rx_data[6])>>2;
                yaw_motor_speed=rx_data[4];
                yaw_motor_speed=yaw_motor_speed<<8;
                yaw_motor_speed=yaw_motor_speed|rx_data[5];*/

            }
            default: {
            }
                break;
        }
    }
}

void PowerDataResolve(uint8_t data[])
{
    uint16_t *pPowerData =(uint16_t *) data;
    PowerData[0]=(float)pPowerData[0]/100.f;//输入电压
    PowerData[1]=(float)pPowerData[1]/100.f;//电容电压
    PowerData[2]=(float)pPowerData[2]/100.f;//输入电流
    PowerData[3]=(float)pPowerData[3]/100.f;//设定功率
}
