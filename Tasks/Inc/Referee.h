//
// Created by MEI on 2022/4/8.
//

#ifndef HNU_INFANTRY_DOWN_NEW_REFEREE_H
#define HNU_INFANTRY_DOWN_NEW_REFEREE_H
#include "main.h"
#include "cmsis_os.h"
#include "Referee.h"
#include "Referee_system.h"
#include "BSP_CRC.h"
#include "string.h"
#include "fifo.h"
#include "stdint.h"
#include "UI_print.h"
#include "keyboard.h"
extern uint8_t RX_AgreementData_Buffer0[Agreement_RX_BUF_NUM];   //接收裁判系统返回数据的接收缓冲区0,该缓冲区设置的相当富�??
extern uint8_t RX_AgreementData_Buffer1[Agreement_RX_BUF_NUM];   //接收裁判系统返回数据的接收缓冲区1，该缓冲区设置的相当富裕
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart6;
extern Frame_header_Typedef Referee_Data_header;         //实例化一个帧头结构体
extern RX_AgreementData     Referee_Data;                //实例化一个数据帧结构�??
extern fifo_s_t RX_AgreementData_FIFO;
extern int UI_TCBNum;
extern UI_TCB* UI_SendTCBSequence[30];
extern float PowerData[4];
extern char spin_flag;
extern uint8_t fric_flag;
extern _Bool fric_wheel_run;
extern _Bool cap_open_flag;
extern ext_game_robot_status_t ext_game_robot_status;
#endif //HNU_INFANTRY_DOWN_NEW_REFEREE_H
