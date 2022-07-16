//
// Created by 14685 on 2022/7/15.
//

#include "bsp_uart.h"
#include <string.h>
#include "usart.h"
#include "controller.h"

uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
RCTypeDef rc;

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver request
    //使能DMA串口接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

void Remote_Control_init(void)
{
    RC_Init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

void Remote_Data_handle(RCTypeDef *rc, uint8_t *buff)
{
    /* 下面是正常遥控器数据的处理 */
    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

    /* 防止遥控器零点有偏差 */
    if(rc->ch1 <= 5 && rc->ch1 >= -5)
        rc->ch1 = 0;
    if(rc->ch2 <= 5 && rc->ch2 >= -5)
        rc->ch2 = 0;
    if(rc->ch3 <= 5 && rc->ch3 >= -5)
        rc->ch3 = 0;
    if(rc->ch4 <= 5 && rc->ch4 >= -5)
        rc->ch4 = 0;

    /* 拨杆值获取 */
    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    /* 遥控器异常值处理，函数直接返回 */
    if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
    {
        memset(rc, 0, sizeof(RCTypeDef));
        return ;
    }

    /* 鼠标移动速度获取 */
    rc->mouse.x = buff[6] | (buff[7] << 8);
    rc->mouse.y = buff[8] | (buff[9] << 8);

    /* 鼠标左右按键键值获取 */
    rc->mouse.l = buff[12];
    rc->mouse.r = buff[13];

    /* 键盘按键键值获取 */
    rc->kb.key_code = buff[14] | buff[15] << 8;

    /* 遥控器左侧上方拨轮数据获取，和遥控器版本有关，有的无法回传此项数据 */
    /* 并未传到下板 */
//    rc->wheel = buff[16] | buff[17] << 8;
//    rc->wheel -= 1024;
}

void Uart_Send(uint8_t uart_id, uint8_t *send_data, uint16_t size){
    switch(uart_id){
        case 1:
        {
            HAL_UART_Transmit(&huart1,send_data,size,HAL_MAX_DELAY);
            break;
        }
        default:
            break;
    }
}