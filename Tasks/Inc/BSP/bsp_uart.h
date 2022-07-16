//
// Created by 14685 on 2022/7/15.
//

#ifndef HNU_RM_DOWN_BSP_UART_H
#define HNU_RM_DOWN_BSP_UART_H
#include "stdint.h"
#include "usart.h"

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP                ((uint16_t)1)
#define RC_SW_MID               ((uint16_t)3)
#define RC_SW_DOWN              ((uint16_t)2)
#define switch_is_down(s)       (s == RC_SW_DOWN)
#define switch_is_mid(s)        (s == RC_SW_MID)
#define switch_is_up(s)         (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)


#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u
#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

/**
  * @brief     解析后的遥控器数据结构体
  */
typedef struct
{
    /* 遥控器的通道数据，数值范围：-660 ~ 660 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //右侧上下
    int16_t ch3;   //左侧左右
    int16_t ch4;   //左侧上下

    /* 遥控器的拨杆数据，上中下分别为：1、3、2 */
    uint8_t sw1;   //左侧拨杆
    uint8_t sw2;   //右侧拨杆

    /* PC 鼠标数据 */
    struct
    {
        /* 鼠标移动相关 */
        int16_t x;   //鼠标平移
        int16_t y;   //鼠标上下
        /* 鼠标按键相关，1为按下，0为松开 */
        uint8_t l;   //左侧按键
        uint8_t r;   //右侧按键
    }mouse;

    /* PC 键盘按键数据 */
    union
    {
        uint16_t key_code;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
            uint16_t F1:1;
            uint16_t F2:1;
            uint16_t F3:1;

        }bit;
    }kb;

    /* 遥控器左侧拨轮数据 */
    int16_t wheel;
} RCTypeDef;

/**
  * @brief     遥控器拨杆数据枚举
  */
enum
{
    RC_UP = 1,
    RC_MI = 3,
    RC_DN = 2,
};



/**
 * @brief               遥控初始化
 * @param rx1_buf       内存缓冲区1
 * @param rx2_buf       内存缓冲区2
 * @param dma_buf_num   数据长度
 */
void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

/**
 * @brief   遥控初始化，无需参数
 */
void Remote_Control_init(void);

/**
 * @brief         遥控数据处理
 * @param rc      储存遥控数据的结构体
 * @param buff    遥控数据缓冲区
 */
void Remote_Data_handle(RCTypeDef *rc, uint8_t *buff);

/**
 * @brief              串口发送数据
 * @param uart_id      串口ID
 * @param send_data    发送的数据
 * @param size         数据长度
 */
void Uart_Send(uint8_t uart_id, uint8_t *send_data, uint16_t size);


extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern RCTypeDef rc;
extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

#endif //HNU_RM_DOWN_BSP_UART_H
