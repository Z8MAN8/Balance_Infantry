/**
  ******************************************************************************
  * @file	 bsp_PWM.c
  * @author  Wang Hongxi
  * @version V1.0.0
  * @date    2020/3/1
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_PWM.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value >
    999){
        value = 999;}

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        __HAL_TIM_SetCompare(tim_pwmHandle, TIM_CHANNEL_1, value);
        break;
    case TIM_CHANNEL_2:
        __HAL_TIM_SetCompare(tim_pwmHandle, TIM_CHANNEL_2, value);
        break;
    case TIM_CHANNEL_3:
        __HAL_TIM_SetCompare(tim_pwmHandle, TIM_CHANNEL_3, value);
        break;
    case TIM_CHANNEL_4:
        __HAL_TIM_SetCompare(tim_pwmHandle, TIM_CHANNEL_4, value);
        break;
    }
}
