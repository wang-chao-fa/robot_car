/**
  ******************************************************************************
  * @file       gpio.c
  * @brief      GPIO 初始化 (LED 状态指示)
  ******************************************************************************
  */

#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能时钟 */
    __HAL_RCC_GPIOF_CLK_ENABLE();

    /* 初始化 LED 为高电平(灭) */
    HAL_GPIO_WritePin(GPIOF, LED_R_Pin | LED_G_Pin, GPIO_PIN_SET);

    /* LED_R (PF12), LED_G (PF11) */
    GPIO_InitStruct.Pin   = LED_R_Pin | LED_G_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
}
