/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   GPIO 引脚配置 (大疆 C板 RGB LED PH10/PH11/PH12)
  ******************************************************************************
  */

#include "gpio.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* 配置板载 RGB LED 引脚默认输出高电平 (熄灭) */
    HAL_GPIO_WritePin(GPIOH, LED_R_Pin|LED_G_Pin|LED_B_Pin, GPIO_PIN_SET);

    /* 配置 PH10 (蓝灯), PH11 (绿灯), PH12 (红灯) 为推挽输出模式 */
    GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|LED_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}
