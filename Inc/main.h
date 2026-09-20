/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : 大疆 RoboMaster C型开发板主头文件
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

void Error_Handler(void);

/* 大疆 RoboMaster C型开发板板载 RGB LED 引脚定义 (低电平点亮) */
#define LED_R_Pin        GPIO_PIN_12
#define LED_R_GPIO_Port  GPIOH
#define LED_G_Pin        GPIO_PIN_11
#define LED_G_GPIO_Port  GPIOH
#define LED_B_Pin        GPIO_PIN_10
#define LED_B_GPIO_Port  GPIOH

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
