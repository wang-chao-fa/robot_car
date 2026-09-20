/**
  ******************************************************************************
  * @file    usart.h
  * @brief   串口配置头文件 (USART1 调试串口, USART3 遥控器接收, USART6 上位机)
  ******************************************************************************
  */
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
