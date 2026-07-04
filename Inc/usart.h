/**
  ******************************************************************************
  * @file       usart.h
  * @brief      USART peripheral header for robot_car
  ******************************************************************************
  */
#ifndef __USART_H
#define __USART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef huart1;   // 调试串口 (TX: PA9, RX: PB7)  115200
extern UART_HandleTypeDef huart3;   // SBUS 遥控接收 (RX: PC11)     100000 9bit 偶校验
extern DMA_HandleTypeDef  hdma_usart3_rx;

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
