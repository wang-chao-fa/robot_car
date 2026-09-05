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
extern UART_HandleTypeDef huart6;   // ROS 通信串口 (TX: PG14, RX: PG9) 115200 8N1
extern DMA_HandleTypeDef  hdma_usart3_rx;

void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART6_UART_Init(void);

/* ROS 下行控制命令全局变量 ($CMD,v,w) */
extern volatile float    g_ros_cmd_v;          // ROS 下发的期望线速度 (m/s)
extern volatile float    g_ros_cmd_w;          // ROS 下发的期望角速度 (rad/s)
extern volatile uint32_t g_ros_cmd_last_time;  // 上次收到 ROS 有效指令的时间戳 (HAL_GetTick)
extern volatile uint8_t  g_ros_cmd_updated;    // 新指令更新标志

void ROS_UART_Rx_Callback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H */
