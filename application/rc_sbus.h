/**
  ******************************************************************************
  * @file       rc_sbus.h
  * @brief      S.BUS 遥控器接收解析模块头文件
  ******************************************************************************
  */
#ifndef RC_SBUS_H
#define RC_SBUS_H

#include "main.h"

/* S.BUS 协议接收缓冲区大小 */
#define SBUS_RX_BUF_NUM            64

/* 遥控器 16 个通道原始值 (约 172~1811, 中位 1024) */
extern uint16_t rc_channels[16];

/* DMA 接收缓冲区与状态标志 */
extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];
extern volatile uint8_t  sbus_updated;
extern volatile uint32_t sbus_last_time;

/* 遥控器失控保护超时时间 (ms) */
#define SBUS_FAILSAFE_TIMEOUT_MS   500

/* SBUS 遥控器通道定义 */
#define RC_CH_GEARSHIFT   0    // CH1: 1号档位电机 (主变速箱 前进/空档/后退)
#define RC_CH_CLUTCH      1    // CH2: 离合电机     (踩下/松开)
#define RC_CH_THROTTLE    2    // CH3: 油门电机     (摇杆比例开度)
#define RC_CH_BRAKE       3    // CH4: 刹车电机     (踩下/松开)
#define RC_CH_ACTUATOR    5    // CH6: 电动推杆     (伸出/停止/缩回)
#define RC_CH_GEARSHIFT2  6    // CH7: 2号档位电机 (副变速箱 梭式换向)
#define RC_CH_STEERING    7    // CH8: 方向盘舵机   (左右转动)

void SBUS_Parse(uint8_t *sbus_buf);
void RC_UART_Idle_Callback(UART_HandleTypeDef *huart);
int16_t SBUS_GetChannel_Mapped(uint8_t ch_index);

#endif /* RC_SBUS_H */
