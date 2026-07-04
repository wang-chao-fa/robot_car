/**
  ******************************************************************************
  * @file       rc_sbus.h
  * @brief      S.BUS 遥控接收解析模块
  *             支持天地飞遥控器 S.BUS 输出协议
  *             16通道, 数值范围: ~172 (最小) ~ 1024 (中位) ~ 1811 (最大)
  ******************************************************************************
  */
#ifndef RC_SBUS_H
#define RC_SBUS_H

#include "main.h"

/* S.BUS 协议一帧固定 25 字节，缓冲区留余量 */
#define SBUS_RX_BUF_NUM   36

/* 解析后 16 个通道数据 (数值范围: ~172 ~ 1811, 中位 ~1024) */
extern uint16_t rc_channels[16];

/* DMA 接收原始数据缓冲区 */
extern uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM];

/* S.BUS 有效帧标志 (超时后自动清除，用于失控保护) */
extern volatile uint8_t sbus_updated;

/* 上次收到帧的时间戳 (ms)，用于失控检测 */
extern volatile uint32_t sbus_last_time;

/* 失控保护超时时间 (ms)：超过此时间没收到遥控帧则紧急停车 */
#define SBUS_FAILSAFE_TIMEOUT_MS   500

void SBUS_Parse(uint8_t *sbus_buf);
void RC_UART_Idle_Callback(UART_HandleTypeDef *huart);

/* 获取单个通道值，并映射为 -1000 ~ +1000 (方便直接用于速度计算) */
int16_t SBUS_GetChannel_Mapped(uint8_t ch_index);

#endif /* RC_SBUS_H */
