/**
  ******************************************************************************
  * @file       inclinometer.h
  * @brief      VALUER 倾角传感器 CANopen (T_PDO1 0x18B) 驱动头文件
  ******************************************************************************
  */
#ifndef INCLINOMETER_H
#define INCLINOMETER_H

#include "main.h"

/* 倾角传感器 CAN 通信与参数配置 */
#define INCLINOMETER_NODE_ID            (0x0B)                  // 节点号 11 (0x0B)
#define INCLINOMETER_PDO1_CAN_ID        (0x180 + INCLINOMETER_NODE_ID) // T_PDO1 数据帧 ID (0x18B)
#define INCLINOMETER_TIMEOUT_MS         (300)                   // 传感器掉线判定超时时间 (ms)

typedef struct {
    float    roll_deg;          // 横滚角 (左右倾斜角，单位: 度，分辨率 0.01°)
    float    pitch_deg;         // 俯仰角 (前后倾斜角，单位: 度，分辨率 0.01°)
    float    yaw_deg;           // 航向角 (单位: 度)
    float    temperature_c;     // 内部温度 (单位: ℃)
    uint8_t  status_byte;       // 状态字
    
    float    roll_offset_deg;   // 软件零点偏置 (度)
    uint32_t last_rx_tick;      // 上次接收数据的时间戳 (ms)
    uint8_t  is_online;         // 传感器在线状态 (1:在线, 0:离线)
    uint32_t rx_count;          // 接收帧计数
} inclinometer_t;

extern inclinometer_t g_inclinometer;

/**
  * @brief  初始化倾角传感器数据结构体
  */
void Inclinometer_Init(void);

/**
  * @brief  在 CAN 接收中断中处理倾角传感器报文
  * @param  std_id CAN 标准帧 ID
  * @param  data   CAN 数据指针
  * @param  len    数据长度
  */
void Inclinometer_ProcessCAN(uint32_t std_id, const uint8_t *data, uint8_t len);

/**
  * @brief  检查倾角传感器是否在线
  * @return 1: 在线正常, 0: 掉线/超时
  */
uint8_t Inclinometer_IsOnline(void);

/**
  * @brief  软件水平标定: 将当前 Roll 姿态记录为 0° 偏置
  */
void Inclinometer_CalibrateZero(void);

#endif /* INCLINOMETER_H */
