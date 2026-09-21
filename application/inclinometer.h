/**
  ******************************************************************************
  * @file       inclinometer.h
  * @brief      VALUER 倾角传感器 CANopen (T_PDO1 0x18B) 驱动头文件
  ******************************************************************************
  */
#ifndef INCLINOMETER_H
#define INCLINOMETER_H

#include "main.h"

/* 倾角传感器 1 (前轮转向轴，Node ID 11 / 0x0B -> 0x18B) */
#define INCLINOMETER_WHEEL_NODE_ID       (0x0B)
#define INCLINOMETER_WHEEL_CAN_ID        (0x180 + INCLINOMETER_WHEEL_NODE_ID) // 0x18B
#define INCLINOMETER_PDO1_CAN_ID         INCLINOMETER_WHEEL_CAN_ID

/* 倾角传感器 2 (车身底盘基准，Node ID 12 / 0x0C -> 0x18C) */
#define INCLINOMETER_BODY_NODE_ID        (0x0C)
#define INCLINOMETER_BODY_CAN_ID         (0x180 + INCLINOMETER_BODY_NODE_ID)  // 0x18C

#define INCLINOMETER_TIMEOUT_MS          (300) // 传感器掉线判定超时时间 (ms)

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

extern inclinometer_t g_inclinometer;      // 兼容原工程: 对应前轮转向轴传感器 (0x18B)
extern inclinometer_t g_inclinometer_body; // 车身底盘基准传感器 (0x18C)

/**
  * @brief  初始化双倾角传感器数据结构体
  */
void Inclinometer_Init(void);

/**
  * @brief  在 CAN 接收中断中处理双倾角传感器报文 (0x18B 与 0x18C)
  * @param  std_id CAN 标准帧 ID
  * @param  data   CAN 数据指针
  * @param  len    数据长度
  */
void Inclinometer_ProcessCAN(uint32_t std_id, const uint8_t *data, uint8_t len);

/**
  * @brief  检查前轮倾角传感器是否在线
  * @return 1: 在线正常, 0: 掉线/超时
  */
uint8_t Inclinometer_IsOnline(void);

/**
  * @brief  检查车身底盘倾角传感器是否在线
  * @return 1: 在线正常, 0: 掉线/超时
  */
uint8_t Inclinometer_Body_IsOnline(void);

/**
  * @brief  获取前轮相对车身的差分纯净转向角 (度)
  * @note   当双传感器均在线时返回 (Roll_wheel - Roll_body)；单传感器时自动平滑回退
  * @return 纯净前轮转向转角 (度，已完全抵消车身倾斜与地面坡度)
  */
float Inclinometer_GetSteerAngle_Deg(void);

/**
  * @brief  软件水平标定
  */
void Inclinometer_CalibrateZero(void);

#endif /* INCLINOMETER_H */

