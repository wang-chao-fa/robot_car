/**
  ******************************************************************************
  * @file       inclinometer.c
  * @brief      VALUER 双倾角传感器 (前轮 0x18B + 车身 0x18C) CANopen 驱动与差分解算实现
  ******************************************************************************
  */
#include "inclinometer.h"
#include <string.h>

inclinometer_t g_inclinometer = {0};      // 传感器 1: 前轮转向轴 (0x18B)
inclinometer_t g_inclinometer_body = {0}; // 传感器 2: 车身底盘基准 (0x18C)

void Inclinometer_Init(void)
{
    memset(&g_inclinometer, 0, sizeof(g_inclinometer));
    g_inclinometer.roll_offset_deg = 0.0f;

    memset(&g_inclinometer_body, 0, sizeof(g_inclinometer_body));
    g_inclinometer_body.roll_offset_deg = 0.0f;
}

static void Inclinometer_Unpack(inclinometer_t *sensor, const uint8_t *data, uint8_t len)
{
    sensor->last_rx_tick = HAL_GetTick();
    sensor->is_online = 1;
    sensor->rx_count++;

    /* 1. 解析 Roll 横滚角 (B0, B1) - 16位有符号整型，小端模式，缩放系数 0.01° */
    int16_t raw_roll = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
    float raw_roll_deg = (float)raw_roll * 0.01f;
    sensor->roll_deg = raw_roll_deg - sensor->roll_offset_deg;

    /* 2. 解析 Pitch 俯仰角 (B2, B3) - 16位有符号整型，小端模式，缩放系数 0.01° */
    int16_t raw_pitch = (int16_t)(data[2] | ((uint16_t)data[3] << 8));
    sensor->pitch_deg = (float)raw_pitch * 0.01f;

    /* 3. 解析 Yaw 航向角 (B4, B5) - 16位有符号整型，小端模式，缩放系数 0.01° */
    int16_t raw_yaw = (int16_t)(data[4] | ((uint16_t)data[5] << 8));
    sensor->yaw_deg = (float)raw_yaw * 0.01f;

    /* 4. 解析传感器内部温度 (B6) - 8位数值，公式: Temp = B6 / 2 - 40 */
    sensor->temperature_c = (float)data[6] / 2.0f - 40.0f;

    if (len >= 8)
    {
        sensor->status_byte = data[7];
    }
}

void Inclinometer_ProcessCAN(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < 7) return;

    /* 1. 匹配前轮转向轴传感器 (ID: 0x18B = 0x180 + 0x0B) */
    if (std_id == INCLINOMETER_WHEEL_CAN_ID || std_id == 0x18B)
    {
        Inclinometer_Unpack(&g_inclinometer, data, len);
    }
    /* 2. 匹配车身底盘基准传感器 (ID: 0x18C = 0x180 + 0x0C) */
    else if (std_id == INCLINOMETER_BODY_CAN_ID || std_id == 0x18C)
    {
        Inclinometer_Unpack(&g_inclinometer_body, data, len);
    }
}

uint8_t Inclinometer_IsOnline(void)
{
    if (g_inclinometer.rx_count == 0)
    {
        return 0;
    }

    if ((HAL_GetTick() - g_inclinometer.last_rx_tick) > INCLINOMETER_TIMEOUT_MS)
    {
        g_inclinometer.is_online = 0;
    }
    return g_inclinometer.is_online;
}

uint8_t Inclinometer_Body_IsOnline(void)
{
    if (g_inclinometer_body.rx_count == 0)
    {
        return 0;
    }

    if ((HAL_GetTick() - g_inclinometer_body.last_rx_tick) > INCLINOMETER_TIMEOUT_MS)
    {
        g_inclinometer_body.is_online = 0;
    }
    return g_inclinometer_body.is_online;
}

float Inclinometer_GetSteerAngle_Deg(void)
{
    uint8_t wheel_online = Inclinometer_IsOnline();
    uint8_t body_online  = Inclinometer_Body_IsOnline();

    if (wheel_online && body_online)
    {
        /* 双传感器差分模式: (前轮横滚角 - 车身底盘横滚角)，彻底抵消车身侧倾与地面坡度！ */
        return (g_inclinometer.roll_deg - g_inclinometer_body.roll_deg);
    }
    else if (wheel_online)
    {
        /* 单传感器回退模式: 车身传感器离线时平滑保底 */
        return g_inclinometer.roll_deg;
    }
    
    return 0.0f;
}

void Inclinometer_CalibrateZero(void)
{
    g_inclinometer.roll_offset_deg += g_inclinometer.roll_deg;
    g_inclinometer_body.roll_offset_deg += g_inclinometer_body.roll_deg;
}

