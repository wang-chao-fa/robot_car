/**
  ******************************************************************************
  * @file       inclinometer.c
  * @brief      VALUER 倾角传感器 CANopen (T_PDO1 0x18B) 驱动实现
  ******************************************************************************
  */
#include "inclinometer.h"
#include <string.h>

inclinometer_t g_inclinometer = {0};

void Inclinometer_Init(void)
{
    memset(&g_inclinometer, 0, sizeof(g_inclinometer));
    g_inclinometer.roll_offset_deg = 0.0f;
}

void Inclinometer_ProcessCAN(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < 7) return;

    /* 匹配倾角传感器 T_PDO1 报文 (ID: 0x18B = 0x180 + 0x0B) */
    if (std_id == INCLINOMETER_PDO1_CAN_ID)
    {
        g_inclinometer.last_rx_tick = HAL_GetTick();
        g_inclinometer.is_online = 1;
        g_inclinometer.rx_count++;

        /* 1. 解析 Roll 横滚角 (B0, B1) - 16位有符号整型，小端模式，缩放系数 0.01° */
        int16_t raw_roll = (int16_t)(data[0] | ((uint16_t)data[1] << 8));
        float raw_roll_deg = (float)raw_roll * 0.01f;
        g_inclinometer.roll_deg = raw_roll_deg - g_inclinometer.roll_offset_deg;

        /* 2. 解析 Pitch 俯仰角 (B2, B3) - 16位有符号整型，小端模式，缩放系数 0.01° */
        int16_t raw_pitch = (int16_t)(data[2] | ((uint16_t)data[3] << 8));
        g_inclinometer.pitch_deg = (float)raw_pitch * 0.01f;

        /* 3. 解析 Yaw 航向角 (B4, B5) - 16位有符号整型，小端模式，缩放系数 0.01° */
        int16_t raw_yaw = (int16_t)(data[4] | ((uint16_t)data[5] << 8));
        g_inclinometer.yaw_deg = (float)raw_yaw * 0.01f;

        /* 4. 解析传感器内部温度 (B6) - 8位数值，公式: Temp = B6 / 2 - 40 */
        g_inclinometer.temperature_c = (float)data[6] / 2.0f - 40.0f;

        if (len >= 8)
        {
            g_inclinometer.status_byte = data[7];
        }
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

void Inclinometer_CalibrateZero(void)
{
    /* 将当前实际实测物理角度累加到零点偏置中，使得当前姿态下的 roll_deg 归零 */
    g_inclinometer.roll_offset_deg += g_inclinometer.roll_deg;
}
