/**
  ******************************************************************************
  * @file       analog_input.c
  * @brief      模拟量 / 前轮角度传感器采集驱动实现
  ******************************************************************************
  */
#include "analog_input.h"
#include <string.h>

analog_input_module_t g_analog_input_module = {0};

void Analog_Input_Init(CAN_HandleTypeDef *hcan)
{
    memset(&g_analog_input_module, 0, sizeof(g_analog_input_module));
}

void Analog_Input_Process_CAN(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < 4) return;

    if (std_id == 0x181 || std_id == 0x281)
    {
        g_analog_input_module.online_flag = 1;
        g_analog_input_module.last_rx_tick = HAL_GetTick();

        uint16_t raw_ch1 = (uint16_t)(data[0] | (data[1] << 8));
        g_analog_input_module.voltage_v[0] = (float)raw_ch1 * (5.0f / 65535.0f);
    }
}

float Analog_Input_GetChannel1_Angle0To90(void)
{
    // 电压 0~5V 线性换算为 0~90.0 度
    float v = g_analog_input_module.voltage_v[0];
    float angle = v * (90.0f / 5.0f);
    if (angle < 0.0f)  angle = 0.0f;
    if (angle > 90.0f) angle = 90.0f;
    return angle;
}
