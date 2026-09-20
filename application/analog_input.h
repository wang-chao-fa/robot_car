/**
  ******************************************************************************
  * @file       analog_input.h
  * @brief      模拟量 / 前轮角度传感器采集驱动头文件
  ******************************************************************************
  */
#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include "main.h"

typedef struct {
    uint8_t  online_flag;
    float    voltage_v[4];
    uint32_t last_rx_tick;
} analog_input_module_t;

extern analog_input_module_t g_analog_input_module;

void Analog_Input_Init(CAN_HandleTypeDef *hcan);
void Analog_Input_Process_CAN(uint32_t std_id, const uint8_t *data, uint8_t len);
float Analog_Input_GetChannel1_Angle0To90(void);

#endif /* ANALOG_INPUT_H */
