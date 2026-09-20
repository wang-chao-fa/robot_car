/**
  ******************************************************************************
  * @file       relay_output.h
  * @brief      8路 CAN 继电器控制盒驱动头文件 (推杆动作执行)
  ******************************************************************************
  */
#ifndef RELAY_OUTPUT_H
#define RELAY_OUTPUT_H

#include "main.h"

#define RELAY_CH1   0
#define RELAY_CH2   1
#define RELAY_CH3   2
#define RELAY_CH4   3
#define RELAY_CH5   4
#define RELAY_CH6   5
#define RELAY_CH7   6
#define RELAY_CH8   7

typedef struct {
    uint16_t node_id;
    uint8_t  states_mask;
    uint32_t last_rx_tick;
} relay_module_t;

extern relay_module_t g_relay_module;

void relay_init(uint16_t node_id);
void relay_control_all(CAN_HandleTypeDef *hcan, uint8_t mask);
void relay_process_can_message(uint32_t std_id, const uint8_t *data, uint8_t len);

#endif /* RELAY_OUTPUT_H */
