/**
  ******************************************************************************
  * @file       relay_output.c
  * @brief      8路 CAN 继电器控制盒驱动实现
  ******************************************************************************
  */
#include "relay_output.h"
#include <string.h>

relay_module_t g_relay_module = {0x101, 0, 0};

void relay_init(uint16_t node_id)
{
    g_relay_module.node_id = node_id;
    g_relay_module.states_mask = 0;
    g_relay_module.last_rx_tick = 0;
}

void relay_control_all(CAN_HandleTypeDef *hcan, uint8_t mask)
{
    if (hcan == NULL) return;
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return;

    CAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8] = {0};
    uint32_t send_mail_box;

    tx_header.StdId = 0x100;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;

    g_relay_module.states_mask = mask;
    tx_data[0] = mask;

    HAL_CAN_AddTxMessage(hcan, &tx_header, tx_data, &send_mail_box);
}

void relay_process_can_message(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < 1) return;
    if (std_id == g_relay_module.node_id)
    {
        g_relay_module.states_mask = data[0];
        g_relay_module.last_rx_tick = HAL_GetTick();
    }
}
