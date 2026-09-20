/**
  ******************************************************************************
  * @file       CAN_receive.c
  * @brief      CAN 总线数据接收中断与集中分发
  ******************************************************************************
  */
#include "CAN_receive.h"
#include "kinco_canopen.h"
#include "mdu_steering_motor.h"
#include "relay_output.h"
#include "analog_input.h"
#include "tractor_ctrl.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            return;
        }

        uint32_t can_id = (rx_header.IDE == CAN_ID_EXT) ? rx_header.ExtId : rx_header.StdId;
        uint8_t dlc = (uint8_t)rx_header.DLC;

        // 1. MDU / 科亚 173 方向盘舵机 (扩展帧 0x07000001, 0x07000007, 0x05800001, 0x05800007 或标准帧)
        if (can_id == 0x07000001 || can_id == 0x07000007 ||
            can_id == 0x05800001 || can_id == 0x05800007 ||
            can_id == 0x241 || can_id == 0x011 || can_id == 0x017 || can_id == 0x0C0402A1)
        {
            MDU_Motor_ProcessCANMessage(can_id, rx_data, dlc);
            continue;
        }

        // 2. 继电器模块 (标准帧 0x101)
        if (can_id == 0x101)
        {
            relay_process_can_message(can_id, rx_data, dlc);
            continue;
        }

        // 3. 模拟量采集模块 (标准帧 0x181, 0x281)
        if (can_id == 0x181 || can_id == 0x281)
        {
            Analog_Input_Process_CAN(can_id, rx_data, dlc);
            continue;
        }

        // 4. 步科电机 (ID 2~6, 标准帧)
        if (rx_header.IDE == CAN_ID_STD)
        {
            uint8_t node_id = (uint8_t)(can_id & 0x7F);
            if (node_id == 2)      kinco_recv_handler(&g_motor_gearshift2, can_id, rx_data, dlc);
            else if (node_id == 3) kinco_recv_handler(&g_motor_brake, can_id, rx_data, dlc);
            else if (node_id == 4) kinco_recv_handler(&g_motor_gearshift, can_id, rx_data, dlc);
            else if (node_id == 5) kinco_recv_handler(&g_motor_clutch, can_id, rx_data, dlc);
            else if (node_id == 6) kinco_recv_handler(&g_motor_throttle, can_id, rx_data, dlc);
        }
    }
}
