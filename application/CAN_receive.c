/**
  ******************************************************************************
  * @file       CAN_receive.c
  * @brief      CAN 接收中断 → 分发给左、右两个步科伺服电机
  *
  * 电机 CAN 节点 ID 对照:
  *   左电机 (g_motor_left):  节点 ID = 2  (来自 ECAN Settings 节点保护ID = 0x00000702)
  *   右电机 (g_motor_right): 节点 ID = 8  (来自 ECAN Settings 节点保护ID = 0x00000708)
  ******************************************************************************
  */
#include "CAN_receive.h"
#include "main.h"

/* 两个电机的全局实例 */
kinco_motor_t g_motor_left;   // 左电机, NodeID = 2
kinco_motor_t g_motor_right;  // 右电机, NodeID = 8

/**
  * @brief  HAL CAN RX FIFO0 回调: 接收所有电机的 CANopen 报文并分发
  * @note   kinco_recv_handler 内部会根据 std_id 判断是否属于自己的节点
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    /* 分发给左电机 */
    kinco_recv_handler(&g_motor_left,  rx_header.StdId, rx_data, rx_header.DLC);
    /* 分发给右电机 */
    kinco_recv_handler(&g_motor_right, rx_header.StdId, rx_data, rx_header.DLC);
}
