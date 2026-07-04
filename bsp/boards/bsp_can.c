/**
  ******************************************************************************
  * @file       bsp_can.c
  * @brief      CAN1 过滤器初始化
  *             设置为接收所有标准帧 (掩码全0)，由软件根据 COB-ID 区分电机
  ******************************************************************************
  */
#include "bsp_can.h"

extern CAN_HandleTypeDef hcan1;

void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter = {0};

    /* 接收所有标准帧 (掩码全0 = 不过滤) */
    can_filter.FilterActivation     = ENABLE;
    can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    can_filter.FilterIdHigh         = 0x0000;
    can_filter.FilterIdLow          = 0x0000;
    can_filter.FilterMaskIdHigh     = 0x0000;
    can_filter.FilterMaskIdLow      = 0x0000;
    can_filter.FilterBank           = 0;
    can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    can_filter.SlaveStartFilterBank = 14; // 无 CAN2，设置边界防止冲突

    HAL_CAN_ConfigFilter(&hcan1, &can_filter);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}
