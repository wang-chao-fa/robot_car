/**
  ******************************************************************************
  * @file       bsp_can.c
  * @brief      CAN 总线底层驱动实现 (过滤器配置与总线故障恢复)
  ******************************************************************************
  */

#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

/**
  * @brief  初始化 CAN1 硬件接收过滤器 (配置为 32 位掩码全通模式)
  */
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  CAN 总线错误检测与自动恢复函数
  */
void CAN_Bus_Error_Recovery(CAN_HandleTypeDef *hcan)
{
    uint32_t err = HAL_CAN_GetError(hcan);
    if (err != HAL_CAN_ERROR_NONE)
    {
        /* 若检测到 Bus-Off 或被动错误，重新复位并启动 CAN 控制器 */
        if (err & (HAL_CAN_ERROR_BOF | HAL_CAN_ERROR_EPV | HAL_CAN_ERROR_EWG))
        {
            HAL_CAN_Stop(hcan);
            HAL_CAN_ResetError(hcan);
            HAL_CAN_Start(hcan);
            HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
        }
    }
}
