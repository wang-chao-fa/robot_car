/**
  ******************************************************************************
  * @file       bsp_can.h
  * @brief      CAN 总线底层驱动头文件 (过滤器配置与总线故障恢复)
  ******************************************************************************
  */

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "struct_typedef.h"
#include "can.h"

/**
  * @brief  初始化 CAN1 接收过滤器 (配置为全通接收，允许所有 ID 报文进入中断)
  */
void can_filter_init(void);

/**
  * @brief  CAN 总线错误检测与自动恢复看门狗
  * @param  hcan: CAN 外设句柄指针
  * @note   当检测到总线离线 (BOF) 或严重被动错误时，自动重置外设恢复通信
  */
void CAN_Bus_Error_Recovery(CAN_HandleTypeDef *hcan);

#endif /* BSP_CAN_H */
