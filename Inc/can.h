/**
  ******************************************************************************
  * @file    can.h
  * @brief   CAN1 接口初始化与参数定义
  ******************************************************************************
  */
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
