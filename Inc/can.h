/**
  ******************************************************************************
  * @file       can.h
  * @brief      CAN peripheral initialization for robot_car
  ******************************************************************************
  */
#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern CAN_HandleTypeDef hcan1;

void MX_CAN1_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
