/**
  ******************************************************************************
  * @file       CAN_receive.h / CAN_receive.c
  * @brief      CAN 接收中断处理，分发给两个电机
  ******************************************************************************
  */
#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"
#include "kinco_canopen.h"

/* 左电机和右电机全局实例，在 CAN_receive.c 中定义 */
extern kinco_motor_t g_motor_left;
extern kinco_motor_t g_motor_right;

#endif /* CAN_RECEIVE_H */
