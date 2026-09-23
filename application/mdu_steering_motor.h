/**
  ******************************************************************************
  * @file       mdu_steering_motor.h
  * @brief      MDU-M / 科亚 173 智能方向盘舵机驱动头文件 (绝对值编码器零点标定版)
  ******************************************************************************
  */
#ifndef MDU_STEERING_MOTOR_H
#define MDU_STEERING_MOTOR_H

#include "main.h"

/* ==============================================================================
 *  【用户参数配置区】方向盘舵机绝对值编码器零点标定偏移量 (度)
 *  当拖拉机前轮处于机械正中直行位置 (0度) 时：
 *  若舵机原始读数 actual_angle_deg 为 Offset，在此填入对应值即可实现零点对齐：
 *  calibrated_angle_deg = actual_angle_deg - STEERING_ZERO_OFFSET_DEG
 * ============================================================================== */
#define STEERING_ZERO_OFFSET_DEG   (0.0f)   // 舵机绝对值编码器零点偏移量 (度)

typedef struct {
    float    actual_angle_deg;     // 舵机原始绝对机械角度 (度)
    float    calibrated_angle_deg; // 零点标定后的实际转向角度 (度, 0度为正前)
    float    zero_offset_deg;      // 零点偏移量 (度)
    float    target_angle_deg;     // 目标转向角度 (度)
    float    home_angle_deg;       // 捕获的原点角度
    uint8_t  home_captured;        // 原点捕获标志
    float    actual_speed_rpm;     // 实际转速 (rpm)
    uint8_t  is_enabled;           // 使能状态 (1:使能, 0:失能)
    uint8_t  error_flag;           // 故障标志
    uint8_t  hand_override_flag;   // 人工抢盘标志
    uint32_t last_rx_tick;         // 上次收到报文时间戳
} mdu_steering_motor_t;

extern mdu_steering_motor_t g_mdu_steering_motor;

void MDU_Motor_Init(CAN_HandleTypeDef *hcan);
void MDU_Motor_Enable(CAN_HandleTypeDef *hcan);
void MDU_Motor_Disable(CAN_HandleTypeDef *hcan);
void MDU_Motor_SetSpeed(CAN_HandleTypeDef *hcan, float speed_rpm);
void MDU_Motor_SetAngle(CAN_HandleTypeDef *hcan, float target_angle_deg);
void MDU_Motor_SetCalibratedAngle(CAN_HandleTypeDef *hcan, float target_calibrated_deg);
float MDU_Motor_GetCalibratedAngle(void);
void MDU_Motor_SetZeroOffset(float offset_deg);
void MDU_Motor_CalibrateZero(void);
void MDU_Motor_ProcessCANMessage(uint32_t can_id, const uint8_t *data, uint8_t len);
void MDU_Motor_Control_Loop(CAN_HandleTypeDef *hcan);

#endif /* MDU_STEERING_MOTOR_H */
