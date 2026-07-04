/**
  ******************************************************************************
  * @file       diff_drive.h
  * @brief      差速小车运动学解算模块
  *
  * 单遥控控制方案:
  *   一个摇杆轴 → 前进/后退 (纵轴: 速度 V)
  *   另一个摇杆轴 → 左转/右转 (横轴: 转向 W)
  *
  * 差速解算:
  *   左轮速度 = V + W
  *   右轮速度 = V - W
  *
  * 通道分配 (待用户调试后由 main.c 中的宏配置，这里提供接口):
  *   CHANNEL_THROTTLE: 前进/后退通道 (纵向)
  *   CHANNEL_STEERING: 转向通道 (横向)
  ******************************************************************************
  */
#ifndef DIFF_DRIVE_H
#define DIFF_DRIVE_H

#include "struct_typedef.h"
#include "kinco_canopen.h"

/* ============================================================
 *  通道分配: 左摇杆单摇杆控制
 *    CH1 (index 0) = 左摇杆 X 轴 → 左右转向 (STEERING)
 *    CH2 (index 1) = 左摇杆 Y 轴 → 前进后退 (THROTTLE)
 * ============================================================ */
#define RC_CH_THROTTLE    1    // 前进/后退: CH2 左摇杆纵轴 (Y轴)
#define RC_CH_STEERING    0    // 左右转向: CH1 左摇杆横轴 (X轴)

/* 最大转速 RPM (根据实际小车调整) */
#define MAX_SPEED_RPM     500

/* 编码器分辨率 (步科驱动器 16位多圈绝对值编码器) */
#define ENCODER_RESOLUTION  65536

/* 转向混合权重 (0~100, 越大转弯越灵活) */
#define STEERING_MIX_RATIO  80

/**
  * @brief  差速解算并输出到两个电机
  * @param  throttle_mapped: 前进/后退量 (-1000 ~ +1000)
  * @param  steering_mapped: 左右转向量 (-1000 ~ +1000, 正值右转)
  * @param  motor_left:  左电机结构体指针
  * @param  motor_right: 右电机结构体指针
  * @param  hcan:        CAN 句柄
  */
void DiffDrive_Update(int16_t throttle_mapped, int16_t steering_mapped,
                      kinco_motor_t *motor_left, kinco_motor_t *motor_right,
                      CAN_HandleTypeDef *hcan);

#endif /* DIFF_DRIVE_H */
