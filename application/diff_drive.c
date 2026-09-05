/**
  ******************************************************************************
  * @file       diff_drive.c
  * @brief      差速小车运动学解算实现
  *
  *  差速解算公式 (单摇杆控制):
  *    左轮速 = (V + W * ratio) / 1000 * MAX_RPM
  *    右轮速 = (V - W * ratio) / 1000 * MAX_RPM
  *
  *  其中:
  *    V = 前进/后退量 (-1000 ~ +1000)
  *    W = 转向量      (-1000 ~ +1000, 正值右转)
  *    ratio = STEERING_MIX_RATIO / 100.0
  *
  *  左电机安装方向: 若为镜像安装 (左电机转向与右电机相反才能让小车前进)
  *  则左电机速度需要取反 (在 main.c 宏 MOTOR_LEFT_INVERT 控制)
  ******************************************************************************
  */
#include "diff_drive.h"
#include "rc_sbus.h"

/* 
 * 左电机方向反转: 因为两侧电机安装方向相反
 * 若左右电机转向定义相同时小车前进，则设为 0
 * 若左右电机转向定义相反时小车前进，则设为 1 (默认差速小车常见)
 */
#define MOTOR_LEFT_INVERT   1   // 根据实际安装方向调整 (1=反转左电机)

/**
  * @brief  差速解算并写入两个电机目标速度
  * @param  throttle_mapped: 前进/后退量 (-1000 ~ +1000)
  * @param  steering_mapped: 左右转向量 (-1000 ~ +1000, 正值右转)
  * @param  motor_left:      左电机
  * @param  motor_right:     右电机
  * @param  hcan:            CAN 句柄
  */
void DiffDrive_Update(int16_t throttle_mapped, int16_t steering_mapped,
                      kinco_motor_t *motor_left, kinco_motor_t *motor_right,
                      CAN_HandleTypeDef *hcan)
{
    /* 差速混合: 计算左右轮目标速度 (单位: RPM) */
    /* steering_mapped 正值右转，因为转向反了，这里乘以 -1 进行反向修正 */
    int32_t turn_contribution = -((int32_t)steering_mapped * STEERING_MIX_RATIO / 100);

    int32_t left_rpm  = (int32_t)throttle_mapped * MAX_SPEED_RPM / 1000 + turn_contribution * MAX_SPEED_RPM / 1000;
    int32_t right_rpm = (int32_t)throttle_mapped * MAX_SPEED_RPM / 1000 - turn_contribution * MAX_SPEED_RPM / 1000;

    /* 限幅 */
    if (left_rpm  >  MAX_SPEED_RPM) left_rpm  =  MAX_SPEED_RPM;
    if (left_rpm  < -MAX_SPEED_RPM) left_rpm  = -MAX_SPEED_RPM;
    if (right_rpm >  MAX_SPEED_RPM) right_rpm =  MAX_SPEED_RPM;
    if (right_rpm < -MAX_SPEED_RPM) right_rpm = -MAX_SPEED_RPM;

    /* 左电机方向修正 (镜像安装时需要反转) */
#if MOTOR_LEFT_INVERT
    left_rpm = -left_rpm;
#endif

    /* 转换为驱动器 DEC 单位并发送 */
    int32_t left_dec  = kinco_rpm_to_dec(left_rpm,  ENCODER_RESOLUTION);
    int32_t right_dec = kinco_rpm_to_dec(right_rpm, ENCODER_RESOLUTION);

    kinco_set_velocity(hcan, motor_left,  left_dec);
    kinco_set_velocity(hcan, motor_right, right_dec);
}

/**
  * @brief  根据左右轮编码器反馈解算小车整体线速度与角速度 (供 ROS 里程计使用)
  * @param  motor_left:  左电机结构体指针
  * @param  motor_right: 右电机结构体指针
  * @param  out_linear_v:  输出中心线速度 (m/s)
  * @param  out_angular_w: 输出旋转角速度 (rad/s, 逆时针/左转为正)
  */
void DiffDrive_GetOdomVelocity(kinco_motor_t *motor_left, kinco_motor_t *motor_right,
                               float *out_linear_v, float *out_angular_w)
{
    if (motor_left == NULL || motor_right == NULL || out_linear_v == NULL || out_angular_w == NULL)
    {
        return;
    }

    /* 1. 读取电机反馈速度转换为 RPM */
    int32_t raw_left_rpm  = kinco_dec_to_rpm(motor_left->actual_velocity,  ENCODER_RESOLUTION);
    int32_t raw_right_rpm = kinco_dec_to_rpm(motor_right->actual_velocity, ENCODER_RESOLUTION);

    /* 2. 镜像安装方向修正 (使正转统一对应物理小车前进) */
#if MOTOR_LEFT_INVERT
    raw_left_rpm = -raw_left_rpm;
#endif

    /* 3. 换算左右轮实际物理线速度 (m/s) */
    float v_left  = (float)raw_left_rpm  * RPM_TO_MS_RATIO;
    float v_right = (float)raw_right_rpm * RPM_TO_MS_RATIO;

    /* 4. 差速正运动解算: 中心线速度 v (m/s) 与 角速度 w (rad/s) */
    *out_linear_v  = -((v_right + v_left) / 2.0f);
    *out_angular_w = (v_right - v_left) / ROBOT_WHEEL_TRACK;
}

/**
  * @brief  根据 ROS 的期望线速度 linear_v 与角速度 angular_w 逆解算输出到电机
  * @param  linear_v:  期望线速度 (m/s)
  * @param  angular_w: 期望角速度 (rad/s)
  * @param  motor_left:  左电机结构体指针
  * @param  motor_right: 右电机结构体指针
  
  * @param  hcan:        CAN 句柄
  */
void DiffDrive_UpdateFromROS(float linear_v, float angular_w,
                             kinco_motor_t *motor_left, kinco_motor_t *motor_right,
                             CAN_HandleTypeDef *hcan)
{
    if (motor_left == NULL || motor_right == NULL || hcan == NULL)
    {
        return;
    }

    /* 反转 ROS 线速度，修正正向前进/反向后退方向 */
    linear_v = -linear_v;

    /* 1. 逆运动学解算左右轮期望线速度 (m/s) */
    float v_left  = linear_v - (angular_w * ROBOT_WHEEL_TRACK / 2.0f);
    float v_right = linear_v + (angular_w * ROBOT_WHEEL_TRACK / 2.0f);

    /* 2. 转换为电机期望转速 (RPM) */
    int32_t left_rpm  = (int32_t)(v_left  / RPM_TO_MS_RATIO);
    int32_t right_rpm = (int32_t)(v_right / RPM_TO_MS_RATIO);

    /* 3. 限幅 */
    if (left_rpm  >  MAX_SPEED_RPM) left_rpm  =  MAX_SPEED_RPM;
    if (left_rpm  < -MAX_SPEED_RPM) left_rpm  = -MAX_SPEED_RPM;
    if (right_rpm >  MAX_SPEED_RPM) right_rpm =  MAX_SPEED_RPM;
    if (right_rpm < -MAX_SPEED_RPM) right_rpm = -MAX_SPEED_RPM;

    /* 4. 左电机方向修正 (镜像安装) */
#if MOTOR_LEFT_INVERT
    left_rpm = -left_rpm;
#endif

    /* 5. 转换为驱动器 DEC 单位并下发 */
    int32_t left_dec  = kinco_rpm_to_dec(left_rpm,  ENCODER_RESOLUTION);
    int32_t right_dec = kinco_rpm_to_dec(right_rpm, ENCODER_RESOLUTION);

    kinco_set_velocity(hcan, motor_left,  left_dec);
    kinco_set_velocity(hcan, motor_right, right_dec);
}

