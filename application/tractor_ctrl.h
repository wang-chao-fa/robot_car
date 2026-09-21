/**
  ******************************************************************************
  * @file       tractor_ctrl.h
  * @brief      拖拉机改装核心控制头文件 (包含所有电机行程参数定义)
  ******************************************************************************
  */
#ifndef TRACTOR_CTRL_H
#define TRACTOR_CTRL_H

#include "main.h"
#include "rc_sbus.h"
#include "kinco_canopen.h"
#include "mdu_steering_motor.h"
#include "relay_output.h"

extern kinco_motor_t g_motor_gearshift2; // ID 2 (2号档位电机)
extern kinco_motor_t g_motor_brake;      // ID 3 (刹车电机)
extern kinco_motor_t g_motor_gearshift;  // ID 4 (1号档位电机)
extern kinco_motor_t g_motor_clutch;     // ID 5 (离合电机)
extern kinco_motor_t g_motor_throttle;   // ID 6 (油门电机)

/* ==============================================================================
 *  【用户参数配置区】步科伺服电机行程、原点与圈数参数配置
 *  注: 步科电机单圈编码器分辨率为 65536 counts (1 圈 = 65536 脉冲)
 *  用户可在此处直接修改各电机的动作行程（支持圈数或脉冲数设定）
 * ============================================================================== */

/* ---------------- 1. 2号档位电机参数 (Node ID 2, 副变速箱/换向机构) ---------------- */
#define GEAR2_ABS_ORIGIN_POS      (0)                                          // 2号档位 原点位置 (脉冲)
#define GEAR2_ABS_TOTAL_SPAN      ((int32_t)(3.0f * ENCODER_RESOLUTION))       // 动作行程 (5圈 = 327680 脉冲)
#define GEAR2_ABS_FWD_POS         (GEAR2_ABS_ORIGIN_POS + GEAR2_ABS_TOTAL_SPAN)// 前进档目标位置
#define GEAR2_ABS_REV_POS         (GEAR2_ABS_ORIGIN_POS - GEAR2_ABS_TOTAL_SPAN)// 倒退档目标位置
#define GEAR2_SPEED_RPM           800   // 换档转速 (RPM)
#define GEAR2_ACC_RPM_S           1500  // 换档加速度 (RPM/s)
#define GEAR2_FWD_HOLD_TIME_MS    3000  // 前目标点停留时间 (ms，转到前进目标点后停留时间，之后返回原点)
#define GEAR2_REV_HOLD_TIME_MS    2000  // 后目标点停留时间 (ms，转到倒退目标点后停留时间，之后返回原点)

/* ---------------- 2. 1号档位电机参数 (Node ID 4, 主变速箱) ---------------- */
#define GEAR_ABS_ORIGIN_POS       (-2310054)                                   // 1号档位 空档原点位置 (脉冲)
#define GEAR_ABS_TOTAL_SPAN       ((int32_t)(6.0f * ENCODER_RESOLUTION))       // 换档行程 (6圈 = 393216 脉冲)
#define GEAR_ABS_FWD_POS          (GEAR_ABS_ORIGIN_POS + GEAR_ABS_TOTAL_SPAN)  // 前进档位置
#define GEAR_ABS_REV_POS          (GEAR_ABS_ORIGIN_POS - GEAR_ABS_TOTAL_SPAN)  // 倒退档位置
#define GEAR_POS_NEUTRAL          GEAR_ABS_ORIGIN_POS
#define GEAR_POS_FORWARD          GEAR_ABS_FWD_POS
#define GEAR_POS_REVERSE          GEAR_ABS_REV_POS

/* ---------------- 3. 刹车电机参数 (Node ID 3) ---------------- */
#define BRAKE_ABS_ORIGIN_POS      (1583752)                                    // 刹车松开位置 (原点脉冲)
#define BRAKE_ABS_TOTAL_SPAN      ((int32_t)(1.2f * ENCODER_RESOLUTION))       // 刹车踩下行程 (1.2圈 = 78643 脉冲)
#define BRAKE_ABS_MAX_POS         (BRAKE_ABS_ORIGIN_POS - BRAKE_ABS_TOTAL_SPAN)// 刹车踩紧位置
#define BRAKE_POS_RELEASED        BRAKE_ABS_ORIGIN_POS
#define BRAKE_POS_APPLIED         BRAKE_ABS_MAX_POS

/* ---------------- 4. 离合电机参数 (Node ID 5) ---------------- */
#define CLUTCH_ABS_ORIGIN_POS     (-1028172)                                   // 离合结合位置 (原点脉冲)
#define CLUTCH_ABS_TOTAL_SPAN     ((int32_t)(40.0f * ENCODER_RESOLUTION))      // 离合踩下行程 (45圈 = 2949120 脉冲)
#define CLUTCH_ABS_MAX_POS        (CLUTCH_ABS_ORIGIN_POS - CLUTCH_ABS_TOTAL_SPAN) // 离合踩下分离位置

#define CLUTCH_PRESS_SPEED_RPM    1500  // 踩下离合速度 (RPM)
#define CLUTCH_PRESS_ACC_RPM_S    3000  // 踩下离合加速度 (RPM/s)
#define CLUTCH_RELEASE_SPEED_RPM  1000  // 松开离合平稳速度 (RPM)
#define CLUTCH_RELEASE_ACC_RPM_S  1500  // 松开离合平稳减速度 (RPM/s)

/* ---------------- 5. 油门电机参数 (Node ID 6) ---------------- */
#define THROTTLE_ABS_ORIGIN_POS   (-162877904)                                 // 油门 0% 怠速原点脉冲
#define THROTTLE_ABS_MAX_POS      (-162890373)                                 // 油门 100% 最大油门脉冲
#define THROTTLE_ABS_TOTAL_SPAN   (THROTTLE_ABS_MAX_POS - THROTTLE_ABS_ORIGIN_POS) // 油门总行程脉冲

/* ---------------- 6. 方向盘电机参数 (ID 7) ---------------- */
#define STEERING_MAX_ANGLE_DEG           1800.0f // 遥控打方向最大转动角度 (±1800度 = ±5圈)
#define SERIAL_AUTO_STEER_MAX_INPUT      (60.0f)   // 上位机遥控打满时的极值 (根据实测上位机下发 ±60.0)
#define SERIAL_AUTO_STEER_POLARITY       (-1)      // 上位机遥控方向盘极性: 1 为正常, -1 为反向 (如果方向反了直接改成 1)

/* ---------------- 7. 前轮转向轴角度传感器分段减速平滑回正参数 ---------------- */
#define FRONT_WHEEL_ZERO_ROLL_DEG        (-8.91f)  // 实测前轮绝对正中基准角度 (度)
#define STEER_CLOSED_LOOP_DEADBAND_DEG   (0.20f)   // 前轮回正对中死区 (度，在 ±0.20° 内停转锁定)
#define STEER_SLOWDOWN_THRESHOLD_DEG     (1.50f)   // 开始减速距离阈值 (度，偏差 < 1.50° 时自动降速防震荡)
#define STEER_TRACK_FAST_SPEED_DPS       (90.0f)   // 远距离恒速快速纠偏速度 (度/秒)
#define STEER_TRACK_SLOW_SPEED_DPS       (15.0f)   // 临近零点最低逼近速度 (度/秒)
#define STEER_CORRECT_DIR_POLARITY       (-1)      // 【纠偏方向极性】: 1 为正常方向, -1 为反转方向 (已校准为 -1)

/* 三档与两档开关解析 */
typedef enum {
    SWITCH_POS_UP   = 1,
    SWITCH_POS_MID  = 0,
    SWITCH_POS_DOWN = -1
} switch_3pos_t;

typedef enum {
    SWITCH_POS_OFF  = 0,
    SWITCH_POS_ON   = 1
} switch_2pos_t;

typedef struct {
    uint8_t  is_active;          // 控制输入有效标志
    int8_t   gear;               // 1号档位 (1:前进, 0:空档, 2:倒退)
    int8_t   gear2;              // 2号档位 (1:前进, 0:停止, 2:倒退)
    uint8_t  clutch;             // 离合 (0:松开结合, 1:踩下脱开)
    float    throttle_pct;       // 油门百分比 (0.0% ~ 100.0%)
    uint8_t  brake;              // 刹车 (0:松开, 1:踩下)
    int8_t   actuator_dir;       // 推杆方向 (1:伸出, 0:停止, -1:缩回)
    float    steer_target_deg;   // 方向盘目标角度 (度)
    uint8_t  steer_is_neutral;   // 方向盘摇杆是否处于中位 (1:中位启用闭环, 0:打方向手动优先)
} tractor_demand_t;

extern float g_steer_closed_loop_adj_deg; // 全局当前闭环纠偏补偿值
extern uint8_t g_tractor_control_mode;    // 全局当前控制模式 (0: 遥控手动, 1: 上位机自动)

void TractorControl_GetSBUSDemand(tractor_demand_t *demand);
void TractorControl_ExecuteDemand(CAN_HandleTypeDef *hcan, const tractor_demand_t *demand);
void TractorControl_Update(CAN_HandleTypeDef *hcan);
void TractorControl_Failsafe(CAN_HandleTypeDef *hcan);
switch_3pos_t TractorControl_Parse3Pos(int16_t mapped_val);
switch_2pos_t TractorControl_Parse2Pos(int16_t mapped_val);

#endif /* TRACTOR_CTRL_H */
