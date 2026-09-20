/**
  ******************************************************************************
  * @file       kinco_canopen.h
  * @brief      步科伺服电机 CANopen / CiA 402 协议驱动头文件
  ******************************************************************************
  */
#ifndef KINCO_CANOPEN_H
#define KINCO_CANOPEN_H

#include "main.h"
#include "struct_typedef.h"

#define ENCODER_RESOLUTION          65536  // 步科电机单圈编码器分辨率 (65536 counts/rev)
#define KINCO_CAN_BAUDRATE          500000 // CAN 波特率 500kbps

/* CANopen NMT 命令 */
#define NMT_CS_START_NODE           0x01
#define NMT_CS_STOP_NODE            0x02
#define NMT_CS_ENTER_PRE_OP         0x80
#define NMT_CS_RESET_NODE           0x81
#define NMT_CS_RESET_COMM           0x82

/* CiA 402 操作模式 */
#define KINCO_MODE_PP               1      // 位置模式
#define KINCO_MODE_PV               3      // 速度模式
#define KINCO_MODE_PT               4      // 转矩模式
#define KINCO_MODE_HM               6      // 回零模式

/* CiA 402 控制字命令 */
#define CMD_SHUTDOWN                0x0006
#define CMD_SWITCH_ON               0x0007
#define CMD_DISABLE_VOLTAGE         0x0000
#define CMD_QUICK_STOP              0x0002
#define CMD_DISABLE_OP              0x0007
#define CMD_ENABLE_OP               0x000F
#define CMD_FAULT_RESET             0x0080

/* PP 位置模式控制位 */
#define CTRL_BIT_NEW_POS            (1 << 4) // Bit 4: 新目标位置触发
#define CTRL_BIT_CHANGE_IMM         (1 << 5) // Bit 5: 立即更新
#define CTRL_BIT_REL_POS            (1 << 6) // Bit 6: 相对位置标志

/* CiA 402 状态字标志位 */
#define STATUS_READY_TO_SWITCH_ON   0x0001
#define STATUS_SWITCHED_ON          0x0002
#define STATUS_OPERATION_ENABLED    0x0004
#define STATUS_FAULT                0x0008
#define STATUS_VOLTAGE_ENABLED      0x0010
#define STATUS_QUICK_STOP           0x0020
#define STATUS_SWITCH_ON_DISABLED   0x0040
#define STATUS_WARNING              0x0080
#define STATUS_TARGET_REACHED       0x0400

/* CiA 402 对象字典索引 */
#define OBJ_ERROR_CODE              0x603F
#define OBJ_CONTROLWORD             0x6040
#define OBJ_STATUSWORD              0x6041
#define OBJ_MODES_OF_OPERATION      0x6060
#define OBJ_MODES_OF_OPERATION_DISP 0x6061
#define OBJ_POSITION_ACTUAL         0x6064
#define OBJ_VELOCITY_ACTUAL         0x606C
#define OBJ_TARGET_TORQUE           0x6071
#define OBJ_TARGET_POSITION         0x607A
#define OBJ_PROFILE_VELOCITY        0x6081
#define OBJ_PROFILE_ACC             0x6083
#define OBJ_PROFILE_DEC             0x6084
#define OBJ_TARGET_VELOCITY         0x60FF
#define OBJ_POSITION_WINDOW         0x6067
#define OBJ_MAX_TORQUE              0x6072
#define OBJ_ERROR_REGISTER          0x1001

/* 步科电机数据结构体 */
typedef struct {
    uint8_t   node_id;              // CANopen 节点号 (ID 2~6)
    uint16_t  controlword;          // 发送的控制字
    volatile uint16_t statusword;   // 接收的状态字
    volatile uint16_t error_code;   // 错误代码 (0x603F)
    uint8_t   error_reg;            // 错误寄存器 (0x1001)
    int8_t    mode_of_operation;    // 目标工作模式
    volatile int8_t mode_display;   // 当前显示模式
    int32_t   target_position;      // 目标位置 (脉冲)
    volatile int32_t actual_position;// 实际位置 (脉冲)
    int32_t   home_position;        // 捕获的原点位置
    uint8_t   home_captured;        // 原点捕获标志
    int32_t   target_velocity;      // 目标速度 (RPM)
    volatile int32_t actual_velocity;// 实际速度 (内部单位)
    uint32_t  profile_speed_rpm;    // 运行速度 (RPM)
    uint32_t  profile_acc_rpm_s;    // 加速度 (RPM/s)
    uint32_t  profile_dec_rpm_s;    // 减速度 (RPM/s)
    volatile uint8_t is_enabled;    // 电机是否已成功使能
    uint8_t   enable_requested;     // 是否请求使能
    volatile uint8_t is_fault;      // 是否故障
    volatile uint8_t target_reached;// 是否到达目标
    uint8_t   enable_step;          // 使能状态机步骤
    uint32_t  last_cmd_time;        // 上次发送命令时间戳
    uint32_t  last_state_cmd_time;  // 上次状态机控制字时间戳
    int32_t   last_sent_position;   // 上次下发的位置
    uint8_t   position_cmd_sent;    // 位置指令已发送标志
    uint8_t   fault_reset_step;     // 故障复位步骤
    uint32_t  fault_reset_time;     // 故障复位时间戳
    uint32_t  last_fault_attempt_time;
} kinco_motor_t;

void kinco_motor_init(kinco_motor_t *motor, uint8_t node_id);
void kinco_nmt_cmd(CAN_HandleTypeDef *hcan, uint8_t node_id, uint8_t cs);
void kinco_sdo_write(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t data, uint8_t len);
void kinco_sdo_read(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index);
void kinco_motor_enable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
void kinco_motor_disable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
void kinco_motor_reset_fault(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
void kinco_set_mode(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int8_t mode);
void kinco_set_velocity(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t rpm);
void kinco_set_torque(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int16_t torque_permille);
void kinco_set_position(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t pos_counts, uint8_t is_relative);
void kinco_set_profile_velocity_custom(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, uint32_t speed_rpm, uint32_t acc_rpm_s, uint32_t dec_rpm_s);
void kinco_motor_config_sync(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, uint32_t speed_rpm, uint32_t acc_rpm_s, uint32_t dec_rpm_s);
void kinco_recv_handler(kinco_motor_t *motor, uint32_t std_id, uint8_t *data, uint8_t dlc);
void kinco_control_loop(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
int32_t kinco_rpm_to_dec(int32_t rpm, uint32_t encoder_res);
int32_t kinco_dec_to_rpm(int32_t dec, uint32_t encoder_res);

#endif /* KINCO_CANOPEN_H */
