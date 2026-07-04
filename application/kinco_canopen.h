#ifndef KINCO_CANOPEN_H
#define KINCO_CANOPEN_H

#include "struct_typedef.h"
#include "main.h"

// CANopen NMT CS 网络管理命令
#define NMT_CS_START_NODE            0x01
#define NMT_CS_STOP_NODE             0x02
#define NMT_CS_ENTER_PRE_OP          0x80
#define NMT_CS_RESET_NODE            0x81
#define NMT_CS_RESET_COMM            0x82

// CANopen DSP402 运行模式 (0x6060)
#define KINCO_MODE_PP                1  // 轮廓位置模式 Profile Position Mode
#define KINCO_MODE_PV                3  // 轮廓速度模式 Profile Velocity Mode
#define KINCO_MODE_PT                4  // 轮廓转矩模式 Profile Torque Mode
#define KINCO_MODE_HM                6  // 找原点模式 Homing Mode

// CANopen DSP402 控制字命令 (0x6040)
#define CMD_SHUTDOWN                 0x0006
#define CMD_SWITCH_ON                0x0007
#define CMD_DISABLE_VOLTAGE          0x0000
#define CMD_QUICK_STOP               0x0002
#define CMD_DISABLE_OP               0x0007
#define CMD_ENABLE_OP                0x000F
#define CMD_FAULT_RESET              0x0080

// 控制字位定义
#define CTRL_BIT_NEW_POS             (1 << 4)   // PP模式: 产生新目标点
#define CTRL_BIT_CHANGE_IMM          (1 << 5)   // PP模式: 立即改变
#define CTRL_BIT_REL_POS             (1 << 6)   // PP模式: 0 = 绝对位置, 1 = 相对位置
#define CTRL_BIT_HALT                (1 << 8)   // 暂停运动

// 状态字位定义 (0x6041)
#define STATUS_READY_TO_SWITCH_ON    0x0001
#define STATUS_SWITCHED_ON           0x0002
#define STATUS_OPERATION_ENABLED     0x0004
#define STATUS_FAULT                 0x0008
#define STATUS_VOLTAGE_ENABLED       0x0010
#define STATUS_QUICK_STOP            0x0020
#define STATUS_SWITCH_ON_DISABLED    0x0040
#define STATUS_WARNING               0x0080
#define STATUS_REMOTE                0x0200
#define STATUS_TARGET_REACHED        0x0400
#define STATUS_INTERNAL_LIMIT        0x0800
#define STATUS_PP_SETPOINT_ACK       0x1000  // PP模式: 接收到目标点 / PV模式: 速度为0
#define STATUS_PP_FOLLOW_ERR         0x2000  // PP模式: 跟随误差

// 对象字典索引 (小端传输)
#define OBJ_CONTROLWORD              0x6040
#define OBJ_STATUSWORD               0x6041
#define OBJ_MODES_OF_OPERATION       0x6060
#define OBJ_MODES_OF_OPERATION_DISP  0x6061
#define OBJ_TARGET_POSITION          0x607A
#define OBJ_PROFILE_ACC              0x6083
#define OBJ_PROFILE_DEC              0x6084
#define OBJ_TARGET_VELOCITY          0x60FF
#define OBJ_POSITION_ACTUAL          0x6064
#define OBJ_VELOCITY_ACTUAL          0x606C

// 步科电机控制结构体 (用于存储电机的实时状态和控制指令)
// 注意: 被中断回调 (kinco_recv_handler) 修改的字段需要 volatile 修饰，
//       以防止编译器优化导致主循环读取到过期值。
typedef struct {
    uint8_t node_id;            // 电机的 CANopen 节点 ID (范围通常是 1-127，必须与驱动器硬件设置一致)
    
    uint16_t controlword;       // 控制字 (对象字典 0x6040)：单片机发送给驱动器的命令，用于控制电机的状态机转换（如使能、复位、急停等）
    volatile uint16_t statusword;        // 状态字 (对象字典 0x6041)：驱动器反馈给单片机的当前状态 [中断写入]
    
    int8_t mode_of_operation;   // 目标运行模式 (对象字典 0x6060)：单片机请求的模式（如 1:位置模式, 3:速度模式, 6:回零模式）
    volatile int8_t mode_display;        // 实际运行模式 (对象字典 0x6061)：驱动器反馈的当前正处于的模式 [中断写入]
    
    int32_t target_velocity;    // 目标速度 (对象字典 0x60FF)：速度模式下，单片机下发给驱动器的目标速度 (单位是驱动器内部的 DEC 单位)
    int32_t target_position;    // 目标位置 (对象字典 0x607A)：位置模式下，单片机下发给驱动器的目标位置 (单位是编码器的脉冲数 Counts)
    
    volatile int32_t actual_velocity;    // 实际速度 (对象字典 0x606C)：驱动器实时反馈的电机当前真实速度 [中断写入]
    volatile int32_t actual_position;    // 实际位置 (对象字典 0x6064)：驱动器实时反馈的电机当前真实位置 [中断写入]
    
    volatile uint8_t is_enabled;         // 电机使能标志位：1 表示单片机期望电机处于使能状态，0 表示期望失能 [中断写入]
    volatile uint8_t is_fault;           // 故障标志位：1 表示驱动器报告了故障 (根据 statusword 的 Bit3 判断) [中断写入]
    volatile uint8_t target_reached;     // 目标到达标志位：1 表示电机已经到达了目标位置或速度 [中断写入]
    
    uint8_t enable_step;        // 使能状态机的内部步数：记录 CiA 402 解锁流程进行到了哪一步 (0:未开始, 1:关闭, 2:准备开启, 3:已开启, 4:完全使能运行)
    uint32_t last_cmd_time;     // 上次发送查询命令的时间戳：用于控制轮询频率，防止 CAN 总线拥堵 (单位：毫秒)
    
    uint8_t fault_reset_step;   // 故障复位状态机步骤 (0:空闲, 1:已发送复位命令等待延时, 2:完成)
    uint32_t fault_reset_time;  // 故障复位命令发送时间戳 (用于非阻塞延时)
} kinco_motor_t;

// 外部声明的全局步科电机实例 (在 CAN_receive.c 中定义，robot_car 工程使用双电机)
extern kinco_motor_t g_motor_left;   // 左电机, NodeID = 2
extern kinco_motor_t g_motor_right;  // 右电机, NodeID = 8

// API 接口函数
void kinco_motor_init(kinco_motor_t *motor, uint8_t node_id);
void kinco_nmt_cmd(CAN_HandleTypeDef *hcan, uint8_t node_id, uint8_t cs);
void kinco_sdo_write(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t data, uint8_t len);
void kinco_sdo_read(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index);

void kinco_motor_enable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
void kinco_motor_disable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
void kinco_motor_reset_fault(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);
void kinco_set_mode(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int8_t mode);

void kinco_set_velocity(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t velocity);
void kinco_set_position(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t position, uint8_t relative);

// 基于使用手册第108页的速度单位转换辅助函数
int32_t kinco_rpm_to_dec(int32_t rpm, int32_t encoder_resolution);
int32_t kinco_dec_to_rpm(int32_t dec, int32_t encoder_resolution);

// 基于使用手册第79/122页的找原点模式函数
void kinco_start_homing(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int8_t method, uint32_t fast_speed, uint32_t slow_speed);

void kinco_recv_handler(kinco_motor_t *motor, uint32_t std_id, uint8_t *data, uint8_t dlc);
void kinco_control_loop(CAN_HandleTypeDef *hcan, kinco_motor_t *motor);

#endif // KINCO_CANOPEN_H
