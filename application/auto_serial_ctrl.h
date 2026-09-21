/**
  ******************************************************************************
  * @file       auto_serial_ctrl.h
  * @brief      上位机串口自动驾驶通信协议处理模块 (基于 USART6 全双工)
  *
  * 协议格式说明:
  *   上位机下发控制帧:  $AUTO,<gear>,<clutch>,<throttle>,<brake>,<actuator>,<steer_rpm>,<heartbeat>*CS\r\n
  *   下位机状态回传帧:  $STATE,<wheel_angle>,<steer_angle>,<steer_rpm>,<mode>,<err_mask>,<hb_echo>*CS\r\n
  *
  * 安全保护机制:
  *   1. 通信超时保护 (500ms 内无有效控制帧自动切回安全状态或刹车)
  *   2. 心跳冻结保护 (连续 10 帧心跳计数值不变，判定上位机算法卡死并切断动力)
  *   3. XOR 异或校验和保护 (校验失败直接丢弃，防止乱码误动作)
  ******************************************************************************
  */

#ifndef AUTO_SERIAL_CTRL_H
#define AUTO_SERIAL_CTRL_H

#include "struct_typedef.h"
#include "main.h"

/* ============================================================
 *  安全与超时参数宏定义
 * ============================================================ */
#define SERIAL_AUTO_TIMEOUT_MS        500    // 通信超时阈值 (ms): 超过该时间未收到有效控制帧即切断动力
#define SERIAL_AUTO_HB_FREEZE_COUNT   10     // 心跳冻结阈值: 连续 N 帧心跳值不变判定上位机卡死
#define SERIAL_AUTO_RX_BUF_SIZE       128    // 串口接收单帧最大缓冲区长度 (字节)

/* ============================================================
 *  自动驾驶控制命令与状态结构体
 * ============================================================ */
typedef struct {
    /* --- 控制指令字段 --- */
    uint8_t  gear;              // 1号档位指令: 0: 空档/原点(0), 1: 前进档, 2: 倒车档
    uint8_t  clutch;            // 离合指令: 0: 结合/释放(0), 1: 踩下/分离
    uint8_t  throttle_percent;  // 油门指令开度: 0 ~ 100 (%)
    uint8_t  brake;             // 刹车指令: 0: 松开(0), 1: 踩下刹车
    int8_t   gear2;             // 2号档位换向指令: 0: 停止/原点, 1: 前进换向, 2 或 -1: 倒退换向 (触发自动动作后归零)
    float    steer_speed_rpm;   // 方向盘转向转速: (-100.0 ~ +100.0 RPM, 为0时启用倾角自动回正)

    /* --- 心跳与安全监控字段 --- */
    uint8_t  heartbeat;         // 当前帧心跳计数值 (0~255，上位机每帧递增)
    uint8_t  last_heartbeat;    // 上一帧心跳计数值 (用于判断冻结)
    uint8_t  hb_freeze_counter; // 心跳冻结计数器 (连续相同帧数)

    /* --- 通信与统计状态 --- */
    uint32_t last_valid_time;   // 上次收到有效控制帧的时间戳 (HAL_GetTick)
    uint8_t  is_active;         // 自动驾驶是否已激活 (收到至少 1 帧有效控制帧)
    uint32_t valid_frame_count; // 累计接收到的有效控制帧数
    uint32_t error_frame_count; // 累计校验失败或格式错误帧数
} serial_auto_cmd_t;

extern serial_auto_cmd_t g_serial_auto_cmd;

/* ============================================================
 *  故障与状态掩码 (用于 $STATE 回传帧中的 err_mask 字段)
 * ============================================================ */
#define ERR_MASK_SBUS_LOST       (1u << 0)  // bit0: 遥控器 SBUS 信号丢失
#define ERR_MASK_MOTOR_FAULT     (1u << 1)  // bit1: 伺服驱动器/电机故障
#define ERR_MASK_STEER_OVERRIDE  (1u << 2)  // bit2: 驾驶员人工抢夺方向盘
#define ERR_MASK_SERIAL_HB_ERR   (1u << 3)  // bit3: 上位机串口通信异常 (超时或心跳卡死)

/* ============================================================
 *  API 函数声明
 * ============================================================ */

/**
  * @brief  初始化上位机自动驾驶通信模块
  */
void Serial_Auto_Init(void);

/**
  * @brief  单字节串口解析 (在 USART6 中断或主循环中逐字节输入)
  * @param  ch: 接收到的单字节数据
  */
void Serial_Auto_ParseByte(uint8_t ch);

/**
  * @brief  判断上位机自动驾驶通信链路是否健康有效
  * @retval 1: 通信正常且心跳活跃; 0: 通信超时或心跳冻结
  */
uint8_t Serial_Auto_IsAlive(void);

/**
  * @brief  通过 USART6 向上位机发送 $STATE 状态回传帧 (通常 20ms 周期调用)
  * @param  huart: 串口句柄指针 (如 &huart6)
  * @param  mode:  当前控制模式 (0: 遥控手动, 1: 上位机自动驾驶)
  */
void Serial_Auto_SendFeedback(UART_HandleTypeDef *huart, uint8_t mode);

/**
  * @brief  获取当前系统的故障掩码
  * @retval 8-bit 故障掩码 (ERR_MASK_xxx 的按位或)
  */
uint8_t Serial_Auto_GetErrMask(void);

#endif /* AUTO_SERIAL_CTRL_H */
