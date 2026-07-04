#include "kinco_canopen.h"
#include "main.h"

// 注意: robot_car 工程使用双电机实例 (g_motor_left, g_motor_right)
// 全局实例在 CAN_receive.c 中定义，此处不重复定义

// CAN 发送超时时间 (毫秒): 等待邮箱空闲的最大时长
#define CAN_SEND_TIMEOUT_MS  5

/**
  * @brief          发送 CAN 数据帧 (带邮箱满重试和错误检查)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      id: 标准帧 ID
  * @param[in]      data: 数据缓冲区指针
  * @param[in]      dlc: 数据长度 (0-8)
  * @retval         0: 发送成功, 1: 发送失败 (邮箱满超时或 HAL 错误)
  */
static uint8_t can_send_frame(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header = {0};
    uint32_t send_mail_box;
    
    tx_header.StdId = id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    
    // 等待至少一个发送邮箱空闲，最多等待 CAN_SEND_TIMEOUT_MS 毫秒
    uint32_t start_tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        if (HAL_GetTick() - start_tick >= CAN_SEND_TIMEOUT_MS)
        {
            return 1; // 超时: 所有邮箱持续忙碌
        }
    }
    
    if (HAL_CAN_AddTxMessage(hcan, &tx_header, data, &send_mail_box) != HAL_OK)
    {
        return 1; // HAL 错误
    }
    
    return 0; // 发送成功
}

/**
  * @brief          初始化步科电机结构体
  * @param[out]     motor: 电机结构体指针
  * @param[in]      node_id: CANopen 节点 ID (1-127)
  */
void kinco_motor_init(kinco_motor_t *motor, uint8_t node_id)
{
    motor->node_id = node_id;
    motor->controlword = 0;
    motor->statusword = 0;
    motor->mode_of_operation = 0;
    motor->mode_display = 0;
    motor->target_velocity = 0;
    motor->target_position = 0;
    motor->actual_velocity = 0;
    motor->actual_position = 0;
    motor->is_enabled = 0;
    motor->is_fault = 0;
    motor->target_reached = 0;
    motor->enable_step = 0;
    motor->last_cmd_time = 0;
    motor->fault_reset_step = 0;
    motor->fault_reset_time = 0;
}

/**
  * @brief          发送 CANopen NMT 网络管理命令
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      node_id: 节点 ID (0 表示所有节点，1-127 表示特定节点)
  * @param[in]      cs: 命令修饰符 (例如 NMT_CS_START_NODE)
  */
void kinco_nmt_cmd(CAN_HandleTypeDef *hcan, uint8_t node_id, uint8_t cs)
{
    uint8_t data[2];
    data[0] = cs;
    data[1] = node_id;
    
    can_send_frame(hcan, 0x000, data, 2);
}

/**
  * @brief          发送 CANopen SDO 写命令 (加速下载)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      node_id: 节点 ID (1-127)
  * @param[in]      index: 对象字典索引
  * @param[in]      sub_index: 对象字典子索引
  * @param[in]      data: 要写入的数据值
  * @param[in]      len: 数据字节长度 (1, 2, 或 4)
  */
void kinco_sdo_write(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t data, uint8_t len)
{
    uint8_t frame_data[8];
    uint32_t cob_id = 0x600 + node_id;
    
    // 命令修饰符 (CS)
    if (len == 1) {
        frame_data[0] = 0x2F; // 1 字节加速
    } else if (len == 2) {
        frame_data[0] = 0x2B; // 2 字节加速
    } else {
        frame_data[0] = 0x23; // 4 字节加速
    }
    
    // 索引 (小端)
    frame_data[1] = index & 0xFF;
    frame_data[2] = (index >> 8) & 0xFF;
    
    // 子索引
    frame_data[3] = sub_index;
    
    // 数据 (小端)
    frame_data[4] = data & 0xFF;
    frame_data[5] = (data >> 8) & 0xFF;
    frame_data[6] = (data >> 16) & 0xFF;
    frame_data[7] = (data >> 24) & 0xFF;
    
    can_send_frame(hcan, cob_id, frame_data, 8);
}

/**
  * @brief          发送 CANopen SDO 读请求 (发起上传请求)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      node_id: 节点 ID (1-127)
  * @param[in]      index: 对象字典索引
  * @param[in]      sub_index: 对象字典子索引
  */
void kinco_sdo_read(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index)
{
    uint8_t frame_data[8];
    uint32_t cob_id = 0x600 + node_id;
    
    frame_data[0] = 0x40; // 读请求
    
    // 索引 (小端)
    frame_data[1] = index & 0xFF;
    frame_data[2] = (index >> 8) & 0xFF;
    
    // 子索引
    frame_data[3] = sub_index;
    
    // 数据字节填充为 0
    frame_data[4] = 0;
    frame_data[5] = 0;
    frame_data[6] = 0;
    frame_data[7] = 0;
    
    can_send_frame(hcan, cob_id, frame_data, 8);
}

/**
  * @brief          设置电机目标状态为使能 (ENABLED)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  */
void kinco_motor_enable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    motor->is_enabled = 1;
    motor->enable_step = 1; // 修复状态机遇死锁：表示开始使能流程，防止被接收回调错误清零
    // 使用 NMT 唤醒节点
    kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
}

/**
  * @brief          设置电机目标状态为失能 (DISABLED)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  */
void kinco_motor_disable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    motor->is_enabled = 0;
    motor->enable_step = 0; // 复位使能步骤，以便于下一次重新使能
    // 通过 SDO 发送标准的 Disable Voltage 或 Shutdown 命令
    kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_DISABLE_VOLTAGE, 2);
}

/**
  * @brief          请求复位电机故障状态 (非阻塞)
  *                 实际的两步复位流程在 kinco_control_loop() 中通过状态机驱动完成，
  *                 避免使用 HAL_Delay() 阻塞主循环。
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  */
void kinco_motor_reset_fault(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    if (motor->fault_reset_step == 0)
    {
        // 步骤 1: 发送 Fault Reset 控制字 (0x0080)
        kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_FAULT_RESET, 2);
        motor->fault_reset_step = 1;
        motor->fault_reset_time = HAL_GetTick();
    }
    // 步骤 2 (5ms 后发送 Shutdown) 在 kinco_control_loop() 中执行
}

/**
  * @brief          设置电机运行模式
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  * @param[in]      mode: 模式常量，如 KINCO_MODE_PP, KINCO_MODE_PV, KINCO_MODE_PT, 或 KINCO_MODE_HM
  */
void kinco_set_mode(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int8_t mode)
{
    motor->mode_of_operation = mode;
    kinco_sdo_write(hcan, motor->node_id, OBJ_MODES_OF_OPERATION, 0, (uint32_t)mode, 1);
}

/**
  * @brief          设置目标速度 (轮廓速度模式)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  * @param[in]      velocity: 目标速度，单位为驱动器原始单位 (DEC)
  */
void kinco_set_velocity(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t velocity)
{
    motor->target_velocity = velocity;
    
    // 仅在电机处于使能状态且没有故障时发送
    if (motor->is_enabled && !motor->is_fault)
    {
        kinco_sdo_write(hcan, motor->node_id, OBJ_TARGET_VELOCITY, 0, (uint32_t)velocity, 4);
    }
}

/**
  * @brief          设置目标位置并触发运动 (轮廓位置模式)
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  * @param[in]      position: 目标位置，单位为编码器脉冲数 (counts)
  * @param[in]      relative: 0 表示绝对定位，1 表示相对定位
  */
void kinco_set_position(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t position, uint8_t relative)
{
    motor->target_position = position;
    
    if (motor->is_enabled && !motor->is_fault)
    {
        // 1. 将目标位置写入 OBJ_TARGET_POSITION (0x607A)
        // can_send_frame 内部已有邮箱满等待机制，无需额外 HAL_Delay
        kinco_sdo_write(hcan, motor->node_id, OBJ_TARGET_POSITION, 0, (uint32_t)position, 4);
        
        // 2. 准备控制字，将新设定点标志 (Bit 4) 设为 0
        // 绝对定位: 0x2F (Enable Op 0x0F | Change Imm 0x20)
        // 相对定位: 0x4F (Enable Op 0x0F | Change Imm 0x20 | Rel Pos 0x40)
        uint16_t ctrl_prepare = CMD_ENABLE_OP | CTRL_BIT_CHANGE_IMM;
        if (relative)
        {
            ctrl_prepare |= CTRL_BIT_REL_POS;
        }
        kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, ctrl_prepare, 2);
        
        // 3. 通过设置新设定点标志 (Bit 4) = 1 触发上升沿
        // 绝对定位: 0x3F (0x2F | New Setpoint 0x10)
        // 相对定位: 0x5F (0x4F | New Setpoint 0x10)
        uint16_t ctrl_trigger = ctrl_prepare | CTRL_BIT_NEW_POS;
        kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, ctrl_trigger, 2);
    }
}

/**
  * @brief          解析从步科电机接收到的 CAN 报文
  * @param[in/out]  motor: 电机结构体指针
  * @param[in]      std_id: 报文的标准 CAN 标识符
  * @param[in]      data: 指向 8 字节数据缓冲区的指针
  * @param[in]      dlc: 数据长度代码
  */
void kinco_recv_handler(kinco_motor_t *motor, uint32_t std_id, uint8_t *data, uint8_t dlc)
{
    // 1. SDO 响应: COB-ID = 0x580 + NodeID
    if (std_id == (0x580 + motor->node_id))
    {
        uint16_t index = (data[2] << 8) | data[1];
        uint8_t sub_index = data[3];
        
        // SDO 上传/读响应 (CS = 0x4F/0x4B/0x43/0x60)
        // CS = 0x60 表示写成功。CS = 0x43/0x4B/0x4F 表示读成功。
        uint8_t cs = data[0];
        if (cs == 0x4F || cs == 0x4B || cs == 0x43) // 读取成功
        {
            uint32_t value = 0;
            if (cs == 0x4F) value = data[4];
            else if (cs == 0x4B) value = (data[5] << 8) | data[4];
            else value = (data[7] << 24) | (data[6] << 16) | (data[5] << 8) | data[4];
            
            switch (index)
            {
                case OBJ_STATUSWORD:
                    motor->statusword = (uint16_t)value;
                    break;
                case OBJ_MODES_OF_OPERATION_DISP:
                    motor->mode_display = (int8_t)value;
                    break;
                case OBJ_POSITION_ACTUAL:
                    motor->actual_position = (int32_t)value;
                    break;
                case OBJ_VELOCITY_ACTUAL:
                    motor->actual_velocity = (int32_t)value;
                    break;
                default:
                    break;
            }
        }
    }
    // 2. TPDO1 响应: COB-ID = 0x180 + NodeID (状态字)
    else if (std_id == (0x180 + motor->node_id))
    {
        if (dlc >= 2)
        {
            motor->statusword = (data[1] << 8) | data[0];
        }
    }
    // 3. TPDO2 响应: COB-ID = 0x280 + NodeID (状态字 + 实际位置)
    else if (std_id == (0x280 + motor->node_id))
    {
        if (dlc >= 6)
        {
            motor->statusword = (data[1] << 8) | data[0];
            motor->actual_position = (int32_t)((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
        }
    }
    // 4. TPDO3 响应: COB-ID = 0x380 + NodeID (状态字 + 实际速度)
    else if (std_id == (0x380 + motor->node_id))
    {
        if (dlc >= 6)
        {
            motor->statusword = (data[1] << 8) | data[0];
            motor->actual_velocity = (int32_t)((data[5] << 24) | (data[4] << 16) | (data[3] << 8) | data[2]);
        }
    }
    
    // 解析状态字细节
    if (motor->statusword & STATUS_FAULT)
    {
        motor->is_fault = 1;
    }
    else
    {
        motor->is_fault = 0;
    }
    
    if ((motor->statusword & 0x006F) == 0x0027) // 处于 Switched On 并且 Operation Enabled 状态
    {
        motor->is_enabled = 1;
    }
    else
    {
        // 如果我们不在使能过程中，才标记为失能
        if (motor->enable_step == 0)
        {
            motor->is_enabled = 0;
        }
    }
    
    if (motor->statusword & STATUS_TARGET_REACHED)
    {
        motor->target_reached = 1;
    }
    else
    {
        motor->target_reached = 0;
    }
}

/**
  * @brief          管理电机使能状态机和状态轮询。
  *                                 请周期性地调用此函数 (例如每 5-10ms)。
  * @param[in]      hcan: CAN 句柄指针
  * @param[in/out]  motor: 电机结构体指针
  */
void kinco_control_loop(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    uint32_t tick = HAL_GetTick();
    
    // 0. 驱动故障复位非阻塞状态机 (替代原来的 HAL_Delay)
    if (motor->fault_reset_step == 1)
    {
        // 等待 5ms 后发送 Shutdown 命令完成复位
        if (tick - motor->fault_reset_time >= 5)
        {
            kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN, 2);
            motor->fault_reset_step = 2; // 复位完成
        }
        return; // 复位过程中不执行其他逻辑
    }
    else if (motor->fault_reset_step == 2)
    {
        // 复位流程结束，回到空闲
        motor->fault_reset_step = 0;
        motor->is_fault = 0;
    }
    
    // 1. 检查是否需要查询实际值 (每 30ms 轮询一项，以避免 SDO 忙碌/冲突)
    if (tick - motor->last_cmd_time > 30)
    {
        motor->last_cmd_time = tick;
        
        // 利用时间循环切换 3 个轮询步骤
        uint8_t step = (tick / 30) % 3;
        if (step == 0)
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_STATUSWORD, 0);
        }
        else if (step == 1)
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_POSITION_ACTUAL, 0);
        }
        else
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_VELOCITY_ACTUAL, 0);
        }
    }
    
    // 2. 如果用户请求使能状态，则驱动 CiA402 使能状态机
    if (motor->is_enabled && !motor->is_fault)
    {
        // 如果状态字还未读取到或是 0，则等待
        if (motor->statusword == 0)
        {
            return;
        }
        
        // 检查标准的 DS402 状态:
        // Not Ready to Switch On / Switch On Disabled (状态字 bit 6 为 1)
        if (motor->statusword & STATUS_SWITCH_ON_DISABLED)
        {
            if (motor->enable_step != 1)
            {
                // 状态转换 2: Shutdown -> Ready to Switch On
                kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN, 2);
                motor->enable_step = 1;
                
                // 驱动器如果是刚重新上电，会处于此状态。我们需要重新唤醒节点并恢复运行模式
                kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
            }
        }
        // Ready to Switch On (bit 0=1, bit 1=0, bit 2=0, bit 6=0)
        else if ((motor->statusword & 0x006F) == 0x0021)
        {
            if (motor->enable_step != 2)
            {
                // 状态转换 3: Switch On -> Switched On
                kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SWITCH_ON, 2);
                motor->enable_step = 2;
            }
        }
        // Switched On (bit 0=1, bit 1=1, bit 2=0, bit 6=0)
        else if ((motor->statusword & 0x006F) == 0x0023)
        {
            if (motor->enable_step != 3)
            {
                // 状态转换 4: Enable Operation -> Operation Enabled
                kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_ENABLE_OP, 2);
                motor->enable_step = 3;
            }
        }
        // Operation Enabled (bit 0=1, bit 1=1, bit 2=1, bit 6=0)
        else if ((motor->statusword & 0x006F) == 0x0027)
        {
            if (motor->enable_step != 4)
            {
                motor->enable_step = 4; // 电机成功使能!
                
                // 成功使能后再设置模式，避开前面的 SDO 写入拥堵，提高配置成功率
                kinco_set_mode(hcan, motor, motor->mode_of_operation);
            }
        }
    }
}

/**
  * @brief          将目标速度从 RPM 转换为驱动器原始 DEC 单位
  *                                 公式来自用户手册第108页: DEC = (RPM * 512 * 编码器分辨率) / 1875
  * @param[in]      rpm: 速度，单位为 RPM
  * @param[in]      encoder_resolution: 编码器每圈脉冲数 (如 10000)
  * @retval         速度，单位为驱动器原始单位 (DEC)
  */
int32_t kinco_rpm_to_dec(int32_t rpm, int32_t encoder_resolution)
{
    // 内部使用 64 位整数以防止乘法溢出
    int64_t temp = (int64_t)rpm * 512 * encoder_resolution;
    return (int32_t)(temp / 1875);
}

/**
  * @brief          将速度从驱动器原始 DEC 单位转换为 RPM
  *                                 公式来自用户手册第108页: RPM = (DEC * 1875) / (512 * 编码器分辨率)
  * @param[in]      dec: 速度，单位为驱动器原始单位 (DEC)
  * @param[in]      encoder_resolution: 编码器每圈脉冲数 (如 10000)
  * @retval         速度，单位为 RPM
  */
int32_t kinco_dec_to_rpm(int32_t dec, int32_t encoder_resolution)
{
    int64_t temp = (int64_t)dec * 1875;
    return (int32_t)(temp / (512 * encoder_resolution));
}

/**
  * @brief          启动找原点过程 (Homing Mode)
  *                                 步骤: 设置方式，设置速度，切换到模式6，然后控制字触发 0x0F -> 0x1F 上升沿
  * @param[in]      hcan: CAN 句柄指针
  * @param[in]      motor: 电机结构体指针
  * @param[in]      method: 找原点方式索引 (例如 17 或 -1)，参考手册第79页
  * @param[in]      fast_speed: 寻找开关的快速目标速度 (DEC 单位)
  * @param[in]      slow_speed: 寻找零点的慢速对齐速度 (DEC 单位)
  */
void kinco_start_homing(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int8_t method, uint32_t fast_speed, uint32_t slow_speed)
{
    if (motor->is_enabled && !motor->is_fault)
    {
        // 1. 设置找原点方式 (0x6098)
        // can_send_frame 内部已有邮箱满等待机制，无需额外 HAL_Delay
        kinco_sdo_write(hcan, motor->node_id, 0x6098, 0, (uint32_t)method, 1);
        
        // 2. 设置找原点速度 (0x6099:1 和 0x6099:2)
        kinco_sdo_write(hcan, motor->node_id, 0x6099, 1, fast_speed, 4);
        kinco_sdo_write(hcan, motor->node_id, 0x6099, 2, slow_speed, 4);
        
        // 3. 切换至找原点模式 (Mode 6)
        kinco_set_mode(hcan, motor, KINCO_MODE_HM);
        
        // 4. 写入 0x001F 触发找原点开始 (基于原有的 0x000F 在 Bit 4 上制造上升沿)
        kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, 0x001F, 2);
    }
}


