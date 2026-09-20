/**
  ******************************************************************************
  * @file       auto_serial_ctrl.c
  * @brief      上位机串口自动驾驶通信协议实现
  ******************************************************************************
  */

#include "auto_serial_ctrl.h"
#include "analog_input.h"
#include "mdu_steering_motor.h"
#include "rc_sbus.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* 全局自动驾驶指令结构体实例 */
serial_auto_cmd_t g_serial_auto_cmd;

/* 串口单字节接收环形/流式缓冲区与状态机 */
static char s_rx_buf[SERIAL_AUTO_RX_BUF_SIZE];
static uint16_t s_rx_idx = 0;
static uint8_t s_frame_started = 0;

/**
  * @brief  计算一串字符数据的 8-bit XOR 异或校验和
  */
static uint8_t Calc_XOR_Checksum(const char *data, uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        cs ^= (uint8_t)data[i];
    }
    return cs;
}

/**
  * @brief  十六进制字符转 4-bit 数值
  */
static int8_t HexCharToNibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/**
  * @brief  两个十六进制字符解析为 1 个单字节数值
  */
static int16_t HexToByte(const char *hex_str)
{
    int8_t hi = HexCharToNibble(hex_str[0]);
    int8_t lo = HexCharToNibble(hex_str[1]);
    if (hi < 0 || lo < 0) return -1;
    return (int16_t)((hi << 4) | lo);
}

/**
  * @brief  处理单帧完整的 $AUTO 控制指令
  */
static void Serial_Auto_ProcessFrame(char *frame, uint16_t len)
{
    if (len < 10) return;

    /* 寻找校验和分隔符 '*' */
    char *star_ptr = strchr(frame, '*');
    if (star_ptr == NULL) return;

    uint16_t data_len = (uint16_t)(star_ptr - frame);
    if ((len - data_len) < 3) return; // 后面必须至少有 2 字节十六进制校验码

    /* 计算数据部分异或校验和 */
    uint8_t calc_cs = Calc_XOR_Checksum(frame, data_len);
    int16_t recv_cs = HexToByte(star_ptr + 1);
    if (recv_cs < 0 || (uint8_t)recv_cs != calc_cs)
    {
        g_serial_auto_cmd.error_frame_count++;
        return;
    }

    /* 提取数据字段 */
    char data_buf[SERIAL_AUTO_RX_BUF_SIZE];
    if (data_len >= SERIAL_AUTO_RX_BUF_SIZE) return;
    memcpy(data_buf, frame, data_len);
    data_buf[data_len] = '\0';

    int gear_tmp = 0, clutch_tmp = 0, throttle_tmp = 0, brake_tmp = 0, actuator_tmp = 0, hb_tmp = 0;
    float steer_tmp = 0.0f;

    int parsed = sscanf(data_buf, "AUTO,%d,%d,%d,%d,%d,%f,%d",
                        &gear_tmp, &clutch_tmp, &throttle_tmp, &brake_tmp,
                        &actuator_tmp, &steer_tmp, &hb_tmp);

    if (parsed != 7)
    {
        g_serial_auto_cmd.error_frame_count++;
        return;
    }

    /* 限制控制量在合法安全范围内 */
    if (gear_tmp < 0 || gear_tmp > 2) gear_tmp = 0;
    if (clutch_tmp != 0 && clutch_tmp != 1) clutch_tmp = 0;
    if (throttle_tmp < 0) throttle_tmp = 0;
    if (throttle_tmp > 100) throttle_tmp = 100;
    if (brake_tmp != 0 && brake_tmp != 1) brake_tmp = 0;
    if (actuator_tmp < -1) actuator_tmp = -1;
    if (actuator_tmp > 1) actuator_tmp = 1;
    if (steer_tmp > 100.0f) steer_tmp = 100.0f;
    if (steer_tmp < -100.0f) steer_tmp = -100.0f;
    hb_tmp &= 0xFF;

    /* 心跳防冻结检测 */
    uint8_t new_hb = (uint8_t)hb_tmp;

    if (g_serial_auto_cmd.is_active)
    {
        if (new_hb == g_serial_auto_cmd.heartbeat)
        {
            if (g_serial_auto_cmd.hb_freeze_counter < 255)
            {
                g_serial_auto_cmd.hb_freeze_counter++;
            }
        }
        else
        {
            g_serial_auto_cmd.hb_freeze_counter = 0;
        }
    }
    else
    {
        g_serial_auto_cmd.hb_freeze_counter = 0;
    }

    /* 更新全局自动驾驶控制量 */
    g_serial_auto_cmd.gear             = (uint8_t)gear_tmp;
    g_serial_auto_cmd.clutch           = (uint8_t)clutch_tmp;
    g_serial_auto_cmd.throttle_percent = (uint8_t)throttle_tmp;
    g_serial_auto_cmd.brake            = (uint8_t)brake_tmp;
    g_serial_auto_cmd.actuator_dir     = (int8_t)actuator_tmp;
    g_serial_auto_cmd.steer_speed_rpm  = steer_tmp;
    g_serial_auto_cmd.last_heartbeat   = g_serial_auto_cmd.heartbeat;
    g_serial_auto_cmd.heartbeat        = new_hb;
    g_serial_auto_cmd.last_valid_time  = HAL_GetTick();
    g_serial_auto_cmd.is_active        = 1;
    g_serial_auto_cmd.valid_frame_count++;
}

/**
  * @brief  初始化上位机自动驾驶通信模块
  */
void Serial_Auto_Init(void)
{
    memset(&g_serial_auto_cmd, 0, sizeof(g_serial_auto_cmd));
    s_rx_idx = 0;
    s_frame_started = 0;
}

/**
  * @brief  单字节串口解析 (在 USART6 RXNE 中断或主循环中调用)
  */
void Serial_Auto_ParseByte(uint8_t ch)
{
    if (ch == '$')
    {
        s_rx_idx = 0;
        s_frame_started = 1;
        return;
    }

    if (!s_frame_started)
    {
        return;
    }

    if (ch == '\n' || ch == '\r')
    {
        if (s_rx_idx > 0)
        {
            s_rx_buf[s_rx_idx] = '\0';
            Serial_Auto_ProcessFrame(s_rx_buf, s_rx_idx);
        }
        s_rx_idx = 0;
        s_frame_started = 0;
        return;
    }

    if (s_rx_idx < SERIAL_AUTO_RX_BUF_SIZE - 1)
    {
        s_rx_buf[s_rx_idx++] = (char)ch;
    }
    else
    {
        s_rx_idx = 0;
        s_frame_started = 0;
    }
}

/**
  * @brief  判断上位机自动驾驶通信链路是否健康有效
  */
uint8_t Serial_Auto_IsAlive(void)
{
    if (!g_serial_auto_cmd.is_active)
    {
        return 0;
    }

    uint32_t now = HAL_GetTick();

    /* 超时保护 */
    if ((now - g_serial_auto_cmd.last_valid_time) > SERIAL_AUTO_TIMEOUT_MS)
    {
        return 0;
    }

    /* 心跳冻结保护 */
    if (g_serial_auto_cmd.hb_freeze_counter >= SERIAL_AUTO_HB_FREEZE_COUNT)
    {
        return 0;
    }

    return 1;
}

/**
  * @brief  获取当前系统的故障掩码
  */
uint8_t Serial_Auto_GetErrMask(void)
{
    uint8_t mask = 0;
    uint32_t now = HAL_GetTick();

    /* bit0: SBUS 遥控信号丢失 */
    if (!sbus_updated || (now - sbus_last_time) > SBUS_FAILSAFE_TIMEOUT_MS)
    {
        mask |= ERR_MASK_SBUS_LOST;
    }

    /* bit1: 电机故障 */
    if (g_mdu_steering_motor.error_flag)
    {
        mask |= ERR_MASK_MOTOR_FAULT;
    }

    /* bit2: 驾驶员人工抢夺方向盘 */
    if (g_mdu_steering_motor.hand_override_flag)
    {
        mask |= ERR_MASK_STEER_OVERRIDE;
    }

    /* bit3: 上位机串口通信异常 */
    if (g_serial_auto_cmd.is_active)
    {
        if ((now - g_serial_auto_cmd.last_valid_time) > SERIAL_AUTO_TIMEOUT_MS ||
            g_serial_auto_cmd.hb_freeze_counter >= SERIAL_AUTO_HB_FREEZE_COUNT)
        {
            mask |= ERR_MASK_SERIAL_HB_ERR;
        }
    }

    return mask;
}

/**
  * @brief  通过 USART6 向上位机发送 $STATE 状态回传帧
  */
void Serial_Auto_SendFeedback(UART_HandleTypeDef *huart, uint8_t mode)
{
    if (huart == NULL) return;

    char payload[128];
    char tx_frame[140];

    float wheel_angle = Analog_Input_GetChannel1_Angle0To90();
    float steer_angle = g_mdu_steering_motor.actual_angle_deg;
    float steer_rpm   = g_mdu_steering_motor.actual_speed_rpm;
    uint8_t err_mask  = Serial_Auto_GetErrMask();
    uint8_t hb_echo   = g_serial_auto_cmd.heartbeat;

    int payload_len = snprintf(payload, sizeof(payload),
        "STATE,%.1f,%.1f,%.1f,%d,%d,%d",
        wheel_angle, steer_angle, steer_rpm,
        (int)mode, (int)err_mask, (int)hb_echo);

    if (payload_len <= 0 || payload_len >= (int)sizeof(payload)) return;

    uint8_t cs = Calc_XOR_Checksum(payload, (uint16_t)payload_len);

    int frame_len = snprintf(tx_frame, sizeof(tx_frame),
        "$%s*%02X\r\n", payload, cs);

    if (frame_len <= 0 || frame_len >= (int)sizeof(tx_frame)) return;

    HAL_UART_Transmit(huart, (uint8_t *)tx_frame, (uint16_t)frame_len, 5);
}
