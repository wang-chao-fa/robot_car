/**
  ******************************************************************************
  * @file       mdu_steering_motor.c
  * @brief      科亚 173 / MDU-M 智能方向盘舵机 CAN 驱动实现
  ******************************************************************************
  */
#include "mdu_steering_motor.h"
#include <string.h>

mdu_steering_motor_t g_mdu_steering_motor = {0};

static uint8_t mdu_can_send(CAN_HandleTypeDef *hcan, uint32_t ext_id, const uint8_t *data, uint8_t len)
{
    if (hcan == NULL || data == NULL) return 1;
    uint32_t timeout = 2000;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && timeout--)
    {
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) return 1;

    CAN_TxHeaderTypeDef tx_header;
    uint32_t send_mail_box;

    tx_header.ExtId = ext_id;
    tx_header.IDE = CAN_ID_EXT;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &send_mail_box) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

void MDU_Motor_Init(CAN_HandleTypeDef *hcan)
{
    memset(&g_mdu_steering_motor, 0, sizeof(g_mdu_steering_motor));
}

void MDU_Motor_Enable(CAN_HandleTypeDef *hcan)
{
    /* 1. 切换到绝对位置模式: 03 0D 20 31 00 00 00 00 */
    uint8_t mode_cmd[8] = {0x03, 0x0D, 0x20, 0x31, 0x00, 0x00, 0x00, 0x00};
    mdu_can_send(hcan, 0x06000001, mode_cmd, 8);
    mdu_can_send(hcan, 0x06000007, mode_cmd, 8);
    HAL_Delay(5);

    /* 2. 发送使能指令: 23 0D 20 01 00 00 00 00 */
    uint8_t enable_cmd[8] = {0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
    mdu_can_send(hcan, 0x06000001, enable_cmd, 8);
    mdu_can_send(hcan, 0x06000007, enable_cmd, 8);

    g_mdu_steering_motor.is_enabled = 1;
}

void MDU_Motor_Disable(CAN_HandleTypeDef *hcan)
{
    /* 发送失能指令: 23 0C 20 01 00 00 00 00 */
    uint8_t disable_cmd[8] = {0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00};
    mdu_can_send(hcan, 0x06000001, disable_cmd, 8);
    mdu_can_send(hcan, 0x06000007, disable_cmd, 8);

    g_mdu_steering_motor.is_enabled = 0;
}

void MDU_Motor_SetSpeed(CAN_HandleTypeDef *hcan, float speed_rpm)
{
    uint8_t tx_data[8] = {0};
    int32_t speed_raw = (int32_t)(speed_rpm * 10.0f); // ±1000 对应 ±100rpm

    tx_data[0] = 0x23;
    tx_data[1] = 0x00;
    tx_data[2] = 0x20;
    tx_data[3] = 0x01;
    tx_data[4] = (uint8_t)((speed_raw >> 8) & 0xFF);
    tx_data[5] = (uint8_t)(speed_raw & 0xFF);
    tx_data[6] = (uint8_t)((speed_raw >> 24) & 0xFF);
    tx_data[7] = (uint8_t)((speed_raw >> 16) & 0xFF);

    mdu_can_send(hcan, 0x06000001, tx_data, 8);
    mdu_can_send(hcan, 0x06000007, tx_data, 8);
}

void MDU_Motor_SetAngle(CAN_HandleTypeDef *hcan, float target_angle_deg)
{
    g_mdu_steering_motor.target_angle_deg = target_angle_deg;

    /* 角度换算: 360° 对应 10000 计数值 (10000 / 360 = 250 / 9) */
    int32_t angle_raw = (int32_t)(target_angle_deg * 10000.0f / 360.0f);

    uint8_t tx_data[8] = {0};
    tx_data[0] = 0x23;
    tx_data[1] = 0x02;
    tx_data[2] = 0x20;
    tx_data[3] = 0x01;
    /* 字节序: DATA_L(H), DATA_L(L), DATA_H(H), DATA_H(L) */
    tx_data[4] = (uint8_t)((angle_raw >> 8) & 0xFF);
    tx_data[5] = (uint8_t)(angle_raw & 0xFF);
    tx_data[6] = (uint8_t)((angle_raw >> 24) & 0xFF);
    tx_data[7] = (uint8_t)((angle_raw >> 16) & 0xFF);

    mdu_can_send(hcan, 0x06000001, tx_data, 8);
    mdu_can_send(hcan, 0x06000007, tx_data, 8);
}

void MDU_Motor_ProcessCANMessage(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (data == NULL || len < 6) return;
    g_mdu_steering_motor.last_rx_tick = HAL_GetTick();

    /* 1. 科亚 173 舵机扩展帧心跳报文 (0x07000001 / 0x07000007) */
    if (can_id == 0x07000001 || can_id == 0x07000007)
    {
        /* Data0(H), Data1(L): 角度累计值 (360°/圈) */
        int16_t angle_raw = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
        /* Data2(H), Data3(L): 电机转速 (带符号 RPM) */
        int16_t speed_raw = (int16_t)(((uint16_t)data[2] << 8) | data[3]);

        g_mdu_steering_motor.actual_angle_deg = (float)angle_raw;
        g_mdu_steering_motor.actual_speed_rpm = (float)speed_raw;

        if (!g_mdu_steering_motor.home_captured)
        {
            g_mdu_steering_motor.home_angle_deg = g_mdu_steering_motor.actual_angle_deg;
            g_mdu_steering_motor.home_captured = 1;
        }

        if (len >= 8)
        {
            uint16_t err_code = ((uint16_t)data[6] << 8) | data[7];
            g_mdu_steering_motor.is_enabled = ((data[7] & 0x01) == 0) ? 1 : 0; // Bit0: 0=使能, 1=失能
            g_mdu_steering_motor.error_flag = (err_code > 0x0001) ? 1 : 0;
            g_mdu_steering_motor.hand_override_flag = (data[6] & 0x04) ? 1 : 0;
        }
    }
    /* 2. 科亚 173 SDO 查询响应报文 (0x05800001 / 0x05800007) */
    else if (can_id == 0x05800001 || can_id == 0x05800007)
    {
        /* SDO 响应确认 */
    }
    /* 3. 兼容标准帧或 J1939 报文 */
    else if (can_id == 0x241 || can_id == 0x011 || can_id == 0x017 || can_id == 0x0C0402A1)
    {
        int32_t angle_raw = (int32_t)(data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24));
        int16_t speed_raw = (int16_t)(data[4] | (data[5] << 8));

        g_mdu_steering_motor.actual_angle_deg = (float)angle_raw * 0.01f;
        g_mdu_steering_motor.actual_speed_rpm = (float)speed_raw * 0.1f;
    }
}
