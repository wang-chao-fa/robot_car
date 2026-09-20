/**
  ******************************************************************************
  * @file       kinco_canopen.c
  * @brief      步科伺服 CANopen / CiA 402 协议驱动与状态机实现
  ******************************************************************************
  */
#include "kinco_canopen.h"
#include <string.h>
#include <stdlib.h>

static uint8_t can_send_frame(CAN_HandleTypeDef *hcan, uint32_t cob_id, const uint8_t *data, uint8_t dlc)
{
    if (hcan == NULL || data == NULL) return 1;

    uint32_t timeout = 2000;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0 && timeout--)
    {
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0)
    {
        return 1;
    }

    CAN_TxHeaderTypeDef tx_header;
    uint32_t send_mail_box;

    tx_header.StdId = cob_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;

    if (HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &send_mail_box) != HAL_OK)
    {
        return 1;
    }

    return 0;
}

void kinco_motor_init(kinco_motor_t *motor, uint8_t node_id)
{
    if (motor == NULL) return;
    memset(motor, 0, sizeof(kinco_motor_t));
    motor->node_id = node_id;
    motor->mode_of_operation = KINCO_MODE_PP;
    motor->target_position = 0x7FFFFFFF;
    motor->last_sent_position = 0x7FFFFFFF;
}

int32_t kinco_rpm_to_dec(int32_t rpm, uint32_t encoder_res)
{
    int64_t dec = ((int64_t)rpm * (int64_t)encoder_res * 512LL) / 1875LL;
    return (int32_t)dec;
}

int32_t kinco_dec_to_rpm(int32_t dec, uint32_t encoder_res)
{
    if (encoder_res == 0) return 0;
    int64_t rpm = ((int64_t)dec * 1875LL) / ((int64_t)encoder_res * 512LL);
    return (int32_t)rpm;
}

void kinco_nmt_cmd(CAN_HandleTypeDef *hcan, uint8_t node_id, uint8_t cs)
{
    uint8_t data[2];
    data[0] = cs;
    data[1] = node_id;
    can_send_frame(hcan, 0x000, data, 2);
}

void kinco_sdo_write(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index, uint32_t data, uint8_t len)
{
    uint8_t frame_data[8] = {0};
    uint32_t cob_id = 0x600 + node_id;

    switch (len)
    {
        case 1: frame_data[0] = 0x2F; break;
        case 2: frame_data[0] = 0x2B; break;
        case 4: frame_data[0] = 0x23; break;
        default: frame_data[0] = 0x23; break;
    }

    frame_data[1] = (uint8_t)(index & 0xFF);
    frame_data[2] = (uint8_t)((index >> 8) & 0xFF);
    frame_data[3] = sub_index;

    frame_data[4] = (uint8_t)(data & 0xFF);
    frame_data[5] = (uint8_t)((data >> 8) & 0xFF);
    frame_data[6] = (uint8_t)((data >> 16) & 0xFF);
    frame_data[7] = (uint8_t)((data >> 24) & 0xFF);

    can_send_frame(hcan, cob_id, frame_data, 8);
}

void kinco_sdo_read(CAN_HandleTypeDef *hcan, uint8_t node_id, uint16_t index, uint8_t sub_index)
{
    uint8_t frame_data[8] = {0};
    uint32_t cob_id = 0x600 + node_id;

    frame_data[0] = 0x40;
    frame_data[1] = (uint8_t)(index & 0xFF);
    frame_data[2] = (uint8_t)((index >> 8) & 0xFF);
    frame_data[3] = sub_index;

    can_send_frame(hcan, cob_id, frame_data, 8);
}

void kinco_motor_enable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    if (motor == NULL || hcan == NULL) return;
    motor->enable_requested = 1;
    motor->enable_step = 0;
    kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
    HAL_Delay(5);
    kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN, 2);
}

void kinco_motor_disable(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    if (motor == NULL || hcan == NULL) return;
    motor->enable_requested = 0;
    motor->is_enabled = 0;
    motor->enable_step = 0;
    kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_DISABLE_VOLTAGE, 2);
}

void kinco_motor_reset_fault(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    if (motor == NULL || hcan == NULL) return;
    motor->fault_reset_step = 1;
    motor->fault_reset_time = HAL_GetTick();
    kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_FAULT_RESET, 2);
}

void kinco_set_mode(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int8_t mode)
{
    if (motor == NULL || hcan == NULL) return;
    motor->mode_of_operation = mode;
    kinco_sdo_write(hcan, motor->node_id, OBJ_MODES_OF_OPERATION, 0, (uint32_t)mode, 1);
}

void kinco_set_velocity(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t rpm)
{
    if (motor == NULL || hcan == NULL) return;
    motor->target_velocity = rpm;
    int32_t dec = kinco_rpm_to_dec(rpm, ENCODER_RESOLUTION);
    kinco_sdo_write(hcan, motor->node_id, OBJ_TARGET_VELOCITY, 0, (uint32_t)dec, 4);
}

void kinco_set_torque(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int16_t torque_permille)
{
    if (motor == NULL || hcan == NULL) return;
    kinco_sdo_write(hcan, motor->node_id, OBJ_TARGET_TORQUE, 0, (uint32_t)(uint16_t)torque_permille, 2);
}

void kinco_set_position(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, int32_t pos_counts, uint8_t is_relative)
{
    if (motor == NULL || hcan == NULL) return;

    motor->target_position = pos_counts;

    // 1. 设置目标位置 OBJ_TARGET_POSITION (0x607A)
    kinco_sdo_write(hcan, motor->node_id, OBJ_TARGET_POSITION, 0, (uint32_t)pos_counts, 4);

    // 2. 准备控制字 -> 触发新位置 (Bit 4=1, Bit 5=1 即 0x003F)
    uint16_t ctrl_prepare = CMD_ENABLE_OP | CTRL_BIT_CHANGE_IMM;
    if (is_relative)
    {
        ctrl_prepare |= CTRL_BIT_REL_POS;
    }
    kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, ctrl_prepare, 2);

    uint16_t ctrl_trigger = ctrl_prepare | CTRL_BIT_NEW_POS;
    kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, ctrl_trigger, 2);

    motor->controlword = ctrl_trigger;
    motor->last_sent_position = pos_counts;
    motor->position_cmd_sent = 1;
}

void kinco_set_profile_velocity_custom(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, uint32_t speed_rpm, uint32_t acc_rpm_s, uint32_t dec_rpm_s)
{
    if (motor == NULL || hcan == NULL) return;

    uint32_t dec_speed = (uint32_t)kinco_rpm_to_dec((int32_t)speed_rpm, ENCODER_RESOLUTION);
    uint32_t dec_acc   = (uint32_t)kinco_rpm_to_dec((int32_t)acc_rpm_s, ENCODER_RESOLUTION);
    uint32_t dec_dec   = (uint32_t)kinco_rpm_to_dec((int32_t)dec_rpm_s, ENCODER_RESOLUTION);

    kinco_sdo_write(hcan, motor->node_id, OBJ_PROFILE_VELOCITY, 0, dec_speed, 4);
    kinco_sdo_write(hcan, motor->node_id, OBJ_PROFILE_ACC, 0, dec_acc, 4);
    kinco_sdo_write(hcan, motor->node_id, OBJ_PROFILE_DEC, 0, dec_dec, 4);
}

void kinco_motor_config_sync(CAN_HandleTypeDef *hcan, kinco_motor_t *motor, uint32_t speed_rpm, uint32_t acc_rpm_s, uint32_t dec_rpm_s)
{
    if (motor == NULL || hcan == NULL) return;

    motor->profile_speed_rpm = speed_rpm;
    motor->profile_acc_rpm_s = acc_rpm_s;
    motor->profile_dec_rpm_s = dec_rpm_s;
    motor->mode_of_operation = KINCO_MODE_PP;

    uint32_t dec_speed = (uint32_t)kinco_rpm_to_dec((int32_t)speed_rpm, ENCODER_RESOLUTION);
    uint32_t dec_acc   = (uint32_t)kinco_rpm_to_dec((int32_t)acc_rpm_s, ENCODER_RESOLUTION);
    uint32_t dec_dec   = (uint32_t)kinco_rpm_to_dec((int32_t)dec_rpm_s, ENCODER_RESOLUTION);

    kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
    HAL_Delay(10);

    kinco_sdo_write(hcan, motor->node_id, OBJ_MODES_OF_OPERATION, 0, motor->mode_of_operation, 1);
    HAL_Delay(10);

    kinco_sdo_write(hcan, motor->node_id, OBJ_PROFILE_VELOCITY, 0, dec_speed, 4);
    HAL_Delay(10);
    kinco_sdo_write(hcan, motor->node_id, OBJ_PROFILE_ACC, 0, dec_acc, 4);
    HAL_Delay(10);
    kinco_sdo_write(hcan, motor->node_id, OBJ_PROFILE_DEC, 0, dec_dec, 4);
    HAL_Delay(10);

    uint32_t pos_window = (motor->node_id == 6) ? 100 : 5000;
    kinco_sdo_write(hcan, motor->node_id, OBJ_POSITION_WINDOW, 0, pos_window, 4);
    HAL_Delay(10);

    if (motor->node_id == 2 || motor->node_id == 3 || motor->node_id == 4 || motor->node_id == 5)
    {
        kinco_sdo_write(hcan, motor->node_id, OBJ_MAX_TORQUE, 0, 3000, 2);
        HAL_Delay(10);
    }
}

void kinco_recv_handler(kinco_motor_t *motor, uint32_t std_id, uint8_t *data, uint8_t dlc)
{
    if (motor == NULL || data == NULL) return;

    if (std_id == (0x580 + motor->node_id))
    {
        uint16_t index = (data[2] << 8) | data[1];
        uint8_t cs = data[0];

        if (cs == 0x4F || cs == 0x4B || cs == 0x43)
        {
            uint32_t value = 0;
            if (cs == 0x4F) value = (uint32_t)data[4];
            else if (cs == 0x4B) value = ((uint32_t)data[5] << 8) | (uint32_t)data[4];
            else value = ((uint32_t)data[7] << 24) | ((uint32_t)data[6] << 16) | ((uint32_t)data[5] << 8) | (uint32_t)data[4];

            switch (index)
            {
                case OBJ_STATUSWORD:
                    motor->statusword = (uint16_t)value;
                    motor->is_fault = (motor->statusword & STATUS_FAULT) ? 1 : 0;
                    if ((motor->statusword & 0x006F) == 0x0027) motor->is_enabled = 1;
                    break;
                case OBJ_ERROR_CODE:
                    motor->error_code = (uint16_t)value;
                    break;
                case OBJ_ERROR_REGISTER:
                    motor->error_reg = (uint8_t)value;
                    break;
                case OBJ_MODES_OF_OPERATION_DISP:
                    motor->mode_display = (int8_t)value;
                    break;
                case OBJ_POSITION_ACTUAL:
                    motor->actual_position = (int32_t)value;
                    if (!motor->home_captured)
                    {
                        motor->home_position = (int32_t)value;
                        motor->target_position = motor->home_position;
                        motor->home_captured = 1;
                    }
                    break;
                case OBJ_VELOCITY_ACTUAL:
                    motor->actual_velocity = (int32_t)value;
                    break;
                default:
                    break;
            }
        }
    }
    else if (std_id == (0x180 + motor->node_id))
    {
        if (dlc >= 2)
        {
            motor->statusword = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
            motor->is_fault = (motor->statusword & STATUS_FAULT) ? 1 : 0;
            if ((motor->statusword & 0x006F) == 0x0027) motor->is_enabled = 1;
        }
    }
    else if (std_id == (0x280 + motor->node_id))
    {
        if (dlc >= 6)
        {
            motor->statusword = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
            motor->actual_position = (int32_t)(((uint32_t)data[5] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[2]);
            motor->is_fault = (motor->statusword & STATUS_FAULT) ? 1 : 0;
            if ((motor->statusword & 0x006F) == 0x0027) motor->is_enabled = 1;

            if (!motor->home_captured)
            {
                motor->home_position = motor->actual_position;
                motor->target_position = motor->home_position;
                motor->home_captured = 1;
            }
        }
    }
    else if (std_id == (0x380 + motor->node_id))
    {
        if (dlc >= 6)
        {
            motor->statusword = ((uint16_t)data[1] << 8) | (uint16_t)data[0];
            motor->actual_velocity = (int32_t)(((uint32_t)data[5] << 24) | ((uint32_t)data[4] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[2]);
            motor->is_fault = (motor->statusword & STATUS_FAULT) ? 1 : 0;
            if ((motor->statusword & 0x006F) == 0x0027) motor->is_enabled = 1;
        }
    }
}

void kinco_control_loop(CAN_HandleTypeDef *hcan, kinco_motor_t *motor)
{
    if (motor == NULL || hcan == NULL) return;
    uint32_t tick = HAL_GetTick();

    if (motor->fault_reset_step == 1)
    {
        if (tick - motor->fault_reset_time >= 20)
        {
            kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN, 2);
            motor->fault_reset_step = 2;
        }
        return;
    }
    else if (motor->fault_reset_step == 2)
    {
        motor->fault_reset_step = 0;
        motor->enable_step = 0;
        motor->is_fault = 0;
        motor->last_state_cmd_time = tick;
        kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
        kinco_sdo_read(hcan, motor->node_id, OBJ_STATUSWORD, 0);
    }

    /* 错峰 SDO 轮询: 避免 5 台电机在同一个毫秒集中突发 CAN 请求 */
    uint32_t offset = ((uint32_t)motor->node_id * 20) % 100;
    if ((tick - motor->last_cmd_time >= 100) || ((tick % 100 == offset) && (tick - motor->last_cmd_time >= 80)))
    {
        motor->last_cmd_time = tick;
        uint8_t step = ((tick / 100) + (uint32_t)motor->node_id * 3) % 4;
        if (step == 0)
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_STATUSWORD, 0);
        }
        else if (step == 1)
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_POSITION_ACTUAL, 0);
        }
        else if (step == 2)
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_VELOCITY_ACTUAL, 0);
        }
        else
        {
            kinco_sdo_read(hcan, motor->node_id, OBJ_ERROR_CODE, 0);
        }
    }

    if (motor->statusword & STATUS_FAULT)
    {
        motor->is_fault = 1;
        motor->is_enabled = 0;
        if (motor->fault_reset_step == 0 && (tick - motor->last_fault_attempt_time >= 800))
        {
            motor->last_fault_attempt_time = tick;
            kinco_sdo_read(hcan, motor->node_id, OBJ_ERROR_CODE, 0);
            kinco_motor_reset_fault(hcan, motor);
            return;
        }
    }

    if (motor->enable_requested && !motor->is_fault)
    {
        if (motor->statusword == 0)
        {
            if (tick - motor->last_state_cmd_time >= 200)
            {
                motor->last_state_cmd_time = tick;
                kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
                kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN, 2);
                kinco_sdo_read(hcan, motor->node_id, OBJ_STATUSWORD, 0);
            }
            return;
        }

        if (motor->statusword & STATUS_SWITCH_ON_DISABLED)
        {
            motor->is_enabled = 0;
            if (motor->enable_step != 1 || (tick - motor->last_state_cmd_time >= 50))
            {
                motor->last_state_cmd_time = tick;
                motor->enable_step = 1;
                kinco_nmt_cmd(hcan, motor->node_id, NMT_CS_START_NODE);
                kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_SHUTDOWN, 2);
            }
        }
        else if (((motor->statusword & 0x006F) == 0x0021) || ((motor->statusword & 0x006F) == 0x0023))
        {
            motor->is_enabled = 0;
            if (motor->enable_step != 3 || (tick - motor->last_state_cmd_time >= 50))
            {
                motor->last_state_cmd_time = tick;
                motor->enable_step = 3;
                kinco_sdo_write(hcan, motor->node_id, OBJ_CONTROLWORD, 0, CMD_ENABLE_OP, 2);
            }
        }
        else if ((motor->statusword & 0x006F) == 0x0027)
        {
            motor->enable_step = 4;
            motor->is_enabled = 1;
        }
    }
}
