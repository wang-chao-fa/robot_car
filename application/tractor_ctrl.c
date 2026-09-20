/**
  ******************************************************************************
  * @file       tractor_ctrl.c
  * @brief      拖拉机改装核心控制实现
  ******************************************************************************
  */
#include "tractor_ctrl.h"
#include "analog_input.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

kinco_motor_t g_motor_gearshift2; // ID 2
kinco_motor_t g_motor_brake;      // ID 3
kinco_motor_t g_motor_gearshift;  // ID 4
kinco_motor_t g_motor_clutch;     // ID 5
kinco_motor_t g_motor_throttle;   // ID 6

switch_3pos_t TractorControl_Parse3Pos(int16_t mapped_val)
{
    if (mapped_val > 300)        return SWITCH_POS_UP;
    else if (mapped_val < -300)  return SWITCH_POS_DOWN;
    else                         return SWITCH_POS_MID;
}

switch_2pos_t TractorControl_Parse2Pos(int16_t mapped_val)
{
    if (mapped_val > 500) return SWITCH_POS_ON;
    else                  return SWITCH_POS_OFF;
}

void TractorControl_GetSBUSDemand(tractor_demand_t *demand)
{
    if (demand == NULL) return;
    memset(demand, 0, sizeof(tractor_demand_t));

    uint32_t now = HAL_GetTick();
    uint8_t sbus_ok = (sbus_updated && ((now - sbus_last_time) <= SBUS_FAILSAFE_TIMEOUT_MS)) ? 1 : 0;
    demand->is_active = sbus_ok;

    if (!sbus_ok) return;

    // 1. 1号挂挡 (CH1)
    int16_t gear_mapped = SBUS_GetChannel_Mapped(RC_CH_GEARSHIFT);
    switch_3pos_t gear_pos = TractorControl_Parse3Pos(gear_mapped);
    if (gear_pos == SWITCH_POS_UP)        demand->gear = 1;
    else if (gear_pos == SWITCH_POS_DOWN) demand->gear = 2;
    else                                  demand->gear = 0;

    // 2. 离合 (CH2)
    int16_t clutch_mapped = SBUS_GetChannel_Mapped(RC_CH_CLUTCH);
    switch_2pos_t clutch_sw = TractorControl_Parse2Pos(clutch_mapped);
    demand->clutch = (clutch_sw == SWITCH_POS_ON) ? 1 : 0;

    // 3. 油门 (CH3)
    int16_t thr_mapped = SBUS_GetChannel_Mapped(RC_CH_THROTTLE);
    float thr_pct = 0.0f;
    if (thr_mapped <= 60)        thr_pct = 0.0f;
    else if (thr_mapped >= 780)  thr_pct = 100.0f;
    else thr_pct = (float)(thr_mapped - 60) / (780.0f - 60.0f) * 100.0f;
    demand->throttle_pct = thr_pct;

    // 4. 刹车 (CH4)
    int16_t brake_mapped = SBUS_GetChannel_Mapped(RC_CH_BRAKE);
    switch_2pos_t brake_sw = TractorControl_Parse2Pos(brake_mapped);
    demand->brake = (brake_sw == SWITCH_POS_ON) ? 1 : 0;

    // 5. 电动推杆 (CH6)
    int16_t act_mapped = SBUS_GetChannel_Mapped(RC_CH_ACTUATOR);
    switch_3pos_t act_pos = TractorControl_Parse3Pos(act_mapped);
    if (act_pos == SWITCH_POS_UP)        demand->actuator_dir = 1;
    else if (act_pos == SWITCH_POS_DOWN) demand->actuator_dir = -1;
    else                                 demand->actuator_dir = 0;

    // 6. 2号挂挡 (CH7)
    int16_t gear2_mapped = SBUS_GetChannel_Mapped(RC_CH_GEARSHIFT2);
    switch_3pos_t gear2_pos = TractorControl_Parse3Pos(gear2_mapped);
    if (gear2_pos == SWITCH_POS_UP)        demand->gear2 = 1;
    else if (gear2_pos == SWITCH_POS_DOWN) demand->gear2 = 2;
    else                                   demand->gear2 = 0;

    // 7. 方向盘舵机 (CH8)
    int16_t steer_mapped = SBUS_GetChannel_Mapped(RC_CH_STEERING);
    if (steer_mapped > -50 && steer_mapped < 50)
    {
        demand->steer_target_deg = 0.0f;
    }
    else
    {
        float x = -(float)steer_mapped / 1000.0f;
        if (x > 1.0f)  x = 1.0f;
        if (x < -1.0f) x = -1.0f;
        float y = 0.30f * x + 0.70f * (x * x * x);
        demand->steer_target_deg = y * STEERING_MAX_ANGLE_DEG;
    }
}

void TractorControl_ExecuteDemand(CAN_HandleTypeDef *hcan, const tractor_demand_t *demand)
{
    if (hcan == NULL || demand == NULL) return;
    uint32_t now = HAL_GetTick();

    /* 1. 1号挂挡电机 (ID 4) */
    static uint8_t last_gear_pos = 0xFF;
    if (g_motor_gearshift.is_enabled && !g_motor_gearshift.is_fault)
    {
        int32_t gear_target = GEAR_ABS_ORIGIN_POS;
        if (demand->gear == 1)      gear_target = GEAR_ABS_FWD_POS;
        else if (demand->gear == 2) gear_target = GEAR_ABS_REV_POS;
        else                        gear_target = GEAR_ABS_ORIGIN_POS;

        if (demand->gear != last_gear_pos)
        {
            last_gear_pos = demand->gear;
            g_motor_gearshift.last_sent_position = 0x7FFFFFFF;
            kinco_set_position(hcan, &g_motor_gearshift, gear_target, 0);
        }
    }
    else
    {
        last_gear_pos = 0xFF;
    }

    /* 2. 2号挂挡电机 (ID 2: 到位自动返回原点) */
    typedef enum {
        GEAR2_STATE_IDLE = 0,
        GEAR2_STATE_MOVING_TO_TARGET,
        GEAR2_STATE_RETURNING_TO_ORIGIN
    } gear2_state_e;

    static gear2_state_e s_gear2_state = GEAR2_STATE_IDLE;
    static uint8_t s_last_gear2_demand = 0;
    static uint32_t s_gear2_start_time = 0;

    if (g_motor_gearshift2.is_enabled && !g_motor_gearshift2.is_fault)
    {
        if (demand->gear2 != 0 && demand->gear2 != s_last_gear2_demand && s_gear2_state == GEAR2_STATE_IDLE)
        {
            int32_t target_pos = (demand->gear2 == 1) ? GEAR2_ABS_FWD_POS : GEAR2_ABS_REV_POS;
            g_motor_gearshift2.target_position = target_pos;
            g_motor_gearshift2.last_sent_position = 0x7FFFFFFF;
            kinco_set_position(hcan, &g_motor_gearshift2, target_pos, 0);
            s_gear2_state = GEAR2_STATE_MOVING_TO_TARGET;
            s_gear2_start_time = now;
        }

        if (s_gear2_state == GEAR2_STATE_MOVING_TO_TARGET)
        {
            int32_t target_pos = (s_last_gear2_demand == 1 || demand->gear2 == 1) ? GEAR2_ABS_FWD_POS : GEAR2_ABS_REV_POS;
            int32_t pos_err = labs(g_motor_gearshift2.actual_position - target_pos);
            if (pos_err < 3000 || (now - s_gear2_start_time >= 800))
            {
                g_motor_gearshift2.target_position = GEAR2_ABS_ORIGIN_POS;
                g_motor_gearshift2.last_sent_position = 0x7FFFFFFF;
                kinco_set_position(hcan, &g_motor_gearshift2, GEAR2_ABS_ORIGIN_POS, 0);
                s_gear2_state = GEAR2_STATE_RETURNING_TO_ORIGIN;
                s_gear2_start_time = now;
            }
        }
        else if (s_gear2_state == GEAR2_STATE_RETURNING_TO_ORIGIN)
        {
            int32_t origin_err = labs(g_motor_gearshift2.actual_position - GEAR2_ABS_ORIGIN_POS);
            if (origin_err < 3000 || (now - s_gear2_start_time >= 800))
            {
                s_gear2_state = GEAR2_STATE_IDLE;
            }
        }

        s_last_gear2_demand = demand->gear2;
    }
    else
    {
        s_gear2_state = GEAR2_STATE_IDLE;
        s_last_gear2_demand = 0;
    }

    /* 3. 离合电机 (ID 5) */
    static uint8_t last_clutch_pos = 0xFF;
    if (g_motor_clutch.is_enabled && !g_motor_clutch.is_fault)
    {
        int32_t clutch_target = (demand->clutch != 0) ? CLUTCH_ABS_MAX_POS : CLUTCH_ABS_ORIGIN_POS;
        g_motor_clutch.target_position = clutch_target;

        if (demand->clutch != last_clutch_pos)
        {
            last_clutch_pos = demand->clutch;
            g_motor_clutch.last_sent_position = 0x7FFFFFFF;
            if (demand->clutch != 0)
            {
                kinco_set_profile_velocity_custom(hcan, &g_motor_clutch, CLUTCH_PRESS_SPEED_RPM, CLUTCH_PRESS_ACC_RPM_S, CLUTCH_PRESS_ACC_RPM_S);
                kinco_set_position(hcan, &g_motor_clutch, CLUTCH_ABS_MAX_POS, 0);
            }
            else
            {
                kinco_set_profile_velocity_custom(hcan, &g_motor_clutch, CLUTCH_RELEASE_SPEED_RPM, CLUTCH_RELEASE_ACC_RPM_S, CLUTCH_RELEASE_ACC_RPM_S);
                kinco_set_position(hcan, &g_motor_clutch, CLUTCH_ABS_ORIGIN_POS, 0);
            }
        }
    }
    else
    {
        last_clutch_pos = 0xFF;
    }

    /* 4. 油门电机 (ID 6) */
    static int32_t last_thr_target = 0x7FFFFFFF;
    if (g_motor_throttle.is_enabled && !g_motor_throttle.is_fault)
    {
        float thr_pct = demand->throttle_pct;
        if (thr_pct < 0.0f)   thr_pct = 0.0f;
        if (thr_pct > 100.0f) thr_pct = 100.0f;

        int32_t target_pos = 0;
        if (thr_pct <= 0.0f)        target_pos = THROTTLE_ABS_ORIGIN_POS;
        else if (thr_pct >= 100.0f) target_pos = THROTTLE_ABS_MAX_POS;
        else                        target_pos = THROTTLE_ABS_ORIGIN_POS + (int32_t)(thr_pct / 100.0f * (float)THROTTLE_ABS_TOTAL_SPAN);

        if (last_thr_target == 0x7FFFFFFF || labs(target_pos - last_thr_target) > 80 || (thr_pct == 0.0f && last_thr_target != target_pos) || (thr_pct >= 100.0f && last_thr_target != target_pos))
        {
            last_thr_target = target_pos;
            g_motor_throttle.last_sent_position = 0x7FFFFFFF;
            kinco_set_position(hcan, &g_motor_throttle, target_pos, 0);
        }
    }
    else
    {
        last_thr_target = 0x7FFFFFFF;
    }

    /* 5. 刹车电机 (ID 3) */
    static uint8_t last_brake_sw = 0xFF;
    if (g_motor_brake.is_enabled && !g_motor_brake.is_fault)
    {
        uint8_t brake_sw = (demand->brake != 0) ? 1 : 0;
        int32_t brake_target = (brake_sw) ? BRAKE_ABS_MAX_POS : BRAKE_ABS_ORIGIN_POS;

        if (brake_sw != last_brake_sw)
        {
            last_brake_sw = brake_sw;
            g_motor_brake.last_sent_position = 0x7FFFFFFF;
            kinco_set_position(hcan, &g_motor_brake, brake_target, 0);
        }
    }
    else
    {
        last_brake_sw = 0xFF;
    }

    /* 6. 电动推杆 (继电器换向) */
    static int8_t last_actuator_state = 0;
    static uint8_t interp_step = 0;
    static uint32_t break_time = 0;
    static int8_t target_actuator_state = 0;

    int8_t current_act_dir = demand->actuator_dir;
    uint8_t group1_mask = (1u << RELAY_CH3) | (1u << RELAY_CH5);
    uint8_t group2_mask = (1u << RELAY_CH4) | (1u << RELAY_CH6);

    if (current_act_dir != last_actuator_state)
    {
        last_actuator_state = current_act_dir;
        target_actuator_state = current_act_dir;

        if (target_actuator_state > 0)
        {
            g_relay_module.states_mask &= ~group2_mask;
            relay_control_all(hcan, g_relay_module.states_mask);
            break_time = now;
            interp_step = 1;
        }
        else if (target_actuator_state < 0)
        {
            g_relay_module.states_mask &= ~group1_mask;
            relay_control_all(hcan, g_relay_module.states_mask);
            break_time = now;
            interp_step = 2;
        }
        else
        {
            g_relay_module.states_mask &= ~(group1_mask | group2_mask);
            relay_control_all(hcan, g_relay_module.states_mask);
            interp_step = 0;
        }
    }

    if (interp_step == 1 && (now - break_time >= 50))
    {
        g_relay_module.states_mask |= group1_mask;
        relay_control_all(hcan, g_relay_module.states_mask);
        interp_step = 0;
    }
    else if (interp_step == 2 && (now - break_time >= 50))
    {
        g_relay_module.states_mask |= group2_mask;
        relay_control_all(hcan, g_relay_module.states_mask);
        interp_step = 0;
    }

    /* 7. 方向盘舵机 (ID 7 / ID 1) 周期 40ms 或角度改变时发送控制指令 */
    static uint32_t last_steer_send_time = 0;
    static float last_sent_steer_deg = 9999.0f;

    float tgt_deg = demand->steer_target_deg;
    if ((now - last_steer_send_time >= 40) || fabsf(tgt_deg - last_sent_steer_deg) > 0.1f)
    {
        last_steer_send_time = now;
        last_sent_steer_deg = tgt_deg;
        MDU_Motor_SetAngle(hcan, tgt_deg);
    }
}

void TractorControl_Failsafe(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return;

    static uint32_t last_failsafe_cmd_time = 0;
    uint32_t now = HAL_GetTick();

    if (now - last_failsafe_cmd_time < 500)
    {
        return;
    }
    last_failsafe_cmd_time = now;

    int32_t g2_origin = g_motor_gearshift2.home_captured ? g_motor_gearshift2.home_position : GEAR2_ABS_ORIGIN_POS;
    if (g_motor_gearshift2.is_enabled && g_motor_gearshift2.last_sent_position != g2_origin)
    {
        kinco_set_position(hcan, &g_motor_gearshift2, g2_origin, 0);
    }
    if (g_motor_gearshift.is_enabled && g_motor_gearshift.last_sent_position != GEAR_ABS_ORIGIN_POS)
    {
        kinco_set_position(hcan, &g_motor_gearshift, GEAR_ABS_ORIGIN_POS, 0);
    }
    if (g_motor_throttle.is_enabled && g_motor_throttle.last_sent_position != THROTTLE_ABS_ORIGIN_POS)
    {
        kinco_set_position(hcan, &g_motor_throttle, THROTTLE_ABS_ORIGIN_POS, 0);
    }
    if (g_motor_brake.is_enabled && g_motor_brake.last_sent_position != BRAKE_ABS_MAX_POS)
    {
        kinco_set_position(hcan, &g_motor_brake, BRAKE_ABS_MAX_POS, 0);
    }
    if (g_motor_clutch.is_enabled && g_motor_clutch.last_sent_position != CLUTCH_ABS_MAX_POS)
    {
        kinco_set_profile_velocity_custom(hcan, &g_motor_clutch, 1500, 3000, 3000);
        kinco_set_position(hcan, &g_motor_clutch, CLUTCH_ABS_MAX_POS, 0);
    }

    g_relay_module.states_mask = 0;
    relay_control_all(hcan, 0);
    MDU_Motor_SetSpeed(hcan, 0.0f);
}

void TractorControl_Update(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return;

    tractor_demand_t sbus_demand;
    TractorControl_GetSBUSDemand(&sbus_demand);

    if (sbus_demand.is_active)
    {
        TractorControl_ExecuteDemand(hcan, &sbus_demand);
    }
    else
    {
        TractorControl_Failsafe(hcan);
    }
}
