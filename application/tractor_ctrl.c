/**
  ******************************************************************************
  * @file       tractor_ctrl.c
  * @brief      拖拉机改装核心控制实现 (带前轮转向基准 -7.91° 有界平滑伺服回正)
  ******************************************************************************
  */
#include "tractor_ctrl.h"
#include "analog_input.h"
#include "inclinometer.h"
#include <stdlib.h>
#include <math.h>
#include <string.h>

kinco_motor_t g_motor_gearshift2; // ID 2
kinco_motor_t g_motor_brake;      // ID 3
kinco_motor_t g_motor_gearshift;  // ID 4
kinco_motor_t g_motor_clutch;     // ID 5
kinco_motor_t g_motor_throttle;   // ID 6

float g_steer_closed_loop_adj_deg = 0.0f; // 全局当前前轮残余误差 (度，供串口监视)

static float s_steer_auto_target_deg = 0.0f; // 方向盘纠偏目标累计角度
static uint8_t s_steer_auto_inited = 0;

/**
  * @brief  前轮分段平滑减速闭环回正计算 (远距离恒速快速纠偏，近距离自动平滑减速防震荡)
  * @param  dt 计算周期 (秒)
  * @return 方向盘舵机目标位置 (度)
  */
static float Steer_ConstantSpeed_Track(float dt)
{
    // 如果倾角传感器掉线，保持当前位置
    if (!Inclinometer_IsOnline())
    {
        return s_steer_auto_target_deg;
    }

    if (!s_steer_auto_inited)
    {
        s_steer_auto_target_deg = g_mdu_steering_motor.actual_angle_deg;
        s_steer_auto_inited = 1;
    }

    // 1. 计算前轮当前与正中基准 (-7.91°) 的误差
    // 目标基准 - 实测角度: FRONT_WHEEL_ZERO_ROLL_DEG - g_inclinometer.roll_deg
    float error = FRONT_WHEEL_ZERO_ROLL_DEG - g_inclinometer.roll_deg;
    g_steer_closed_loop_adj_deg = error; // 记录残余误差供串口监视显示

    float abs_err = fabsf(error);

    // 2. 前轮死区判断: 在 ±0.20° 以内认为已到达正中零点，完全停转
    if (abs_err <= STEER_CLOSED_LOOP_DEADBAND_DEG)
    {
        // 到达零点死区，保持当前目标角度不变（电机停转）
        return s_steer_auto_target_deg;
    }

    // 3. 分段平滑减速逻辑 (远距离快速推进，快到零点时平滑减速):
    // 距离远 (> 1.50°): 保持 STEER_TRACK_FAST_SPEED_DPS (90°/s) 快速转动
    // 距离近 (0.20° ~ 1.50°): 速度线性减小至 STEER_TRACK_SLOW_SPEED_DPS (15°/s)，轻柔逼近零点
    float current_speed_dps = STEER_TRACK_FAST_SPEED_DPS;
    if (abs_err < STEER_SLOWDOWN_THRESHOLD_DEG)
    {
        float ratio = (abs_err - STEER_CLOSED_LOOP_DEADBAND_DEG) / (STEER_SLOWDOWN_THRESHOLD_DEG - STEER_CLOSED_LOOP_DEADBAND_DEG);
        if (ratio < 0.0f) ratio = 0.0f;
        if (ratio > 1.0f) ratio = 1.0f;
        current_speed_dps = STEER_TRACK_SLOW_SPEED_DPS + ratio * (STEER_TRACK_FAST_SPEED_DPS - STEER_TRACK_SLOW_SPEED_DPS);
    }

    // 4. 方向盘目标角度积分累加 (无圈数限制，可无限多圈自由转动直到前轮到达零点):
    // STEER_CORRECT_DIR_POLARITY: 1 为正向，-1 为反向
    float step = (float)STEER_CORRECT_DIR_POLARITY * current_speed_dps * dt;
    if (error > 0.0f)
    {
        s_steer_auto_target_deg += step;
    }
    else
    {
        s_steer_auto_target_deg -= step;
    }

    return s_steer_auto_target_deg;
}

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

    // 7. 方向盘舵机 (CH8) - 摇杆中位识别与手动优先
    int16_t steer_mapped = SBUS_GetChannel_Mapped(RC_CH_STEERING);
    if (steer_mapped > -50 && steer_mapped < 50)
    {
        demand->steer_target_deg = 0.0f;
        demand->steer_is_neutral = 1; // 摇杆回中：使能前轮闭环自动回正
    }
    else
    {
        demand->steer_is_neutral = 0; // 手动打方向：控制权交给遥控器
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

    /* 1. 1号挂挡电机 (ID 4: 具备 100ms 自动重试防漏发机制) */
    static uint8_t last_gear_demand = 0xFF;
    static uint32_t last_gear_send_time = 0;

    if (g_motor_gearshift.is_enabled && !g_motor_gearshift.is_fault)
    {
        int32_t gear_target = GEAR_ABS_ORIGIN_POS;
        if (demand->gear == 1)      gear_target = GEAR_ABS_FWD_POS;
        else if (demand->gear == 2) gear_target = GEAR_ABS_REV_POS;
        else                        gear_target = GEAR_ABS_ORIGIN_POS;

        g_motor_gearshift.target_position = gear_target;

        int32_t pos_err = labs(g_motor_gearshift.actual_position - gear_target);
        uint8_t demand_changed = (demand->gear != last_gear_demand);

        if (demand_changed || (pos_err > 5000 && labs(g_motor_gearshift.actual_velocity) < 5 && (now - last_gear_send_time >= 300)))
        {
            last_gear_demand = demand->gear;
            last_gear_send_time = now;
            kinco_set_position(hcan, &g_motor_gearshift, gear_target, 0);
        }
    }
    else
    {
        last_gear_demand = 0xFF;
    }

    /* 2. 2号挂挡电机 (ID 2: 到达目标点按配置时间停留后自动返回原点) */
    typedef enum {
        GEAR2_STATE_IDLE = 0,
        GEAR2_STATE_MOVING_TO_TARGET,
        GEAR2_STATE_HOLDING_AT_TARGET,
        GEAR2_STATE_RETURNING_TO_ORIGIN
    } gear2_state_e;

    static gear2_state_e s_gear2_state = GEAR2_STATE_IDLE;
    static uint8_t s_last_gear2_demand = 0;
    static uint32_t s_gear2_start_time = 0;
    static uint32_t s_gear2_last_send = 0;
    static uint32_t s_gear2_hold_start_time = 0;

    if (g_motor_gearshift2.is_enabled && !g_motor_gearshift2.is_fault)
    {
        if (demand->gear2 != 0 && demand->gear2 != s_last_gear2_demand)
        {
            s_last_gear2_demand = demand->gear2;
            int32_t target_pos = (demand->gear2 == 1) ? GEAR2_ABS_FWD_POS : GEAR2_ABS_REV_POS;
            g_motor_gearshift2.target_position = target_pos;
            kinco_set_position(hcan, &g_motor_gearshift2, target_pos, 0);
            s_gear2_state = GEAR2_STATE_MOVING_TO_TARGET;
            s_gear2_start_time = now;
            s_gear2_last_send = now;
        }
        else if (demand->gear2 == 0)
        {
            s_last_gear2_demand = 0;
        }

        if (s_gear2_state == GEAR2_STATE_MOVING_TO_TARGET)
        {
            int32_t target_pos = (s_last_gear2_demand == 1) ? GEAR2_ABS_FWD_POS : GEAR2_ABS_REV_POS;
            int32_t pos_err = labs(g_motor_gearshift2.actual_position - target_pos);

            if (pos_err > 5000 && labs(g_motor_gearshift2.actual_velocity) < 5 && (now - s_gear2_last_send >= 300) && (now - s_gear2_start_time < 700))
            {
                s_gear2_last_send = now;
                kinco_set_position(hcan, &g_motor_gearshift2, target_pos, 0);
            }

            if (pos_err < 3000 || (now - s_gear2_start_time >= 1500))
            {
                s_gear2_state = GEAR2_STATE_HOLDING_AT_TARGET;
                s_gear2_hold_start_time = now;
            }
        }
        else if (s_gear2_state == GEAR2_STATE_HOLDING_AT_TARGET)
        {
            uint32_t hold_duration = (s_last_gear2_demand == 1) ? GEAR2_FWD_HOLD_TIME_MS : GEAR2_REV_HOLD_TIME_MS;
            if (now - s_gear2_hold_start_time >= hold_duration)
            {
                g_motor_gearshift2.target_position = GEAR2_ABS_ORIGIN_POS;
                kinco_set_position(hcan, &g_motor_gearshift2, GEAR2_ABS_ORIGIN_POS, 0);
                s_gear2_state = GEAR2_STATE_RETURNING_TO_ORIGIN;
                s_gear2_start_time = now;
                s_gear2_last_send = now;
            }
        }
        else if (s_gear2_state == GEAR2_STATE_RETURNING_TO_ORIGIN)
        {
            int32_t origin_err = labs(g_motor_gearshift2.actual_position - GEAR2_ABS_ORIGIN_POS);

            if (origin_err > 5000 && labs(g_motor_gearshift2.actual_velocity) < 5 && (now - s_gear2_last_send >= 300) && (now - s_gear2_start_time < 700))
            {
                s_gear2_last_send = now;
                kinco_set_position(hcan, &g_motor_gearshift2, GEAR2_ABS_ORIGIN_POS, 0);
            }

            if (origin_err < 3000 || (now - s_gear2_start_time >= 1500))
            {
                s_gear2_state = GEAR2_STATE_IDLE;
            }
        }
    }
    else
    {
        s_gear2_state = GEAR2_STATE_IDLE;
        s_last_gear2_demand = 0;
    }

    /* 3. 离合电机 (ID 5) */
    static uint8_t last_clutch_demand = 0xFF;
    static uint32_t last_clutch_send_time = 0;

    if (g_motor_clutch.is_enabled && !g_motor_clutch.is_fault)
    {
        int32_t clutch_target = (demand->clutch != 0) ? CLUTCH_ABS_MAX_POS : CLUTCH_ABS_ORIGIN_POS;
        g_motor_clutch.target_position = clutch_target;

        int32_t pos_err = labs(g_motor_clutch.actual_position - clutch_target);
        uint8_t demand_changed = (demand->clutch != last_clutch_demand);

        if (demand_changed || (pos_err > 5000 && labs(g_motor_clutch.actual_velocity) < 5 && (now - last_clutch_send_time >= 300)))
        {
            last_clutch_demand = demand->clutch;
            last_clutch_send_time = now;
            kinco_set_position(hcan, &g_motor_clutch, clutch_target, 0);
        }
    }
    else
    {
        last_clutch_demand = 0xFF;
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

        g_motor_throttle.target_position = target_pos;

        if (last_thr_target == 0x7FFFFFFF || labs(target_pos - last_thr_target) > 80 || (thr_pct == 0.0f && last_thr_target != target_pos) || (thr_pct >= 100.0f && last_thr_target != target_pos))
        {
            last_thr_target = target_pos;
            kinco_set_position(hcan, &g_motor_throttle, target_pos, 0);
        }
    }
    else
    {
        last_thr_target = 0x7FFFFFFF;
    }

    /* 5. 刹车电机 (ID 3) */
    static uint8_t last_brake_demand = 0xFF;
    static uint32_t last_brake_send_time = 0;

    if (g_motor_brake.is_enabled && !g_motor_brake.is_fault)
    {
        uint8_t brake_sw = (demand->brake != 0) ? 1 : 0;
        int32_t brake_target = (brake_sw) ? BRAKE_ABS_MAX_POS : BRAKE_ABS_ORIGIN_POS;
        g_motor_brake.target_position = brake_target;

        int32_t pos_err = labs(g_motor_brake.actual_position - brake_target);
        uint8_t demand_changed = (brake_sw != last_brake_demand);

        if (demand_changed || (pos_err > 5000 && labs(g_motor_brake.actual_velocity) < 5 && (now - last_brake_send_time >= 300)))
        {
            last_brake_demand = brake_sw;
            last_brake_send_time = now;
            kinco_set_position(hcan, &g_motor_brake, brake_target, 0);
        }
    }
    else
    {
        last_brake_demand = 0xFF;
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

    /* 7. 方向盘舵机 (ID 7 / ID 1) - 遥控打方向绝对优先；摇杆回中时恒速回正到 -7.91° */
    static uint32_t last_steer_send_time = 0;
    static float last_sent_steer_deg = 9999.0f;

    float final_steer_deg = 0.0f;

    if (demand->steer_is_neutral)
    {
        // 摇杆居中: 启用恒速自动纠偏回正
        float dt = (last_steer_send_time == 0) ? 0.04f : ((float)(now - last_steer_send_time) / 1000.0f);
        if (dt <= 0.001f || dt > 0.2f) dt = 0.04f;

        final_steer_deg = Steer_ConstantSpeed_Track(dt);
    }
    else
    {
        // 摇杆打方向: 手动遥控绝对优先
        final_steer_deg = demand->steer_target_deg;
        // 手动打方向时同步当前位置，松手回中时从当前方向盘实际位置平滑开始恒速纠偏
        s_steer_auto_target_deg = g_mdu_steering_motor.actual_angle_deg;
        g_steer_closed_loop_adj_deg = 0.0f;
    }

    if ((now - last_steer_send_time >= 40) || fabsf(final_steer_deg - last_sent_steer_deg) > 0.1f)
    {
        last_steer_send_time = now;
        last_sent_steer_deg = final_steer_deg;
        MDU_Motor_SetAngle(hcan, final_steer_deg);
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
