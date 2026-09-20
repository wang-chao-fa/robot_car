/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 拖拉机改装遥控控制主程序 (5步科电机 + 1方向盘舵机 + 1推杆继电器)
  * @target         : STM32F407IGH (RoboMaster 开发板 C 型)
  ******************************************************************************
  */

#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

#include "bsp_can.h"
#include "CAN_receive.h"
#include "mdu_steering_motor.h"
#include "rc_sbus.h"
#include "tractor_ctrl.h"
#include "relay_output.h"
#include "analog_input.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

void SystemClock_Config(void);

int main(void)
{
    /* MCU 基础初始化 */
    HAL_Init();
    SystemClock_Config();

    /* 外设初始化 */
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    /* 1. CAN 接收过滤器配置 */
    can_filter_init();

    /* 2. 继电器模块与模拟量采集模块初始化 */
    relay_init(0x101);
    Analog_Input_Init(&hcan1);

    /* 3. MDU 方向盘电机初始化 */
    MDU_Motor_Init(&hcan1);

    /* 等待 1500ms 确保步科驱动器上电就绪 */
    HAL_Delay(1500);

    /* 4. 初始化 5 台步科伺服电机 */
    kinco_motor_init(&g_motor_gearshift2, 2); // 2号档位 (CH7)
    kinco_motor_init(&g_motor_brake,      3); // 3号刹车 (CH4)
    kinco_motor_init(&g_motor_gearshift,  4); // 1号档位 (CH1)
    kinco_motor_init(&g_motor_clutch,     5); // 离合电机 (CH2)
    kinco_motor_init(&g_motor_throttle,   6); // 油门电机 (CH3)

    /* 5. 同步电机出厂速度与加减速度参数 */
    kinco_motor_config_sync(&hcan1, &g_motor_gearshift2, 800,  1500, 1500);
    kinco_motor_config_sync(&hcan1, &g_motor_brake,      120,   200,  200);
    kinco_motor_config_sync(&hcan1, &g_motor_gearshift,  800,  1000, 1000);
    kinco_motor_config_sync(&hcan1, &g_motor_clutch,    1500,  3000, 3000);
    kinco_motor_config_sync(&hcan1, &g_motor_throttle,    48,   120,  120);

    /* 6. 请求使能所有电机 */
    kinco_motor_enable(&hcan1, &g_motor_gearshift2);
    kinco_motor_enable(&hcan1, &g_motor_brake);
    kinco_motor_enable(&hcan1, &g_motor_clutch);
    kinco_motor_enable(&hcan1, &g_motor_gearshift);
    kinco_motor_enable(&hcan1, &g_motor_throttle);
    MDU_Motor_Enable(&hcan1);

    /* 7. 启动 SBUS 遥控器 DMA 空闲中断接收 */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, sbus_rx_buf, SBUS_RX_BUF_NUM);

    static uint32_t print_tick = 0;
    static uint32_t ctrl_loop_tick = 0;
    static char tx_buf[512];

    /* 主循环 */
    while (1)
    {
        uint32_t now = HAL_GetTick();

        /* CAN 总线错误监测与自动恢复 */
        CAN_Bus_Error_Recovery(&hcan1);

        /* 步科 CANopen CiA402 状态机推进与轮询 */
        kinco_control_loop(&hcan1, &g_motor_gearshift2);
        kinco_control_loop(&hcan1, &g_motor_brake);
        kinco_control_loop(&hcan1, &g_motor_clutch);
        kinco_control_loop(&hcan1, &g_motor_gearshift);
        kinco_control_loop(&hcan1, &g_motor_throttle);

        /* 20ms 周期拖拉机核心逻辑控制环 */
        if (now - ctrl_loop_tick >= 20)
        {
            ctrl_loop_tick = now;
            uint8_t sbus_ok = (sbus_updated && ((now - sbus_last_time) <= SBUS_FAILSAFE_TIMEOUT_MS)) ? 1 : 0;

            if (sbus_ok)
            {
                TractorControl_Update(&hcan1);
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);   // 遥控在线红灯熄灭
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); // 绿灯亮
            }
            else
            {
                TractorControl_Failsafe(&hcan1);
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); // 遥控离线红灯常亮报警
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
            }
        }

        /* 200ms 周期串口非阻塞中断打印 (绝对不卡主循环) */
        if (now - print_tick >= 200)
        {
            print_tick = now;
            HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

            if (huart1.gState == HAL_UART_STATE_READY)
            {
                int16_t ch1_gear  = SBUS_GetChannel_Mapped(RC_CH_GEARSHIFT);
                int16_t ch2_clut  = SBUS_GetChannel_Mapped(RC_CH_CLUTCH);
                int16_t ch3_thro  = SBUS_GetChannel_Mapped(RC_CH_THROTTLE);
                int16_t ch4_brak  = SBUS_GetChannel_Mapped(RC_CH_BRAKE);
                int16_t ch6_actu  = SBUS_GetChannel_Mapped(RC_CH_ACTUATOR);
                int16_t ch7_gear2 = SBUS_GetChannel_Mapped(RC_CH_GEARSHIFT2);
                int16_t ch8_ster  = SBUS_GetChannel_Mapped(RC_CH_STEERING);
                uint8_t failsafe_flag = ((now - sbus_last_time) > SBUS_FAILSAFE_TIMEOUT_MS) ? 1 : 0;

                long steer_act_x10 = (long)(g_mdu_steering_motor.actual_angle_deg * 10.0f);
                long steer_tgt_x10 = (long)(g_mdu_steering_motor.target_angle_deg * 10.0f);

                int len = snprintf(tx_buf, sizeof(tx_buf),
                    "[RC] G1:%4d G2:%4d Cl:%4d Th:%4d Br:%4d Ac:%4d St:%4d | FS:%d\r\n"
                    "ID2(G2):en=%d st=0x%04X tgt=%ld act=%ld | ID3(Brk):en=%d st=0x%04X tgt=%ld act=%ld\r\n"
                    "ID4(G1):en=%d st=0x%04X tgt=%ld act=%ld | ID5(Clt):en=%d st=0x%04X tgt=%ld act=%ld\r\n"
                    "ID6(Thr):en=%d st=0x%04X tgt=%ld act=%ld | ID7(Str):tgt=%ld.%01ld act=%ld.%01ld\r\n\r\n",
                    ch1_gear, ch7_gear2, ch2_clut, ch3_thro, ch4_brak, ch6_actu, ch8_ster, failsafe_flag,
                    g_motor_gearshift2.is_enabled, g_motor_gearshift2.statusword, (long)g_motor_gearshift2.target_position, (long)g_motor_gearshift2.actual_position,
                    g_motor_brake.is_enabled, g_motor_brake.statusword, (long)g_motor_brake.target_position, (long)g_motor_brake.actual_position,
                    g_motor_gearshift.is_enabled, g_motor_gearshift.statusword, (long)g_motor_gearshift.target_position, (long)g_motor_gearshift.actual_position,
                    g_motor_clutch.is_enabled, g_motor_clutch.statusword, (long)g_motor_clutch.target_position, (long)g_motor_clutch.actual_position,
                    g_motor_throttle.is_enabled, g_motor_throttle.statusword, (long)g_motor_throttle.target_position, (long)g_motor_throttle.actual_position,
                    steer_tgt_x10 / 10, labs(steer_tgt_x10 % 10), steer_act_x10 / 10, labs(steer_act_x10 % 10)
                );

                if (len > 0)
                {
                    if (len >= (int)sizeof(tx_buf)) len = (int)sizeof(tx_buf) - 1;
                    HAL_UART_Transmit_IT(&huart1, (uint8_t *)tx_buf, (uint16_t)len);
                }
            }
        }
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 6;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        for (volatile int i = 0; i < 1000000; i++);
    }
}
