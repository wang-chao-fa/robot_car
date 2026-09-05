/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 差速小车遥控控制主程序
  *
  * 硬件平台: STM32F407IGH (RoboMaster 开发板 C 型)
  *
  * 功能说明:
  *   - 通过 USART3 (SBUS) 接收天地飞遥控器指令 (16通道, S.BUS协议)
  *   - 通过 CAN1 (CANopen / 50kbps) 控制两个步科低压伺服电机
  *   - 单摇杆差速控制: 一轴前进/后退, 另一轴左/右转向
  *   - 失控保护: 遥控信号丢失超过 500ms 后自动停车
  *   - USART1 (115200) 用于调试打印
  *
  * 电机节点 ID:
  *   左电机: NodeID = 2 (ECAN节点保护ID 0x0702)
  *   右电机: NodeID = 8 (ECAN节点保护ID 0x0708)
  *
  * 遥控通道分配 (在 diff_drive.h 中修改):
  *   RC_CH_THROTTLE = 1  (CH2, 左摇杆Y轴, 前进后退)
  *   RC_CH_STEERING = 0  (CH1, 左摇杆X轴, 左右转向)
  *
  * 调试步骤:
  *   1. 先将两个电机 CAN 口都接入 CAN1 (PD0/PD1)，上电观察 LED 状态
  *   2. 用 remote_Controller 工程确认各通道数值，记录对应摇杆
  *   3. 修改 diff_drive.h 中的 RC_CH_THROTTLE 和 RC_CH_STEERING
  *   4. 若小车前进方向相反，修改 diff_drive.c 中的 MOTOR_LEFT_INVERT
  *   5. 调整 diff_drive.h 中 MAX_SPEED_RPM 设置合适速度上限
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "CAN_receive.h"
#include "rc_sbus.h"
#include "diff_drive.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 控制周期 (ms): 每隔此时间执行一次速度下发 */
#define CONTROL_PERIOD_MS     20

/* 电机状态打印周期 (ms): 调试用 */
#define DEBUG_PRINT_PERIOD_MS 500

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t tx_buf[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  应用程序入口
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */

    /* MCU 初始化 ---------------------------------------------------------*/
    HAL_Init();
    SystemClock_Config();

    /* 外设初始化 ---------------------------------------------------------*/
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();

    /* USER CODE BEGIN 2 */

    /* --- CAN1 滤波器配置 + 启动 --- */
    can_filter_init();

    /* --- 初始化双电机 --- */
    kinco_motor_init(&g_motor_left,  2);   // 左电机 NodeID = 2
    kinco_motor_init(&g_motor_right, 8);   // 右电机 NodeID = 8

    /* --- 使能双电机 (NMT Start + CiA402 状态机) --- */
    kinco_motor_enable(&hcan1, &g_motor_left);
    kinco_motor_enable(&hcan1, &g_motor_right);

    /* --- 设置轮廓速度模式 --- */
    kinco_set_mode(&hcan1, &g_motor_left,  KINCO_MODE_PV);
    kinco_set_mode(&hcan1, &g_motor_right, KINCO_MODE_PV);

    /* --- 启动 SBUS 接收: 使能 IDLE 中断 + DMA --- */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3, sbus_rx_buf, SBUS_RX_BUF_NUM);

    /* --- 启动 ROS 通信接收中断: 使能 RXNE 接收中断 --- */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);

    /* LED 绿灯亮: 表示初始化完成 */
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

    uint16_t len = sprintf((char *)tx_buf,
        "[robot_car] Init OK. Left NodeID=2, Right NodeID=8\r\n"
        "Ctrl: Left-Joystick single-stick | Throttle=CH%d(Y) Steering=CH%d(X)\r\n",
        RC_CH_THROTTLE + 1, RC_CH_STEERING + 1);
    HAL_UART_Transmit(&huart1, tx_buf, len, 200);

    /* USER CODE END 2 */

    /* 主循环 -------------------------------------------------------------*/
    /* USER CODE BEGIN WHILE */

    uint32_t control_tick = 0;
    uint32_t debug_tick   = 0;

    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        uint32_t now = HAL_GetTick();

        /* ======================== 电机状态机维护 ======================== */
        kinco_control_loop(&hcan1, &g_motor_left);
        kinco_control_loop(&hcan1, &g_motor_right);

        /* ======================== 控制周期任务 ========================== */
        if (now - control_tick >= CONTROL_PERIOD_MS)
        {
            control_tick = now;

            /* --- 遥控器与 ROS 双重控制抢占调度逻辑 --- */
            int16_t throttle = SBUS_GetChannel_Mapped(RC_CH_THROTTLE);
            int16_t steering = SBUS_GetChannel_Mapped(RC_CH_STEERING);

            /* 判断遥控器信号状态 */
            uint8_t sbus_timeout = ((now - sbus_last_time) > SBUS_FAILSAFE_TIMEOUT_MS) ? 1 : 0;
            /* 判断摇杆是否处于零位死区内 (|映射值| < 30) */
            uint8_t rc_in_center = (throttle > -30 && throttle < 30 && steering > -30 && steering < 30) ? 1 : 0;
            /* 判断 ROS 控制指令是否在 500ms 内有效更新 */
            uint8_t ros_cmd_valid = ((now - g_ros_cmd_last_time) <= 500) ? 1 : 0;

            if (!sbus_timeout && !rc_in_center)
            {
                /* 【优先级 1】: 遥控器手动抢占控制 (只要人动摇杆，立刻响应遥控器) */
                DiffDrive_Update(throttle, steering,
                                 &g_motor_left, &g_motor_right,
                                 &hcan1);

                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);   // 红灯灭
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); // 绿灯亮
            }
            else if (ros_cmd_valid)
            {
                /* 【优先级 2】: ROS 自动驾驶指令控制 (摇杆归中且 ROS 指令有效) */
                DiffDrive_UpdateFromROS(g_ros_cmd_v, g_ros_cmd_w,
                                        &g_motor_left, &g_motor_right,
                                        &hcan1);

                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);   // 红灯灭
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET); // 绿灯亮
            }
            else
            {
                /* 【优先级 3】: 遥控器中位/断连 且 ROS 无有效指令 -> 强制停车 */
                kinco_set_velocity(&hcan1, &g_motor_left,  0);
                kinco_set_velocity(&hcan1, &g_motor_right, 0);

                if (sbus_timeout)
                {
                    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET); // 红灯警报
                }
            }

            /* --- 解算并通过 USART6 发送 ROS 轮式里程计所需的速度数据 (50Hz) --- */
            float ros_linear_v  = 0.0f;
            float ros_angular_w = 0.0f;
            DiffDrive_GetOdomVelocity(&g_motor_left, &g_motor_right, &ros_linear_v, &ros_angular_w);

            static uint8_t ros_tx_buf[64];
            uint16_t ros_len = sprintf((char *)ros_tx_buf, "$ODOM,%.3f,%.3f\r\n", ros_linear_v, ros_angular_w);
            HAL_UART_Transmit(&huart6, ros_tx_buf, ros_len, 20);
        }

        /* ======================== 调试打印 (500ms) ====================== */
        if (now - debug_tick >= DEBUG_PRINT_PERIOD_MS)
        {
            debug_tick = now;

            int16_t thr = SBUS_GetChannel_Mapped(RC_CH_THROTTLE);
            int16_t str = SBUS_GetChannel_Mapped(RC_CH_STEERING);

            uint8_t failsafe_flag = ((now - sbus_last_time) > SBUS_FAILSAFE_TIMEOUT_MS) ? 1 : 0;

            len = sprintf((char *)tx_buf,
                "RC: THR=%5d STR=%5d | FS=%d\r\n"
                "L: en=%d rpm=%4ld | R: en=%d rpm=%4ld\r\n"
                "CH1-8: %4d %4d %4d %4d %4d %4d %4d %4d\r\n\r\n",
                thr, str, failsafe_flag,
                g_motor_left.is_enabled,
                (long)kinco_dec_to_rpm(g_motor_left.actual_velocity,  ENCODER_RESOLUTION),
                g_motor_right.is_enabled,
                (long)kinco_dec_to_rpm(g_motor_right.actual_velocity, ENCODER_RESOLUTION),
                rc_channels[0], rc_channels[1], rc_channels[2], rc_channels[3],
                rc_channels[4], rc_channels[5], rc_channels[6], rc_channels[7]);

            HAL_UART_Transmit(&huart1, tx_buf, len, 200);
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief  系统时钟配置 (168MHz, HSE 25MHz 外部晶振)
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 6;
    RCC_OscInitStruct.PLL.PLLN       = 168;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;   // APB1 = 42MHz (CAN/USART)
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  错误处理: 禁用中断，红灯闪烁，原地等待
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();

    /* 红灯常亮表示 Fatal Error */
    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);

    while (1)
    {
        // 系统进入安全停止状态
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
