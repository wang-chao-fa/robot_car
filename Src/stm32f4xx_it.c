/**
  ******************************************************************************
  * @file       stm32f4xx_it.c
  * @brief      系统中断与异常服务处理函数
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"
#include "rc_sbus.h"
#include "auto_serial_ctrl.h"

extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ==================== Cortex-M4 系统异常处理 ==================== */

void NMI_Handler(void) {}

void HardFault_Handler(void)
{
    /* 发生硬件故障时，红灯高频闪烁指示 */
    while (1)
    {
        HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
        for (volatile int i = 0; i < 2000000; i++);
    }
}

void MemManage_Handler(void)
{
    while (1) {}
}

void BusFault_Handler(void)
{
    while (1) {}
}

void UsageFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void) {}

void DebugMon_Handler(void) {}

void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/* ==================== 外设中断服务处理 ==================== */

/**
  * @brief  CAN1 RX0 接收中断 (调度 CANopen 与传感器协议解析)
  */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief  DMA1 Stream1 中断 (USART3 SBUS 遥控接收 DMA)
  */
void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

/**
  * @brief  USART1 全局中断 (处理调试打印非阻塞发送完成)
  */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief  USART3 全局中断 (处理 SBUS 空闲中断)
  */
void USART3_IRQHandler(void)
{
    RC_UART_Idle_Callback(&huart3);
}

/**
  * @brief  USART6 全局中断 (处理上位机自动驾驶协议单字节解析)
  */
void USART6_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)(huart6.Instance->DR & 0xFF);
        Serial_Auto_ParseByte(ch);
    }
    HAL_UART_IRQHandler(&huart6);
}
