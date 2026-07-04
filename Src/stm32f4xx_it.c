/**
  ******************************************************************************
  * @file       stm32f4xx_it.c
  * @brief      中断服务程序 (robot_car)
  *             整合了 CAN1 接收中断 和 USART3 SBUS 空闲中断
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_it.h"
#include "usart.h"
#include "rc_sbus.h"

extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart3_rx;

/* ==================== Cortex-M4 核心异常处理 ======================== */

void NMI_Handler(void) {}

void HardFault_Handler(void)
{
    while (1) {}
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

/* ==================== 外设中断处理 ================================== */

/**
  * @brief  CAN1 RX0 中断 → 接收步科伺服电机 CANopen 反馈
  */
void CAN1_RX0_IRQHandler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/**
  * @brief  DMA1 Stream1 中断 → USART3 DMA 接收完成
  */
void DMA1_Stream1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

/**
  * @brief  USART3 全局中断 → S.BUS 空闲中断解析
  *         空闲中断触发后，调用 RC_UART_Idle_Callback 解析 S.BUS 帧
  */
void USART3_IRQHandler(void)
{
    RC_UART_Idle_Callback(&huart3);
    HAL_UART_IRQHandler(&huart3);
}
