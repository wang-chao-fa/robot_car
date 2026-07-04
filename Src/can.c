/**
  ******************************************************************************
  * @file       can.c
  * @brief      CAN1 初始化 (500 kbps，用于步科 CANopen 伺服通信)
  *
  * 波特率计算 (APB1 = 42 MHz):
  *   BaudRate = APB1 / Prescaler / (1 + BS1 + BS2)
  *            = 42,000,000 / 6 / (1 + 10 + 3)
  *            = 500,000 bps = 500 kbps
  * 注: 步科驱动器上位机 CAN波特率参数 "50 DEC" 即代表 500kbps
  *
  * 硬件引脚: CAN1_RX = PD0, CAN1_TX = PD1
  ******************************************************************************
  */

#include "can.h"

CAN_HandleTypeDef hcan1;

/**
  * @brief  CAN1 初始化: 500 kbps，标准帧，自动重传，接收FIFO0
  */
void MX_CAN1_Init(void)
{
    hcan1.Instance                  = CAN1;
    hcan1.Init.Prescaler            = 6;              // 42MHz / 6 / 14 = 500kbps
    hcan1.Init.Mode                 = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth        = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1             = CAN_BS1_10TQ;   // BS1 = 10 TQ
    hcan1.Init.TimeSeg2             = CAN_BS2_3TQ;    // BS2 = 3 TQ
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = ENABLE;         // 自动重传，提高可靠性
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  CAN MSP 初始化: 配置 GPIO、时钟和中断
  *         CAN1: PD0 (RX), PD1 (TX)
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef *canHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (canHandle->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* CAN1 GPIO: PD0 = RX, PD1 = TX */
        GPIO_InitStruct.Pin       = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* CAN1 RX0 中断 (最高优先级，确保电机响应及时) */
        HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef *canHandle)
{
    if (canHandle->Instance == CAN1)
    {
        __HAL_RCC_CAN1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0 | GPIO_PIN_1);
        HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    }
}
