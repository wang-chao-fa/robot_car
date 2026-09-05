/**
  ******************************************************************************
  * @file       usart.c
  * @brief      USART 初始化
  *             USART1: 调试输出 (115200 8N1, TX=PA9, RX=PB7)
  *             USART3: SBUS 遥控接收 (100000 9bit 偶校验, RX=PC11, DMA)
  ******************************************************************************
  */

#include "usart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef  hdma_usart3_rx;

/* ROS 下行控制命令全局变量 ($CMD,v,w) */
volatile float    g_ros_cmd_v         = 0.0f;
volatile float    g_ros_cmd_w         = 0.0f;
volatile uint32_t g_ros_cmd_last_time = 0;
volatile uint8_t  g_ros_cmd_updated   = 0;

/* USART1 init: 115200, 8N1, 调试打印 */
void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART3 init: 100000 baud, 9bit, 偶校验, S.BUS协议 */
void MX_USART3_UART_Init(void)
{
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 100000;
    huart3.Init.WordLength   = UART_WORDLENGTH_9B;
    huart3.Init.StopBits     = UART_STOPBITS_2;     // S.BUS: 2个停止位
    huart3.Init.Parity       = UART_PARITY_EVEN;
    huart3.Init.Mode         = UART_MODE_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USART6 init: 115200 baud, 8bit, 无校验, ROS 通信串口 (TX: PG14, RX: PG9) */
void MX_USART6_UART_Init(void)
{
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 115200;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief  UART MSP 初始化: GPIO, DMA, 中断
  */
void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* USART1: PB7=RX, PA9=TX */
        GPIO_InitStruct.Pin       = GPIO_PIN_7;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_9;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if (uartHandle->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* USART3: PC11=RX */
        GPIO_InitStruct.Pin       = GPIO_PIN_11;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* DMA1 Stream1 Channel4 -> USART3_RX */
        __HAL_RCC_DMA1_CLK_ENABLE();
        hdma_usart3_rx.Instance                 = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode                = DMA_NORMAL;
        hdma_usart3_rx.Init.Priority            = DMA_PRIORITY_VERY_HIGH;
        hdma_usart3_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
        {
            Error_Handler();
        }
        __HAL_LINKDMA(uartHandle, hdmarx, hdma_usart3_rx);

        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

        HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if (uartHandle->Instance == USART6)
    {
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOG_CLK_ENABLE();

        /* USART6: PG14=TX, PG9=RX (RoboMaster C型板标准引脚) */
        GPIO_InitStruct.Pin       = GPIO_PIN_14 | GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART6_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_7);
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
    }
    else if (uartHandle->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_11);
        HAL_DMA_DeInit(uartHandle->hdmarx);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
    else if (uartHandle->Instance == USART6)
    {
        __HAL_RCC_USART6_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOG, GPIO_PIN_14 | GPIO_PIN_9);
        HAL_NVIC_DisableIRQ(USART6_IRQn);
    }
}

#include <stdio.h>
#include <string.h>

/**
  * @brief  USART6 中断回调: 解析 ROS 发送的 $CMD,<v>,<w>\r\n 命令
  */
void ROS_UART_Rx_Callback(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)(huart->Instance->DR & 0xFF);
        static char rx_buf[64];
        static uint8_t rx_idx = 0;

        if (ch == '\n' || ch == '\r')
        {
            if (rx_idx > 0)
            {
                rx_buf[rx_idx] = '\0';
                float v_tmp = 0.0f, w_tmp = 0.0f;
                if (sscanf(rx_buf, "$CMD,%f,%f", &v_tmp, &w_tmp) == 2)
                {
                    g_ros_cmd_v         = v_tmp;
                    g_ros_cmd_w         = w_tmp;
                    g_ros_cmd_last_time = HAL_GetTick();
                    g_ros_cmd_updated   = 1;
                }
                rx_idx = 0;
            }
        }
        else
        {
            if (rx_idx < sizeof(rx_buf) - 1)
            {
                rx_buf[rx_idx++] = ch;
            }
            else
            {
                rx_idx = 0; // 缓冲区溢出重置
            }
        }
    }
}
