/**
  ******************************************************************************
  * @file       rc_sbus.c
  * @brief      S.BUS 遥控器协议解析实现
  ******************************************************************************
  */
#include "rc_sbus.h"
#include <string.h>

uint16_t rc_channels[16] = {1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024, 1024};
uint8_t  sbus_rx_buf[SBUS_RX_BUF_NUM];

volatile uint8_t  sbus_updated   = 0;
volatile uint32_t sbus_last_time = 0;

void SBUS_Parse(uint8_t *sbus_buf)
{
    if (sbus_buf == NULL) return;
    if (sbus_buf[0] != 0x0F) return;

    rc_channels[0]  = ((sbus_buf[1]       | sbus_buf[2]  << 8)  & 0x07FF);
    rc_channels[1]  = ((sbus_buf[2]  >> 3 | sbus_buf[3]  << 5)  & 0x07FF);
    rc_channels[2]  = ((sbus_buf[3]  >> 6 | sbus_buf[4]  << 2   | sbus_buf[5]  << 10) & 0x07FF);
    rc_channels[3]  = ((sbus_buf[5]  >> 1 | sbus_buf[6]  << 7)  & 0x07FF);
    rc_channels[4]  = ((sbus_buf[6]  >> 4 | sbus_buf[7]  << 4)  & 0x07FF);
    rc_channels[5]  = ((sbus_buf[7]  >> 7 | sbus_buf[8]  << 1   | sbus_buf[9]  << 9)  & 0x07FF);
    rc_channels[6]  = ((sbus_buf[9]  >> 2 | sbus_buf[10] << 6)  & 0x07FF);
    rc_channels[7]  = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3)  & 0x07FF);
    rc_channels[8]  = ((sbus_buf[12]      | sbus_buf[13] << 8)  & 0x07FF);
    rc_channels[9]  = ((sbus_buf[13] >> 3 | sbus_buf[14] << 5)  & 0x07FF);
    rc_channels[10] = ((sbus_buf[14] >> 6 | sbus_buf[15] << 2   | sbus_buf[16] << 10) & 0x07FF);
    rc_channels[11] = ((sbus_buf[16] >> 1 | sbus_buf[17] << 7)  & 0x07FF);
    rc_channels[12] = ((sbus_buf[17] >> 4 | sbus_buf[18] << 4)  & 0x07FF);
    rc_channels[13] = ((sbus_buf[18] >> 7 | sbus_buf[19] << 1   | sbus_buf[20] << 9)  & 0x07FF);
    rc_channels[14] = ((sbus_buf[20] >> 2 | sbus_buf[21] << 6)  & 0x07FF);
    rc_channels[15] = ((sbus_buf[21] >> 5 | sbus_buf[22] << 3)  & 0x07FF);

    sbus_last_time = HAL_GetTick();
    sbus_updated   = 1;
}

void RC_UART_Idle_Callback(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return;

    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        uint32_t remain = __HAL_DMA_GET_COUNTER(huart->hdmarx);
        HAL_UART_DMAStop(huart);

        uint32_t recv_len = (remain <= SBUS_RX_BUF_NUM) ? (SBUS_RX_BUF_NUM - remain) : 0;

        if (recv_len >= 25)
        {
            for (uint32_t i = 0; i <= recv_len - 25; i++)
            {
                if (sbus_rx_buf[i] == 0x0F)
                {
                    SBUS_Parse(&sbus_rx_buf[i]);
                    break;
                }
            }
        }

        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        HAL_UART_Receive_DMA(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
    }
    else
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) || __HAL_UART_GET_FLAG(huart, UART_FLAG_NE) || 
            __HAL_UART_GET_FLAG(huart, UART_FLAG_FE)  || __HAL_UART_GET_FLAG(huart, UART_FLAG_PE))
        {
            __HAL_UART_CLEAR_OREFLAG(huart);
            __HAL_UART_CLEAR_NEFLAG(huart);
            __HAL_UART_CLEAR_FEFLAG(huart);
            __HAL_UART_CLEAR_PEFLAG(huart);
            HAL_UART_DMAStop(huart);
            HAL_UART_Receive_DMA(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
        }
    }
}

int16_t SBUS_GetChannel_Mapped(uint8_t ch_index)
{
    if (ch_index >= 16) return 0;
    int32_t val = (int32_t)rc_channels[ch_index] - 1024;
    int32_t mapped = (val * 1000) / 820;

    if (mapped > 1000)  mapped = 1000;
    if (mapped < -1000) mapped = -1000;
    if (mapped > -20 && mapped < 20) mapped = 0; // 消除摇杆抖动死区

    return (int16_t)mapped;
}
