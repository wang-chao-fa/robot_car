/**
  ******************************************************************************
  * @file       rc_sbus.c
  * @brief      S.BUS 遥控接收解析模块
  *             - 通过 USART3 DMA + 空闲中断接收 S.BUS 数据
  *             - 解析 25 字节帧为 16 个 11位通道值
  *             - 提供失控保护标志 (超时清除)
  ******************************************************************************
  */
#include "rc_sbus.h"

uint16_t rc_channels[16];
uint8_t  sbus_rx_buf[SBUS_RX_BUF_NUM];

volatile uint8_t  sbus_updated  = 0;
volatile uint32_t sbus_last_time = 0;

/**
  * @brief  解析标准 S.BUS / W.BUS 协议数据 (25字节)
  *         帧头: 0x0F, 帧尾: 0x00
  *         16通道，每通道11位，连续紧密排列
  * @param  sbus_buf: 指向 25 字节的原始接收缓冲区
  */
void SBUS_Parse(uint8_t *sbus_buf)
{
    /* S.BUS 帧头校验 */
    if (sbus_buf[0] != 0x0F)
    {
        return;
    }

    /* 11-bit 通道解包 (小端位序) */
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

    /* 更新时间戳和有效标志 */
    sbus_last_time = HAL_GetTick();
    sbus_updated   = 1;
}

/**
  * @brief  串口空闲中断回调 (在 USART3_IRQHandler 中调用)
  *         检测到空闲中断 → 停 DMA → 解析 → 重启 DMA
  */
void RC_UART_Idle_Callback(UART_HandleTypeDef *huart)
{
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        HAL_UART_DMAStop(huart);

        uint32_t recv_len = SBUS_RX_BUF_NUM - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        if (recv_len == 25)
        {
            SBUS_Parse(sbus_rx_buf);
        }

        HAL_UART_Receive_DMA(huart, sbus_rx_buf, SBUS_RX_BUF_NUM);
    }
}

/**
  * @brief  将通道原始值 (172~1811) 映射为 -1000 ~ +1000
  *         中位 1024 映射为 0
  * @param  ch_index: 通道索引 0~15
  * @retval 映射后的值 -1000 ~ +1000
  */
int16_t SBUS_GetChannel_Mapped(uint8_t ch_index)
{
    if (ch_index >= 16) return 0;

    /* 原始范围: 172~1811, 中位: 1024, 单边幅度约 820 */
    int32_t raw = (int32_t)rc_channels[ch_index] - 1024;

    /* 死区: 消除摇杆中位抖动 (±20) */
    if (raw > -20 && raw < 20) raw = 0;

    /* 缩放到 ±1000 */
    int32_t mapped = raw * 1000 / 820;

    /* 限幅 */
    if (mapped >  1000) mapped =  1000;
    if (mapped < -1000) mapped = -1000;

    return (int16_t)mapped;
}
