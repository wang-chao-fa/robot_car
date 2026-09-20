/**
  ******************************************************************************
  * @file       stm32f4xx_it.h
  * @brief      系统中断与异常服务函数声明
  ******************************************************************************
  */
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* 系统核心异常 */
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);

/* 外设中断服务函数 */
void CAN1_RX0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void USART3_IRQHandler(void);
void USART6_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
