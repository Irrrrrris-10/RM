/*
 * 遥控诊断实现（新增）。采样寄存器并保留复位前信息，帮助区分没收到数据、接收错误与看门狗重启。诊断本身不授权执行器运动。
 */
#include "rc_diag.h"
#include "usart.h"
#include "rc_status.hpp"
// NOLOAD and outside startup BSS clearing: best-effort evidence across warm resets.
volatile RcDiag g_rc_diag __attribute__((section(".noinit.rc_diag"), used));
volatile RcDiag g_rc_previous{};
extern "C" void RcDiagBoot() {
  const uint32_t flags = RCC->CSR;
  const bool retained = g_rc_diag.magic == 0x52434403u &&
      !(flags & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF));
  uint32_t boots = 1;
  if (retained) {
    boots = g_rc_diag.boot_count + 1;
    auto* dst = reinterpret_cast<volatile uint8_t*>(&g_rc_previous);
    auto* src = reinterpret_cast<volatile uint8_t*>(&g_rc_diag);
    for (unsigned i = 0; i < sizeof(RcDiag); ++i) dst[i] = src[i];
  }
  auto* dst = reinterpret_cast<volatile uint8_t*>(&g_rc_diag);
  for (unsigned i = 0; i < sizeof(RcDiag); ++i) dst[i] = 0;
  g_rc_diag.ndtr_min = 18;
  g_rc_diag.boot_count = boots;
  g_rc_diag.previous_available = retained;
  g_rc_diag.reset_csr = flags;
  g_rc_diag.reset_iwdg = (flags & RCC_CSR_IWDGRSTF) != 0;
  g_rc_diag.magic = 0x52434403u;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  // Halted debugging must not itself cause watchdog resets; running protection stays enabled.
  __HAL_DBGMCU_FREEZE_IWDG();
}
extern "C" void RcDiagUartIrq() {
  g_rc_diag.uart_irq = g_rc_diag.uart_irq + 1;
  g_rc_diag.irq_uart_sr = USART1->SR; // Never read DR: that would consume UART data.
}
extern "C" void RcDiagDmaIrq() {
  g_rc_diag.dma_irq = g_rc_diag.dma_irq + 1;
  g_rc_diag.irq_dma_lisr = DMA2->LISR;
}
extern "C" void RcDiagSample() {
  g_rc_diag.sample_tick = HAL_GetTick();
  g_rc_diag.cycles = g_safety_debug.cycles;
  g_rc_diag.ndtr = DMA2_Stream2->NDTR;
  g_rc_diag.dma_cr = DMA2_Stream2->CR;
  g_rc_diag.uart_sr = USART1->SR;
  g_rc_diag.uart_cr1 = USART1->CR1;
  g_rc_diag.uart_cr3 = USART1->CR3;
  g_rc_diag.uart_brr = USART1->BRR;
  g_rc_diag.gpio_moder = GPIOB->MODER;
  g_rc_diag.gpio_afrl = GPIOB->AFR[0];
  g_rc_diag.gpio_idr = GPIOB->IDR;
  g_rc_diag.uart_irq_enabled = NVIC_GetEnableIRQ(USART1_IRQn);
  g_rc_diag.dma_irq_enabled = NVIC_GetEnableIRQ(DMA2_Stream2_IRQn);
}
