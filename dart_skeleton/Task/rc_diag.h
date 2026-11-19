/*
 * 遥控诊断数据声明（新增）。记录启动、UART/DMA 事件、寄存器和复位原因；诊断快照不保证与当前硬件寄存器实时一致。
 */
#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  uint32_t magic, boot_count, previous_available, reset_csr, reset_iwdg;
  uint32_t uart_irq, dma_irq, idle_events, tc_events, ht_events;
  uint32_t valid_frames, invalid_frames, error_events, last_uart_error;
  uint32_t start_attempts, start_status, abort_status, abort_completions;
  uint32_t event_tick, event_type, event_size, raw_size, reject_reason;
  uint32_t irq_uart_sr, irq_dma_lisr;
  uint32_t sample_tick, cycles, ndtr, dma_cr, uart_sr, uart_cr1, uart_cr3, uart_brr;
  uint32_t gpio_moder, gpio_afrl, gpio_idr, pclk2_hz, uart_irq_enabled, dma_irq_enabled;
  uint32_t watchdog_feeds, ndtr_min, sample_ndtr_after;
  uint8_t raw[36];
  uint8_t sampled_raw[36];
} RcDiag;
extern volatile RcDiag g_rc_diag;
extern volatile RcDiag g_rc_previous;
void RcDiagBoot(void);
void RcDiagUartIrq(void);
void RcDiagDmaIrq(void);
void RcDiagSample(void);
#ifdef __cplusplus
}
#endif
