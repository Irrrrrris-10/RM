/*
 * 通信接线与看门狗（考核自行实现）。USART1 空闲/DMA 收 18 字节，校验后调用已有 DT7 解码；异常重启接收。正式模式喂狗依赖有效遥控帧和控制周期推进。
 */
#include "diagnostic_config.hpp"
#include "can_feedback.hpp"
#include "comm_task.hpp"
#include "rc_diag.h"
#include "rc_ins.hpp"
#include "rc_status.hpp"
#include "usart.h"
#include "iwdg.h"
volatile RcDebug g_rc_debug{};
namespace {
// One DBUS frame. Normal DMA stops at 18 bytes; both TC and IDLE are handled.
uint8_t rx[18];
dart::RcSample sample;
uint32_t last_fed_cycle = 0;
volatile bool restart_pending = false;
// 启动普通DMA接收18字节并启用空闲事件；失败会挂起重试。
void StartRx() {
  g_rc_diag.start_attempts = g_rc_diag.start_attempts + 1;
  const auto status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx, sizeof(rx));
  g_rc_diag.start_status = status;
  if (status == HAL_OK) {
    // 不处理半帧，关闭DMA半传输中断。
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    restart_pending = false;
  } else {
    restart_pending = true;
    g_rc_debug.restart_errors = g_rc_debug.restart_errors + 1;
  }
}
// 异步终止当前接收；完成回调再重新启动，避免并发重复启动DMA。
void RestartRx() {
  const auto status = HAL_UART_AbortReceive_IT(&huart1);
  g_rc_diag.abort_status = status;
  if (status != HAL_OK) {
    restart_pending = true;
    g_rc_debug.restart_errors = g_rc_debug.restart_errors + 1;
    return;
  }
}
}
// 短临界区复制整帧遥控数据，避免中断更新一半时被读到。
dart::RcSample ReadRcSample() {
  const auto mask = __get_PRIMASK();
  __disable_irq();
  const auto result = sample;
  // 恢复进入函数前的中断状态，不是无条件开启中断。
  __set_PRIMASK(mask);
  return result;
}
void CommTaskInit() {
  g_rc_diag.pclk2_hz = HAL_RCC_GetPCLK2Freq();
  // Configure before reception; generated Core files remain untouched.
  if(huart1.hdmarx->Init.Mode!=DMA_NORMAL) {
    HAL_DMA_DeInit(huart1.hdmarx);
    huart1.hdmarx->Init.Mode=DMA_NORMAL;
    if(HAL_DMA_Init(huart1.hdmarx)!=HAL_OK) {
      g_rc_debug.restart_errors=g_rc_debug.restart_errors+1;
      g_rc_diag.start_status=HAL_ERROR;
      CanFeedbackInit();return;
    }
  }
  CreateRc(); StartRx();
  CanFeedbackInit();
}
void CommTask() {
  // No actuator output until drivers and protections are implemented.
  CanFeedbackPoll();
  RcDiagSample();
  if (g_rc_diag.ndtr < g_rc_diag.ndtr_min) g_rc_diag.ndtr_min = g_rc_diag.ndtr;
  // Diagnostic sampling only; DMA may advance during copying, so record both counters.
  for (unsigned i = 0; i < sizeof(g_rc_diag.sampled_raw); ++i) g_rc_diag.sampled_raw[i] = i < sizeof(rx) ? rx[i] : 0;
  g_rc_diag.sample_ndtr_after = huart1.hdmarx->Instance->NDTR;
  if (restart_pending) RestartRx();
}
// HAL接收回调采用C链接；只处理USART1，记录事件再校验完整数据。
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size) {
  if (uart != &huart1) return;
  const auto event = HAL_UARTEx_GetRxEventType(uart);
  g_rc_diag.event_tick = HAL_GetTick();
  g_rc_diag.event_type = event;
  g_rc_diag.event_size = size;
  const unsigned count = size < sizeof(rx) ? size : sizeof(rx);
  for (unsigned i = 0; i < sizeof(g_rc_diag.raw); ++i) g_rc_diag.raw[i] = i < count ? rx[i] : 0;
  g_rc_diag.raw_size = count;
  if (event == HAL_UART_RXEVENT_HT) {
    g_rc_diag.ht_events = g_rc_diag.ht_events + 1;
    return;
  }
  if (event == HAL_UART_RXEVENT_TC) {
    g_rc_diag.tc_events = g_rc_diag.tc_events + 1;
  } else {
    g_rc_diag.idle_events = g_rc_diag.idle_events + 1;
  }
  g_rc_debug.last_size = size;
  g_rc_diag.reject_reason = dart::DbusRejectReason(rx, size);
  // 先做18字节合理性检查，合法后才调用骨架DT7解码。
  if (dart::ValidDbus(rx, size)) {
    g_rc_diag.valid_frames = g_rc_diag.valid_frames + 1;
    auto* rc = CreateRc();
    rc->decode(rx);
    // 发布解码值及接收时刻，供控制任务判断手势、拨杆和超时。
    sample = {true, HAL_GetTick(), rc->rc_lh(), rc->rc_lv(), rc->rc_rh(),
              rc->rc_rv(), rc->rc_wheel(), static_cast<uint8_t>(rc->rc_l_switch()),
              static_cast<uint8_t>(rc->rc_r_switch())};
    g_rc_debug.valid_frames = g_rc_debug.valid_frames + 1;
    g_rc_debug.last_valid_tick = sample.tick;
    g_rc_debug.lh = sample.lh; g_rc_debug.lv = sample.lv;
    g_rc_debug.rh = sample.rh; g_rc_debug.rv = sample.rv;
    g_rc_debug.wheel = sample.wheel;
    g_rc_debug.left = sample.left; g_rc_debug.right = sample.right;
    // Both valid reception and a progressing control scheduler are required.
    const auto cycle = g_safety_debug.cycles;
    // 正式模式必须有有效遥控帧且控制周期在推进，才刷新看门狗。
    if (!dart::kDiagnosticMode && cycle != last_fed_cycle) {
      HAL_IWDG_Refresh(&hiwdg);
      g_rc_diag.watchdog_feeds = g_rc_diag.watchdog_feeds + 1;
      last_fed_cycle = cycle;
    }
  } else {
    sample.valid = false;
    g_rc_diag.invalid_frames = g_rc_diag.invalid_frames + 1;
    g_rc_debug.invalid_frames = g_rc_debug.invalid_frames + 1;
  }
  RestartRx();
}
// 串口出错立即使采样无效，记录错误并重启接收。
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart) {
  if (uart != &huart1) return;
  sample.valid = false;
  g_rc_diag.error_events = g_rc_diag.error_events + 1;
  g_rc_diag.last_uart_error = uart->ErrorCode;
  g_rc_debug.uart_errors = g_rc_debug.uart_errors + 1;
  RestartRx();
}

// 接收终止完成后启动下一轮DMA。
extern "C" void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* uart) {
  if (uart == &huart1) {
    g_rc_diag.abort_completions = g_rc_diag.abort_completions + 1;
    StartRx();
  }
}
