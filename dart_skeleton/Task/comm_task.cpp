#include "comm_task.hpp"
#include "rc_ins.hpp"
#include "basic_configs.hpp"
#include "iwdg.h"
#include "motor_can.hpp"
#include <cstring>

namespace {
uint8_t rx_buffer[36]{};
dart::RcSafety safety;
bool overflow = false;
bool restart_pending = false;

bool StartReceive() {
  const auto result = HAL_UARTEx_ReceiveToIdle_DMA(rc_uart_ptr, rx_buffer, sizeof(rx_buffer));
  if (result == HAL_OK) __HAL_DMA_DISABLE_IT(rc_uart_ptr->hdmarx, DMA_IT_HT);
  return result == HAL_OK;
}
void RestartReceive() {
  HAL_UART_AbortReceive(rc_uart_ptr);
  overflow = false;
  restart_pending = !StartReceive();
  if (restart_pending) safety.invalidate();
}
}

void CommTaskInit() {
  MotorCanInit();
  CreateRc();
  safety.invalidate();
  RestartReceive();
}

void CommTask() {
  // Recovery only: never feed IWDG from a timer or main loop.
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  if (restart_pending) RestartReceive();
  __set_PRIMASK(mask);
  MotorCanTask();
}

dart::RcSnapshot GetRcSnapshot() {
  const uint32_t mask = __get_PRIMASK();
  __disable_irq();
  dart::RcSnapshot snapshot{*CreateRc(), safety.online(HAL_GetTick()), safety.last_tick()};
  __set_PRIMASK(mask);
  return snapshot;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size) {
  if (uart != rc_uart_ptr) return;
  const auto event = HAL_UARTEx_GetRxEventType(uart);
  if (event == HAL_UART_RXEVENT_HT) return;
  if (event == HAL_UART_RXEVENT_TC) {
    // More than one frame without an idle boundary: reject until next idle.
    overflow = true;
    safety.invalidate();
    return;
  }
  uint8_t frame[18]{};
  const bool complete = !overflow && size == sizeof(frame);
  if (complete) std::memcpy(frame, rx_buffer, sizeof(frame));
  const bool valid = complete && safety.accept(frame, sizeof(frame), HAL_GetTick());
  if (!valid) safety.invalidate();
  if (valid) {
    CreateRc()->decode(frame);
    HAL_IWDG_Refresh(&hiwdg);
  }
  // Idle gap is the frame boundary; re-arm after both good and malformed frames.
  RestartReceive();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart) {
  if (uart != rc_uart_ptr) return;
  safety.invalidate();
  RestartReceive();
}
