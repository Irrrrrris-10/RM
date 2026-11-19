#pragma once
#include "motor_protocol.hpp"
namespace dart {
struct CanDiagnostics {
  bool initialized = false;
  bool fault_latched = false;
  uint32_t rx_accepted = 0;
  uint32_t rx_rejected = 0;
  uint32_t tx_queued = 0;
  uint32_t tx_busy = 0;
  uint32_t errors = 0;
  uint32_t last_error = 0;
};
struct MotorSnapshot {
  DartMotorBus motors;
  CanDiagnostics diagnostics;
  uint32_t tick = 0;
};
}
bool MotorCanInit();
void MotorCanTask();
dart::MotorSnapshot GetMotorSnapshot();
