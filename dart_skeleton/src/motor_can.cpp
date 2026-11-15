#include "motor_can.hpp"
#include "can.h"

namespace {
dart::DartMotorBus motors;
dart::CanDiagnostics diagnostics;
constexpr uint32_t kMailboxes = CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2;
// Preserve interrupt state, including when called inside TIM3.
class CriticalSection {
 public:
  CriticalSection() : mask_(__get_PRIMASK()) { __disable_irq(); }
  ~CriticalSection() { __set_PRIMASK(mask_); }
 private:
  uint32_t mask_;
};
void Fault(uint32_t error) {
  diagnostics.fault_latched = true;
  ++diagnostics.errors;
  diagnostics.last_error = error;
  motors.right.setInput(0); motors.left.setInput(0); motors.screw.setInput(0);
  if (diagnostics.initialized) HAL_CAN_AbortTxRequest(&hcan1,kMailboxes);
}
}

bool MotorCanInit() {
  // Startup only: no blocking HAL start/stop operations in periodic interrupts.
  CAN_FilterTypeDef filter{};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  // All-pass hardware filter; protocol validates IDE/RTR/DLC and IDs in software.
  if (HAL_CAN_ConfigFilter(&hcan1,&filter) != HAL_OK || HAL_CAN_Start(&hcan1) != HAL_OK) {
    Fault(HAL_CAN_GetError(&hcan1)); return false;
  }
  diagnostics.initialized = true;
  if (HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK) {
    Fault(HAL_CAN_GetError(&hcan1)); return false;
  }
  return true;
}

void MotorCanTask() {
  CriticalSection lock;
  if (!diagnostics.initialized || diagnostics.fault_latched) return;
  // SCE interrupt is not configured in the supplied CubeMX project: poll ESR.
  const uint32_t error = hcan1.Instance->ESR & (CAN_ESR_BOFF | CAN_ESR_EPVF | CAN_ESR_LEC);
  if (error != 0) { Fault(error); return; }
  // Never queue behind a stale frame or wait inside the 2 ms interrupt.
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
    ++diagnostics.tx_busy;
    HAL_CAN_AbortTxRequest(&hcan1,kMailboxes);
    return;
  }
  // Commissioning stage: explicitly zero all inputs and inhibit actuation.
  // Future control integration must recheck RC, Working, homing and motor health here.
  motors.right.setInput(0); motors.left.setInput(0); motors.screw.setInput(0);
  auto frame = motors.command(HAL_GetTick(),false);
  CAN_TxHeaderTypeDef header{};
  header.StdId = frame.id; header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA; header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;
  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(&hcan1,&header,frame.data.data(),&mailbox) == HAL_OK)
    ++diagnostics.tx_queued;
  else Fault(HAL_CAN_GetError(&hcan1));
}

dart::MotorSnapshot GetMotorSnapshot() {
  CriticalSection lock;
  return {motors,diagnostics,HAL_GetTick()};
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* can) {
  if (can != &hcan1) return;
  // FIFO depth is three; bound ISR work even if unrelated traffic is continuous.
  for (unsigned i=0;i<3 && HAL_CAN_GetRxFifoFillLevel(can,CAN_RX_FIFO0);++i) {
    CAN_RxHeaderTypeDef header{};
    dart::MotorFrame frame;
    if (HAL_CAN_GetRxMessage(can,CAN_RX_FIFO0,&header,frame.data.data()) != HAL_OK) {
      Fault(HAL_CAN_GetError(can)); return;
    }
    frame.id = header.IDE == CAN_ID_STD ? header.StdId : header.ExtId;
    frame.extended = header.IDE != CAN_ID_STD;
    frame.remote = header.RTR != CAN_RTR_DATA;
    frame.dlc = static_cast<uint8_t>(header.DLC);
    if (motors.receive(frame,HAL_GetTick())) ++diagnostics.rx_accepted;
    else ++diagnostics.rx_rejected;
  }
}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* can) {
  if (can == &hcan1) Fault(HAL_CAN_GetError(can));
}
