/*
 * CAN 底层与分发（考核自行实现）。接收 0x201~0x203，按 ID 更新对象；发送共享 0x200 帧。发送成功/ACK 不等于机械动作完成；恢复指令另有授权，离线仍禁止输出。
 */
#include "can_feedback.hpp"
#include "can.h"
#include "motor_ins.hpp"
#include "diagnostic_config.hpp"
#include "control_config.hpp"
#include "rc_status.hpp"
volatile CanDebug g_can_debug{};
namespace {
uint32_t pending_since=0; bool was_active=false;
void CheckTxCompletion() {
 const uint32_t tsr=hcan1.Instance->TSR;
 g_can_debug.tx_last_tsr=tsr;
 for(unsigned j=0;j<3;++j) {
  const unsigned shift=8*j;
  if(tsr&(1u<<shift)) {
   if(tsr&(2u<<shift))g_can_debug.tx_confirmed=g_can_debug.tx_confirmed+1;
   if(tsr&(12u<<shift)){g_can_debug.tx_fault=1;g_can_debug.tx_failures=g_can_debug.tx_failures+1;}
   hcan1.Instance->TSR=1u<<shift; // W1C only this mailbox completion group.
  }
 }
}
}
void CanFeedbackInit() {
  g_can_debug.start_status=0xffffffffu;
  g_can_debug.notification_status=0xffffffffu;
  CAN_FilterTypeDef f{};
  f.FilterBank=0; f.FilterMode=CAN_FILTERMODE_IDMASK;
  f.FilterScale=CAN_FILTERSCALE_32BIT; f.FilterFIFOAssignment=CAN_RX_FIFO0;
  f.FilterActivation=ENABLE; f.SlaveStartFilterBank=14;
  g_can_debug.bitrate=HAL_RCC_GetPCLK1Freq()/hcan1.Init.Prescaler/14;
  g_can_debug.filter_status=HAL_CAN_ConfigFilter(&hcan1,&f);
  if(g_can_debug.filter_status!=HAL_OK) return;
  g_can_debug.start_status=HAL_CAN_Start(&hcan1);
  if(g_can_debug.start_status!=HAL_OK) return;
  g_can_debug.notification_status=HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  g_can_debug.started=g_can_debug.notification_status==HAL_OK;
}
void CanFeedbackPoll() {
  // Snapshot shared feedback coherently; never transmit motor commands here.
  const auto mask=__get_PRIMASK(); __disable_irq();
  uint32_t online=0; const auto now=HAL_GetTick();
  for(unsigned i=0;i<3;++i) {
    const auto snapshot=MotorByIndex(i)->Snapshot(now);
    if(snapshot.online) online|=1u<<i;
    auto& dst=g_can_debug.motors[i]; const auto& src=snapshot.feedback;
    dst.frames=src.frames; dst.tick=src.tick; dst.angle=src.angle;
    dst.rpm=src.rpm; dst.current_raw=src.current_raw; dst.temperature=src.temperature;
  }
  g_can_debug.online_mask=online;
  __set_PRIMASK(mask);
  g_can_debug.error_code=HAL_CAN_GetError(&hcan1);
  g_can_debug.esr=hcan1.Instance->ESR;
  if(__HAL_CAN_GET_FLAG(&hcan1,CAN_FLAG_FOV0)) {
    g_can_debug.fifo_overruns=g_can_debug.fifo_overruns+1;
    hcan1.Instance->RF0R = CAN_RF0R_FOVR0; // W1C: do not release FIFO entries.
  }
}
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* can) {
  if(can!=&hcan1) return;
  // Bound ISR work even on a busy bus; pending frames retrigger the interrupt.
  for(unsigned n=0;n<3 && HAL_CAN_GetRxFifoFillLevel(can,CAN_RX_FIFO0);++n) {
    CAN_RxHeaderTypeDef h{}; uint8_t data[8]{};
    if(HAL_CAN_GetRxMessage(can,CAN_RX_FIFO0,&h,data)!=HAL_OK) {
      g_can_debug.read_errors=g_can_debug.read_errors+1; break;
    }
    g_can_debug.rx_frames=g_can_debug.rx_frames+1;
    g_can_debug.last_id=h.IDE==CAN_ID_STD?h.StdId:h.ExtId;
    g_can_debug.last_dlc=h.DLC;
    const int i=dart::MotorIndex(h.StdId,h.IDE!=CAN_ID_STD,h.RTR!=CAN_RTR_DATA);
    if(i<0) { g_can_debug.unknown_frames=g_can_debug.unknown_frames+1; continue; }
    if(!MotorByIndex(i)->Decode(data,h.DLC,HAL_GetTick()))
      g_can_debug.rejected_frames=g_can_debug.rejected_frames+1;
  }
}

void ReadMotorStates(dart::MotorState (&out)[3],uint32_t now) {
 const auto mask=__get_PRIMASK();__disable_irq();
 for(unsigned i=0;i<3;++i)out[i]=MotorByIndex(i)->Snapshot(now);
 __set_PRIMASK(mask);
}
bool CanTransmit(const dart::ControlOutput& command,bool permission) {
 if constexpr(dart::kDiagnosticMode) return false; // No command frames at all in diagnostics.
 CheckTxCompletion();
 const auto rc=ReadRcSample();const auto now=HAL_GetTick();
 // 正常输出要求双下档与内八；recovery 允许受控恢复，但仍须遥控和电机在线。
 bool active=permission && command.active && dart::config::kHardwareCalibrated &&
  dart::RcOnline(rc,now) && (command.recovery || (rc.left==2 && rc.right==2 &&
  rc.lh>.8f && rc.lv<-.8f && rc.rh<-.8f && rc.rv<-.8f)) &&
  !g_can_debug.tx_fault && !(hcan1.Instance->ESR & CAN_ESR_BOFF);
 dart::MotorState states[3];ReadMotorStates(states,now);
 for(const auto& m:states)active=active&&m.online;
 if(!g_can_debug.started)return false;
 if(!active && was_active) {
  HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
  g_can_debug.tx_aborts=g_can_debug.tx_aborts+1;
 }
 const bool pending=HAL_CAN_IsTxMessagePending(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
 if(pending) {
  g_can_debug.tx_busy=g_can_debug.tx_busy+1;
  if(uint32_t(now-pending_since)>=10) {
   HAL_CAN_AbortTxRequest(&hcan1,CAN_TX_MAILBOX0|CAN_TX_MAILBOX1|CAN_TX_MAILBOX2);
   g_can_debug.tx_fault=1;active=false;
  } else if(active) {was_active=true;return true;}
 }
 if(!(hcan1.Instance->ESR&CAN_ESR_BOFF) && !permission && !pending)g_can_debug.tx_fault=0;
 if(!HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)){g_can_debug.tx_fault=1;return false;}
 uint8_t frame[8]{};
 for(unsigned i=0;i<3;++i) {
  auto* motor=MotorByIndex(i);motor->SetInput(active?command.raw[i]:0);motor->Encode(frame);
  g_can_debug.command_raw[i]=motor->input();
 }
 CAN_TxHeaderTypeDef h{};h.StdId=0x200;h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=8;
 uint32_t mailbox=0;
 if(HAL_CAN_AddTxMessage(&hcan1,&h,frame,&mailbox)!=HAL_OK){
  g_can_debug.tx_failures=g_can_debug.tx_failures+1;g_can_debug.tx_fault=1;return false;
 }
 if(active&&(g_can_debug.command_raw[0]||g_can_debug.command_raw[1]||g_can_debug.command_raw[2]))
  g_can_debug.nonzero_tx_frames=g_can_debug.nonzero_tx_frames+1;
 pending_since=now;was_active=active;g_can_debug.tx_frames=g_can_debug.tx_frames+1;
 return active;
}
