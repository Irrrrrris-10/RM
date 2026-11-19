/*
 * 周期调度（考核自行实现）。TIM3 每 2 ms 读取遥控/电机快照，选择测试或正常流程，执行输出门控并更新调试结构。调试中的目标值不等于执行器实际运动。
 */
#include "diagnostic_config.hpp"
#include "control_config.hpp"
#include "iwdg.h"
#include "main_task.hpp"
#include "comm_task.hpp"
#include "rc_status.hpp"
#include "rc_diag.h"
#include "can_feedback.hpp"
#include "main_fsm.hpp"
#include "fsm_ins.hpp"
#include "motor_ins.hpp"
#include "tim.h"
#include "can.h"
volatile SafetyDebug g_safety_debug{};
struct ControlDebug { uint32_t step,fault,hardware_ready; float belt_right_mm,belt_left_mm,servo_deg; };
volatile ControlDebug g_control_debug{};
struct HomeDebug {
 uint32_t homing,homed,home_done[2],stall_ms[2],last_motion_step,fault_step,fault;
 double zero_angle[2],current_angle[2];
 float target_mm,position_mm[2],speed_mm_s[2],current_a[2];
};
volatile HomeDebug g_home_debug{};
struct BeltSyncDebug { float error_mm,correction_mm_s,target_speed_mm_s[2]; };
volatile BeltSyncDebug g_belt_sync_debug{};
void MainTaskInit() {
 CommTaskInit();
 InitServoShoot(!dart::kDiagnosticMode&&dart::config::kHardwareCalibrated);
 g_safety_debug.timer_started=HAL_TIM_Base_Start_IT(&htim3)==HAL_OK;
}
void MainTask() {
 const auto now=HAL_GetTick();const auto rc=ReadRcSample();
 dart::ControlInput in;in.now=now;ReadMotorStates(in.motor,now);
 const bool online=dart::RcOnline(rc,now);
 in.rc_online=online;
 in.permitted=online&&rc.left==2&&rc.right==2;

 // 正常发射及受控恢复统一由主状态机调度。

 in.gesture=rc.lh>.8f&&rc.lv<-.8f&&rc.rh<-.8f&&rc.rv<-.8f;
 in.hardware_ready=!dart::kDiagnosticMode&&dart::config::kHardwareCalibrated&&ServoShootReady();
 in.can_ok=g_can_debug.started&&!g_can_debug.tx_fault&&!(hcan1.Instance->ESR&CAN_ESR_BOFF);

 auto* fsm=CreateMainFsm();const auto before=fsm->sequence.step();
 auto output=fsm->Update(in,online);
 // Freeze the last homing/position snapshot on a fault, including after RC release.
 const auto after=fsm->sequence.step();
 if(before!=dart::Step::Homing&&after==dart::Step::Homing) {
  // Reissue reset immediately on a new trigger, before sending motor commands.
  ApplyServoShoot(dart::config::kServoSafeDeg,false);
  g_home_debug.fault=0;g_home_debug.fault_step=0;
 }
 if(after!=dart::Step::Standby&&after!=dart::Step::Ready&&g_home_debug.fault==0) {
  const auto& belt=fsm->sequence.belt;
  g_home_debug.homing=belt.Homing();g_home_debug.homed=belt.Homed();
  g_home_debug.target_mm=belt.Target();
  g_belt_sync_debug.error_mm=belt.position(0)-belt.position(1);
  g_belt_sync_debug.correction_mm_s=belt.SyncCorrection();
  for(unsigned j=0;j<2;++j)g_belt_sync_debug.target_speed_mm_s[j]=belt.WantedSpeed(j);
  for(unsigned j=0;j<2;++j){
   g_home_debug.home_done[j]=belt.HomeDone(j);g_home_debug.stall_ms[j]=belt.StallMs(j);
   g_home_debug.zero_angle[j]=belt.Origin(j);g_home_debug.current_angle[j]=in.motor[j].output_angle;
   g_home_debug.position_mm[j]=belt.position(j);g_home_debug.speed_mm_s[j]=belt.Speed(j);
   g_home_debug.current_a[j]=in.motor[j].current_amp;
  }
  if(after==dart::Step::Fault){
   g_home_debug.fault_step=static_cast<uint32_t>(before);
   g_home_debug.fault=static_cast<uint32_t>(fsm->sequence.fault());
  }else g_home_debug.last_motion_step=static_cast<uint32_t>(after);
 }
 if(before!=dart::Step::Homing&&fsm->sequence.step()==dart::Step::Homing)
  g_safety_debug.trigger_events=g_safety_debug.trigger_events+1;
 const bool permission=in.hardware_ready&&(output.recovery||(in.permitted&&in.gesture&&fsm->work_state()==dart::Fsm::kWorkStateWorking));
 const bool sent=CanTransmit(output,permission);
 // Recheck latest RC after computation to avoid applying a stale fire command.
 const auto latest=ReadRcSample();
 const bool fire=sent&&dart::RcOnline(latest,HAL_GetTick())&&(output.recovery||(latest.left==2&&latest.right==2&&
   latest.lh>.8f&&latest.lv<-.8f&&latest.rh<-.8f&&latest.rv<-.8f));
 ApplyServoShoot(output.servo_deg,fire);
 g_control_debug.step=static_cast<uint32_t>(fsm->sequence.step());
 g_control_debug.fault=static_cast<uint32_t>(fsm->sequence.fault());
 g_control_debug.hardware_ready=in.hardware_ready;
 g_control_debug.belt_right_mm=fsm->sequence.belt.position(0);
 g_control_debug.belt_left_mm=fsm->sequence.belt.position(1);
 g_control_debug.servo_deg=fire?output.servo_deg:dart::config::kServoSafeDeg;
 g_safety_debug.cycles=g_safety_debug.cycles+1;
 g_safety_debug.online=online;g_safety_debug.work_state=fsm->work_state();
 g_safety_debug.one_shoot=in.permitted;g_safety_debug.output_enabled=sent;
 g_safety_debug.diagnostic_mode=dart::kDiagnosticMode;
}
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer) {
 if(timer!=&htim3)return;
 MainTask();CommTask();
 if constexpr(dart::kDiagnosticMode) {HAL_IWDG_Refresh(&hiwdg);g_rc_diag.watchdog_feeds=g_rc_diag.watchdog_feeds+1;}
}
