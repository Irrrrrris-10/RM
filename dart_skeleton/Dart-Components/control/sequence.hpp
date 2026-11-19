/*
 * 顶层单发流程（考核自行实现，另扩展受控恢复）。Tick 每 2 ms 调用。正常流程需工作档和内八；中途取消转入恢复流程。注意：安全档自动恢复会继续运动，是用户扩展，与原考核安全档立即归零要求不同。
 */
#pragma once
#include "belt_fsm.hpp"
#include "screw_fsm.hpp"
#include "shoot_fsm.hpp"
namespace dart {
// 状态编号供 Ozone 使用：0~9 为原流程；10停止、11回下拉点、12解锁、13找零、14完成、15停靠。
enum class Step:uint8_t {Standby,Ready,Homing,Positioning,Pulldown,Pullup,Shoot,Finish,WaitRelease,Fault,RecoveryStop,RecoveryDown,RecoveryRelease,RecoveryHome,RecoveryDone,RecoveryPark};
class Sequence {
 public:
 constexpr ControlOutput Tick(const ControlInput& in) {
  ControlOutput out{};
  // 恢复故障保持锁定；拨杆切换不清除，避免通信恢复后自行续动。
  if(recovery_fault_)return ControlOutput{};
  if(Recovering())return Recover(in);
  // 只有正在动作且遥控在线的主动取消才走恢复；掉线由停止路径处理。
  if(in.rc_online&&(!in.permitted||!in.gesture)&&in.hardware_ready&&step_>=Step::Homing&&step_<=Step::Finish&&fault_==Fault::None){
   for(unsigned j=0;j<3;++j)recovery_epoch_[j]=in.motor[j].epoch;
   recover_down_=step_==Step::Pullup; released_=false; armed_=false;
   shoot.Reset();step_=Step::RecoveryStop;recovery_start_=stop_start_=in.now;stop_stable_=0;
   return Recover(in);
  }
  // 正常流程要求拨杆许可、Working状态和硬件许可同时满足。
  const bool enabled=in.permitted&&in.working&&in.hardware_ready;
  if(!enabled){Stop();armed_=false;return out;}
  // Fault latch only clears by leaving the operating mode / going offline.
  if(fault_!=Fault::None){step_=Step::Fault;return out;}
  // 待机松开手势才重新武装；活动阶段松手已由上面的恢复分支截获。
  if(!in.gesture){Stop();armed_=true;step_=Step::Ready;return out;}
  if(step_==Step::Standby||step_==Step::Ready) {
   // 防止上电时一直保持内八就自动发射。
   if(!armed_)return out;
   armed_=false;
   if(!AllOnline(in)||!in.can_ok){Fail(!AllOnline(in)?Fault::Offline:Fault::Can);return out;}
   shoot.Reset(); // Every accepted new cycle begins with the safe servo angle.
   // 记录本轮起点，同时初始化找零并捕获丝杆保持位置。
   shot_start_=in.now;belt.BeginHome(in);screw.Capture(in.motor[2]);step_=Step::Homing;
  }
  if(step_==Step::WaitRelease)return out;
  if(!AllOnline(in)||!in.can_ok){Fail(!AllOnline(in)?Fault::Offline:Fault::Can);return out;}
  if(uint32_t(in.now-shot_start_)>=config::kShotTimeoutMs){Fail(Fault::SequenceTimeout);return out;}
  // 先计算两侧同步带闭环，再计算丝杆保持；出现任一模块故障则整组零输出。
  belt.Update(in);
  out.raw[2]=screw.Update(in.motor[2]);
  if(belt.fault()!=Fault::None||screw.fault()!=Fault::None) {
   Fail(belt.fault()!=Fault::None?belt.fault():screw.fault());return ControlOutput{};
  }
  out.raw[0]=belt.input(0);out.raw[1]=belt.input(1);out.active=true;
  // 每步仅在到位或计时条件满足后切换，不用阻塞延时。
  switch(step_) {
   // 回零结束后先到准备位置，不立即击发。
   case Step::Homing:
    if(belt.Homed()){belt.SetTarget(config::kUpMm,in.now,false,true);step_=Step::Positioning;}break;
   // 准备到位后才允许下拉。
   case Step::Positioning:
    if(belt.IsAtPos()){belt.SetTarget(config::kDownMm,in.now);step_=Step::Pulldown;}break;
   // 两侧下拉到位并稳定后，切换正常上返目标。
   case Step::Pulldown:
    if(belt.IsAtPos()){belt.SetTarget(config::kUpMm,in.now,true);step_=Step::Pullup;}break;
   // 上返到位后启动舵机击发计时。
   case Step::Pullup:
    if(belt.IsAtPos()){shoot.Fire(in.now);step_=Step::Shoot;}break;
   // 击发保持时间到后请求舵机复位，进入收尾等待。
   case Step::Shoot:
    if(shoot.Done(in.now)){shoot.Reset();finish_start_=in.now;step_=Step::Finish;}break;
   // 收尾完成后零电流等待松手，保持内八不会重复发射。
   case Step::Finish:
    if(uint32_t(in.now-finish_start_)>=config::kServoResetMs){step_=Step::WaitRelease;out=ControlOutput{};}break;
   default:break;
  }
  out.servo_deg=shoot.angle();return out;
 }
 constexpr bool Recovering()const{return step_>=Step::RecoveryStop;}
 constexpr ControlOutput Recover(const ControlInput& in){
  ControlOutput out{};
  // 完成后必须安全档且松开手势，才允许退出等待、重新使能。
  if(step_==Step::RecoveryDone){
   if(in.rc_online&&!in.permitted&&!in.gesture){Stop();armed_=false;}
   return out;
  }
  if(!in.rc_online||!in.hardware_ready||!AllOnline(in)||!in.can_ok){
   recovery_fault_=true;Fail(!AllOnline(in)||!in.rc_online?Fault::Offline:(!in.can_ok?Fault::Can:Fault::Servo));return out;
  }
  // 恢复途中任何电机坐标代次改变，都不能再沿用旧坐标，锁定故障。
  for(unsigned j=0;j<3;++j)if(in.motor[j].epoch!=recovery_epoch_[j]){recovery_fault_=true;Fail(Fault::FeedbackReset);return out;}
  if(uint32_t(in.now-recovery_start_)>=config::kShotTimeoutMs){recovery_fault_=true;Fail(Fault::SequenceTimeout);return out;}
  // 先请求零电流，再确认反馈速度稳定降低；零电流不是机械制动。
  if(step_==Step::RecoveryStop){
   bool stopped=true;for(unsigned j=0;j<2;++j)stopped=stopped&&Abs(in.motor[j].output_speed*config::kBeltMmPerRad)<=config::kSpeedTolerance;
   // 每次满足低速条件累计2ms，任何一次不满足就清零累计。
   stop_stable_=stopped?stop_stable_+2:0;
   if(stop_stable_>=config::kSettleMs){
    if(recover_down_){belt.SetTarget(config::kDownMm,in.now);belt.RecoveryMode();step_=Step::RecoveryDown;}
    else if(belt.Homed()){belt.SetTarget(config::kUpMm,in.now);belt.RecoveryMode();step_=Step::RecoveryPark;}
    else {belt.BeginHome(in);belt.RecoveryMode();step_=Step::RecoveryHome;}
   }else if(uint32_t(in.now-stop_start_)>=config::kRecoveryStopTimeoutMs){recovery_fault_=true;Fail(Fault::MoveTimeout);}
   return out;
  }
  // 先计算两侧同步带闭环，再计算丝杆保持；出现任一模块故障则整组零输出。
  belt.Update(in);out.raw[2]=screw.Update(in.motor[2]);
  if(belt.fault()!=Fault::None||screw.fault()!=Fault::None){recovery_fault_=true;Fail(belt.fault()!=Fault::None?belt.fault():screw.fault());return ControlOutput{};}
  out.raw[0]=belt.input(0);out.raw[1]=belt.input(1);out.active=true;out.recovery=true;
  // 回到下拉点后才解锁，不能在返回途中提前解锁。
  if(step_==Step::RecoveryDown&&belt.IsAtPos()){shoot.Fire(in.now);step_=Step::RecoveryRelease;released_=true;}
  // 解锁等待结束后保持释放状态，按已标定坐标返回-5mm。
  else if(step_==Step::RecoveryRelease&&shoot.Done(in.now)){belt.SetTarget(config::kUpMm,in.now);belt.RecoveryMode();step_=Step::RecoveryPark;}
  // 只有原本没有零点时才先堵转找零，然后退到-5mm。
  else if(step_==Step::RecoveryHome&&belt.Homed()){
   shoot.Reset();released_=false;belt.SetTarget(config::kUpMm,in.now);belt.RecoveryMode();step_=Step::RecoveryPark;return ControlOutput{};
  }
  // 停靠到位才复位舵机、进入完成等待，并立即返回零电流输出。
  else if(step_==Step::RecoveryPark&&belt.IsAtPos()){shoot.Reset();step_=Step::RecoveryDone;return ControlOutput{};}
  // 解锁后保持释放角度至返回 -5 mm 到位，避免途中重新锁住机构。
  out.servo_deg=released_?config::kServoFireDeg:config::kServoSafeDeg;return out;
 }
 constexpr Step step()const{return step_;}
 constexpr Fault fault()const{return fault_;}
 constexpr void Fail(Fault f){fault_=f;step_=Step::Fault;shoot.Reset();}
 // 重置普通流程和模块；不会清除恢复故障锁存 recovery_fault_。
 constexpr void Stop(){belt.Reset();screw.Reset();shoot.Reset();step_=Step::Standby;fault_=Fault::None;}
 BeltFsm belt; ScrewFsm screw; ShootFsm shoot;
 private:Step step_=Step::Standby;Fault fault_=Fault::None;bool armed_=false;
 bool recover_down_=false,released_=false,recovery_fault_=false;
 uint32_t recovery_epoch_[3]{};
 uint32_t recovery_start_=0,stop_start_=0,stop_stable_=0;
 uint32_t shot_start_=0,finish_start_=0;
};
}
