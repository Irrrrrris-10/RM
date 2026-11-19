/*
 * 双同步带闭环（考核自行实现）。位置单位 mm、速度 mm/s；相对编码器通过低速堵转建立零点。位置环生成速度目标，速度环生成原始电流指令；含同步补偿与保护。
 */
#pragma once
#include "control_types.hpp"
#include "control_config.hpp"
namespace dart {
class BeltFsm {
 public:
 constexpr void Reset() { *this=BeltFsm{}; }
 // 每次找零记录起点和反馈 epoch；检测到堵转后才将当前位置确认为零。
 constexpr void BeginHome(const ControlInput& in) {
  Reset(); homing_=true;start_=in.now;
  for(unsigned j=0;j<2;++j) {origin_[j]=in.motor[j].output_angle;epoch_[j]=in.motor[j].epoch;}
 }
 // returning 选上返限速，preparation 选准备阶段响应和稳定时间。
 constexpr void SetTarget(float mm,uint32_t now,bool returning=false,bool preparation=false) {
  target_=Clamp(mm,config::kBeltMinMm,config::kBeltMaxMm);start_=now;settled_=0;
  speed_limit_=returning?config::kReturnBeltSpeed:config::kMaxBeltSpeed;
  preparation_=preparation;
 }
 constexpr void RecoveryMode(){recovery_=true;speed_limit_=config::kRecoverySpeed;
  for(unsigned j=0;j<2;++j){velocity_[j].Reset();position_[j].Reset();stall_[j]=0;}
 }
 constexpr void Update(const ControlInput& in) {
  raw_[0]=raw_[1]=0;
  sync_correction_=0;wanted_[0]=wanted_[1]=0;
  if(fault_!=Fault::None)return;
  for(unsigned j=0;j<2;++j) {
   if(!in.motor[j].online){fault_=Fault::Offline;return;}
   if(in.motor[j].epoch!=epoch_[j]){fault_=Fault::FeedbackReset;return;}
   pos_[j]=float(in.motor[j].output_angle-origin_[j])*config::kBeltMmPerRad;
   speed_[j]=in.motor[j].output_speed*config::kBeltMmPerRad;
  }
  // 回零阶段：启动宽限后，低速且电流超过阈值持续成立才确认顶端。
  if(homing_) {
   if(uint32_t(in.now-start_)>=config::kHomeTimeoutMs){fault_=Fault::HomeTimeout;return;}
   for(unsigned j=0;j<2;++j) {
    if(home_done_[j])continue;
    if(Abs(pos_[j])>config::kHomeTravelMax){fault_=Fault::HomeTravel;return;}
    const bool stopped=uint32_t(in.now-start_)>=config::kHomeGraceMs &&
      Abs(speed_[j])<config::kHomeSpeedThreshold && Abs(in.motor[j].current_amp)>=config::kHomeCurrentA;
    stall_[j]=stopped?stall_[j]+2:0;
    if(stall_[j]>=config::kHomeStallMs) {
      home_done_[j]=true;origin_[j]=in.motor[j].output_angle;pos_[j]=0;velocity_[j].Reset();
    } else raw_[j]=Clamp(home_velocity_[j].Update((recovery_?config::kRecoverySpeed:config::kHomeSpeed),speed_[j],config::kPeriod),-config::kHomeRawLimit,config::kHomeRawLimit);
   }
   if(home_done_[0]&&home_done_[1]){homing_=false;homed_=true;stall_[0]=stall_[1]=0;}
   return;
  }
  if(!homed_)return;
  if(Abs(pos_[0]-pos_[1])>config::kMaxSkewMm){fault_=Fault::Skew;return;}
  // 右减左的位置误差：右侧减去修正、左侧加上修正，修正独立限幅。
  sync_correction_=Clamp(config::kBeltSyncKp*(pos_[0]-pos_[1]),
    -config::kBeltSyncMaxSpeed,config::kBeltSyncMaxSpeed);
  bool at=true;
  for(unsigned j=0;j<2;++j) {
   if(pos_[j]<config::kBeltMinMm-5||pos_[j]>config::kBeltMaxMm+5){fault_=Fault::Travel;return;}
   const float response=preparation_?config::kPreparationResponseScale:1.0f;
   const float base=Clamp(response*position_[j].Update(target_,pos_[j],config::kPeriod),-speed_limit_,speed_limit_);
   wanted_[j]=Clamp(base+(j==0?-sync_correction_:sync_correction_),
      -speed_limit_,speed_limit_);
   raw_[j]=velocity_[j].Update(wanted_[j],speed_[j],config::kPeriod);
   // Never command farther into the calibrated upper endpoint.
   if(pos_[j]>=0 && raw_[j]>0)raw_[j]=0;
   const bool near=Abs(target_-pos_[j])<=config::kPositionTolerance && Abs(speed_[j])<=config::kSpeedTolerance;
   at=at&&near;
   stall_[j]=!near && Abs(raw_[j])>config::kBeltRawLimit*.7f && Abs(speed_[j])<1?stall_[j]+2:0;
   if(stall_[j]>=config::kJamMs){fault_=Fault::Stall;raw_[0]=raw_[1]=0;return;}
  }
  // 两侧位置误差和速度同时合格并持续足够时间，才判定到位。
  settled_=at?settled_+2:0;
  if(!IsAtPos()&&uint32_t(in.now-start_)>=(recovery_?config::kRecoveryMoveTimeoutMs:config::kMoveTimeoutMs)){fault_=Fault::MoveTimeout;raw_[0]=raw_[1]=0;}
 }
 constexpr bool Homed()const{return homed_;}
 constexpr bool HomeDone(unsigned j)const{return home_done_[j];}
 constexpr bool Homing()const{return homing_;}
 constexpr double Origin(unsigned j)const{return origin_[j];}
 constexpr uint32_t StallMs(unsigned j)const{return stall_[j];}
 constexpr float Target()const{return target_;}
 constexpr float Speed(unsigned j)const{return speed_[j];}
 constexpr float WantedSpeed(unsigned j)const{return wanted_[j];}
 constexpr float SyncCorrection()const{return sync_correction_;}
 constexpr bool IsAtPos()const{return settled_>=(preparation_?config::kPreparationSettleMs:config::kSettleMs);}
 constexpr Fault fault()const{return fault_;}
 constexpr float input(unsigned j)const{return fault_==Fault::None?raw_[j]:0;}
 constexpr float position(unsigned j)const{return pos_[j];}
 private:
 Pid home_velocity_[2]{Pid({100,100,0,1000,config::kHomeRawLimit}),Pid({100,100,0,1000,config::kHomeRawLimit})};
 static constexpr PidConfig PositionConfig(){auto c=config::kBeltPosition;c.output_limit=config::kReturnBeltSpeed;return c;}
 Pid position_[2]{Pid(PositionConfig()),Pid(PositionConfig())};
 Pid velocity_[2]{Pid(config::kBeltSpeed),Pid(config::kBeltSpeed)};
 double origin_[2]{};float pos_[2]{},speed_[2]{},raw_[2]{},target_=0;
 float wanted_[2]{},sync_correction_=0;
 float speed_limit_=config::kMaxBeltSpeed;
 bool preparation_=false,recovery_=false;
 uint32_t epoch_[2]{},stall_[2]{},start_=0,settled_=0;
 bool homing_=false,homed_=false,home_done_[2]{};Fault fault_=Fault::None;
};
}
