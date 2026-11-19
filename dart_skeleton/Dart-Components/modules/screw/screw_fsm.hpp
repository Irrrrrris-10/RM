/*
 * 丝杆保持模块（考核自行实现）。Capture 保存触发时输出轴角度；位置环与速度环维持该位置，不执行连续旋转；检查离线、坐标重置、偏差和堵转。
 */
#pragma once
#include "control_types.hpp"
#include "control_config.hpp"
namespace dart {
class ScrewFsm {
 public:
 constexpr void Reset(){*this=ScrewFsm{};}
 // 以当前输出轴角度作为本轮保持目标，并记录坐标代次。
 constexpr void Capture(const MotorState& m){Reset();target_=m.output_angle;epoch_=m.epoch;}
 constexpr float Update(const MotorState& m) {
  if(!m.online){fault_=Fault::Offline;return 0;}
  if(m.epoch!=epoch_){fault_=Fault::FeedbackReset;return 0;}
  const float error=float(target_-m.output_angle);
  if(Abs(error)>config::kScrewMaxError){fault_=Fault::ScrewTravel;return 0;}
  if(fault_!=Fault::None)return 0;
  // 位置误差经外环转换为速度目标。
  const float speed=position_.Update(error,0,config::kPeriod);
  // 速度内环根据反馈计算原始电流指令。
  const float raw=velocity_.Update(speed,m.output_speed,config::kPeriod);
  // 位置仍有误差、输出较大而速度极低时累计卡滞时间，否则归零。
  stall_=Abs(error)>.02f && Abs(raw)>config::kScrewRawLimit*.7f && Abs(m.output_speed)<.01f?stall_+2:0;
  if(stall_>=config::kJamMs){fault_=Fault::Stall;return 0;}
  return raw;
 }
 constexpr Fault fault()const{return fault_;}
 private:
 double target_=0;uint32_t epoch_=0,stall_=0;Fault fault_=Fault::None;
 Pid position_{config::kScrewPosition},velocity_{config::kScrewSpeed};
};
}
