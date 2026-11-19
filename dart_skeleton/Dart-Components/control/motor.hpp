/*
 * 电机设备类（考核自行实现）。解析反馈并转换到输出轴 rad、rad/s、A；方向同时用于反馈和指令。epoch 变化表示多圈坐标失去连续性，控制模块须停止使用旧零点。
 */
#pragma once
#include "motor_feedback.hpp"
#include "pid.hpp"
namespace dart {
constexpr float kPi=3.14159265358979323846f;
// 反馈快照：原始帧、输出轴连续角度(rad)、转速(rad/s)、电流(A)、坐标代次和在线状态。
struct MotorState {
 MotorFeedback feedback{};
 double output_angle=0;
 float output_speed=0,current_amp=0;
 uint32_t epoch=0;
 bool online=false;
};
class Motor {
 public:
 // 构造时配置ID、方向正负、减速比、满量程安培数及报文上限。
 constexpr Motor(unsigned id,int direction,float ratio,float max_amp,int raw_limit):
 id_(id),direction_(direction),ratio_(ratio),max_amp_(max_amp),raw_limit_(raw_limit){}
 // 接收完整反馈并更新内部状态；拒绝非法帧时保留上一份状态。
 constexpr bool Decode(const uint8_t* data,unsigned size,uint32_t now) {
   MotorFeedback next=state_.feedback;
   if(!DecodeMotor(next,data,size,now)) return false;
   // 数据间隔过长时不猜测丢失圈数：重置累计并递增 epoch，让上层发现坐标失效。
   if(!state_.feedback.Online(now) || uint32_t(now-state_.feedback.tick)>=10) { turns_=0; state_.epoch++; }
   else {
     // 相邻编码值跨越半圈以上视为过零，修正累计圈数；依赖反馈足够连续。
     int delta=int(next.angle)-int(state_.feedback.angle);
     if(delta>4096) --turns_; else if(delta< -4096) ++turns_;
   }
   state_.feedback=next;
   // Follow assessment's 8191 angular scale; rollover detection uses 8192 codes.
   // 先合成转子多圈角度，再按方向取号并除减速比，得到输出轴角度。
   state_.output_angle=direction_*(double(turns_)*2*kPi+double(next.angle)*2*kPi/8191)/ratio_;
   // rpm乘2π/60转换rad/s，再换算到输出轴。
   state_.output_speed=direction_*next.rpm*(2*kPi/60)/ratio_;
   // 反馈原始电流按满量程缩放到A，并应用统一方向。
   state_.current_amp=direction_*next.current_raw*max_amp_/raw_limit_;
   return true;
 }
 // 返回副本并按当前时间重算在线状态，避免调用者修改内部状态。
 constexpr MotorState Snapshot(uint32_t now) const {auto s=state_;s.online=s.feedback.Online(now);return s;}
 // 拒绝NaN/无穷等异常输入并限幅；最终转换成整数原始指令。
 constexpr void SetInput(float input) {input_=Finite(input)?int(Clamp(input,-float(raw_limit_),float(raw_limit_))):0;}
 // 每台只写共享帧中自己的两个字节；调用者先清零帧并统一发送。
 constexpr void Encode(uint8_t (&frame)[8]) const {
   if(id_<1||id_>4)return;
   const uint16_t raw=static_cast<uint16_t>(input_*direction_);
   frame[(id_-1)*2]=raw>>8;frame[(id_-1)*2+1]=raw&255;
 }
 constexpr int input()const{return input_;}
 private:
 unsigned id_;int direction_;float ratio_,max_amp_;int raw_limit_,input_=0;
 int64_t turns_=0;MotorState state_{};
};
}
