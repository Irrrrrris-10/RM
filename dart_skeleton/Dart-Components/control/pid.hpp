/*
 * PID 类（考核自行实现）。输入目标、测量值和秒单位周期；积分项已包含 Ki，积分限幅单位与输出相同。提供输出限幅与条件积分抗饱和。
 */
#pragma once
namespace dart {
constexpr float Abs(float x) { return x<0?-x:x; }
constexpr float Clamp(float x,float lo,float hi) { return x<lo?lo:(x>hi?hi:x); }
constexpr bool Finite(float x) { return x==x && x<3.4e38f && x>-3.4e38f; }
struct PidConfig { float kp,ki,kd,integral_limit,output_limit; };
class Pid {
 public:
  constexpr explicit Pid(PidConfig c={0,0,0,0,0}):c_(c){}
  // 清除积分和误差历史，下一次更新不计算微分。
  constexpr void Reset() { integral_=previous_=0; first_=true; }
  constexpr float Update(float target,float measured,float dt) {
    // 异常输入或非正周期会重置控制器并输出0。
    if(!Finite(target)||!Finite(measured)||!Finite(dt)||dt<=0) {Reset();return 0;}
    // 误差=目标-测量值，正负决定纠偏方向。
    const float error=target-measured;
    // 首次计算不做差分，避免刚启动时产生微分冲击。
    const float derivative=first_?0:(error-previous_)/dt;
    // 积分保存的是 Ki*误差*时间累计值，因此限幅直接作用于积分输出。
    const float candidate=Clamp(integral_+c_.ki*error*dt,-c_.integral_limit,c_.integral_limit);
    const float raw=c_.kp*error+candidate+c_.kd*derivative;
    // Conditional integration prevents winding farther into saturation.
    // 未饱和或积分方向有利于退出饱和时才接受新积分值。
    if(Abs(raw)<=c_.output_limit || (raw>0 && error<0) || (raw<0 && error>0)) integral_=candidate;
    // 保存误差供下一周期微分使用。
    previous_=error;first_=false;
    // 把P、I、D相加并执行最终输出限幅。
    return Clamp(c_.kp*error+integral_+c_.kd*derivative,-c_.output_limit,c_.output_limit);
  }
 private: PidConfig c_;float integral_=0,previous_=0;bool first_=true;
};
}
