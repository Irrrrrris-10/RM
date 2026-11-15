#pragma once
#include "pid.hpp"
#include <limits>
#include <cstdint>
namespace dart {
class BeltController {
 public:
  enum class State { Unhomed, Homing, Holding, Moving, Fault };
  struct Config {
    float min_mm=0, max_mm=0, home_speed=0, max_speed=0, tolerance=0, settle_speed=0, home_travel=0;
    uint32_t home_timeout=0, move_timeout=0, stable_ms=0;
    Pid::Config position{}, velocity{}; // position output is mm/s; velocity output is raw current.
  };
  struct Input {
    float right_mm=0,left_mm=0,right_speed=0,left_speed=0;
    bool right_limit=false,left_limit=false;
    bool feedback_ok=false,limits_valid=false,authorized=false;
  };
  struct Output { float right=0,left=0; };
  constexpr bool configure(Config c) {
    reset(); c_=c;
    configured_=positive(c.max_speed)&&positive(c.home_speed)&&c.home_speed<=c.max_speed &&
      positive(c.tolerance)&&positive(c.settle_speed)&&positive(c.home_travel)&&
      finite(c.min_mm)&&finite(c.max_mm)&&c.min_mm<0&&c.max_mm<=0&&c.min_mm<c.max_mm &&
      c.home_timeout>0&&c.move_timeout>0&&c.stable_ms>0 &&
      pos_r_.configure(c.position)&&pos_l_.configure(c.position)&&
      vel_r_.configure(c.velocity)&&vel_l_.configure(c.velocity)&&
      c.position.output_limit>0&&c.position.output_limit<=c.max_speed &&
      c.velocity.output_limit>0&&c.velocity.output_limit<=16384;
    return configured_;
  }
  constexpr void reset() { state_=State::Unhomed; out_={}; hit_r_=hit_l_=stable_=at_pos_=false; resetPid(); }
  constexpr State state() const { return state_; }
  constexpr bool isAtPos() const { return at_pos_; }
  constexpr Output output() const { return out_; }
  constexpr bool startHome(const Input& in,uint32_t now) {
    if(!configured_ || state_!=State::Unhomed || !safe(in)) return false;
    resetPid(); start_r_=in.right_mm; start_l_=in.left_mm; started_=now;
    state_=State::Homing; return true;
  }
  constexpr bool moveTo(float mm,uint32_t now) {
    if((state_!=State::Holding && state_!=State::Moving)||!finite(mm)||mm<c_.min_mm||mm>c_.max_mm) return false;
    target_=mm; started_=now; stable_=at_pos_=false; resetPid(); state_=State::Moving; return true;
  }
  constexpr Output update(const Input& in,uint32_t now) {
    out_={};
    if(state_==State::Fault) return out_;
    if(!configured_ || !safe(in)) { reset(); return out_; }
    if(state_==State::Unhomed) return out_;
    if(state_==State::Homing) {
      if(uint32_t(now-started_)>=c_.home_timeout || abs(in.right_mm-start_r_)>c_.home_travel || abs(in.left_mm-start_l_)>c_.home_travel) return fault();
      // Stop each side immediately at its switch; only accept home when both are active together.
      if(in.right_limit) { zero_r_=in.right_mm; hit_r_=true; }
      if(in.left_limit) { zero_l_=in.left_mm; hit_l_=true; }
      if(in.right_limit&&in.left_limit) {
        if(!stable_) { stable_=true; stable_since_=now; }
        if(uint32_t(now-stable_since_)>=c_.stable_ms) {
          state_=State::Holding; target_=0; resetPid(); stable_=false;
        }
      } else stable_=false;
      if(state_==State::Homing) {
        out_.right=vel_r_.update(c_.home_speed,in.right_speed,!hit_r_);
        out_.left=vel_l_.update(c_.home_speed,in.left_speed,!hit_l_);
      }
      return out_;
    }
    const float r=in.right_mm-zero_r_,l=in.left_mm-zero_l_;
    if(r<c_.min_mm-c_.tolerance || l<c_.min_mm-c_.tolerance || r>c_.tolerance || l>c_.tolerance) return fault();
    if(state_==State::Moving && uint32_t(now-started_)>=c_.move_timeout) return fault();
    const bool close=abs(r-target_)<=c_.tolerance&&abs(l-target_)<=c_.tolerance&&
      abs(in.right_speed)<=c_.settle_speed&&abs(in.left_speed)<=c_.settle_speed;
    if(close) {
      if(!stable_) { stable_=true; stable_since_=now; }
      at_pos_=uint32_t(now-stable_since_)>=c_.stable_ms;
      if(at_pos_) state_=State::Holding;
    } else { stable_=at_pos_=false; }
    out_.right=vel_r_.update(pos_r_.update(target_,r),in.right_speed);
    out_.left=vel_l_.update(pos_l_.update(target_,l),in.left_speed);
    if(in.right_limit&&out_.right>0) { out_.right=0; pos_r_.reset(); vel_r_.reset(); }
    if(in.left_limit&&out_.left>0) { out_.left=0; pos_l_.reset(); vel_l_.reset(); }
    return out_;
  }
 private:
  static constexpr bool finite(float x) { return x==x&&x<=std::numeric_limits<float>::max()&&x>=-std::numeric_limits<float>::max(); }
  static constexpr bool positive(float x) { return finite(x)&&x>0; }
  static constexpr float abs(float x) { return x<0?-x:x; }
  static constexpr bool safe(const Input& i) { return i.authorized&&i.feedback_ok&&i.limits_valid&&finite(i.right_mm)&&finite(i.left_mm)&&finite(i.right_speed)&&finite(i.left_speed); }
  constexpr void resetPid() { pos_r_.reset();pos_l_.reset();vel_r_.reset();vel_l_.reset(); }
  constexpr Output fault() { resetPid(); state_=State::Fault; at_pos_=false; out_={}; return out_; }
  Config c_{}; bool configured_=false,hit_r_=false,hit_l_=false,stable_=false,at_pos_=false;
  State state_=State::Unhomed; Output out_{};
  uint32_t started_=0,stable_since_=0;
  float zero_r_=0,zero_l_=0,start_r_=0,start_l_=0,target_=0;
  Pid pos_r_,pos_l_,vel_r_,vel_l_;
};
}
