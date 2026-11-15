#include "belt_controller.hpp"
using B=dart::BeltController;
constexpr B::Config Config() {
  return {-700,-5,10,100,1,2,100,1000,5000,10,
    {2,0,0,0,100,0.002f},{10,0,0,0,1000,0.002f}};
}
constexpr B::Input Input() { return {0,0,0,0,false,false,true,true,true}; }
constexpr bool HomeAndSafety() {
  B b; auto in=Input();
  if(b.startHome(in,0)) return false;
  if(!b.configure(Config())||b.moveTo(-5,0)) return false;
  in.authorized=false; if(b.startHome(in,0)) return false;
  in.authorized=true; if(!b.startHome(in,0)) return false;
  auto out=b.update(in,1); if(out.right<=0||out.left<=0) return false;
  in.right_limit=true; out=b.update(in,2);
  if(out.right!=0||out.left<=0) return false;
  in.left_limit=true; b.update(in,3); b.update(in,13);
  if(b.state()!=B::State::Holding) return false;
  if(b.moveTo(-701,14)||b.moveTo(1,14)||!b.moveTo(-5,14)) return false;
  out=b.update(in,15); if(out.right>=0||out.left>=0) return false;
  in.right_limit=in.left_limit=false; in.right_mm=in.left_mm=-5;
  b.update(in,16); if(b.isAtPos()) return false;
  b.update(in,26); if(!b.isAtPos()) return false;
  in.authorized=false; out=b.update(in,27);
  return out.right==0&&out.left==0&&!b.isAtPos()&&b.state()==B::State::Unhomed;
}
constexpr bool Faults() {
  B b; auto in=Input(); b.configure(Config()); b.startHome(in,0);
  b.update(in,1000); if(b.state()!=B::State::Fault) return false;
  if(b.startHome(in,1001)) return false;
  b.reset(); b.startHome(in,0); in.left_mm=101; b.update(in,2);
  if(b.state()!=B::State::Fault) return false;
  b.reset(); in=Input(); b.startHome(in,0);
  in.right_limit=in.left_limit=true; b.update(in,1); b.update(in,11);
  b.moveTo(-5,12); b.update(in,5012);
  if(b.state()!=B::State::Fault) return false;
  b.reset(); in=Input(); b.startHome(in,0); in.feedback_ok=false;
  auto out=b.update(in,1); if(out.right!=0||b.state()!=B::State::Unhomed) return false;
  in=Input(); b.startHome(in,0); in.right_mm=std::numeric_limits<float>::quiet_NaN();
  return b.update(in,1).right==0&&b.state()==B::State::Unhomed;
}
constexpr bool SimulatedMotion() {
  B b; b.configure(Config()); auto in=Input();
  b.startHome(in,0); in.right_limit=in.left_limit=true;
  b.update(in,1); b.update(in,11); b.moveTo(-100,12);
  for(uint32_t t=14;t<5000;t+=2) {
    in.right_limit=in.right_mm>=0; in.left_limit=in.left_mm>=0;
    const auto u=b.update(in,t);
    // Illustrative velocity plant: acceleration = raw command - damping*speed.
    in.right_speed+=0.002f*(u.right-2*in.right_speed);
    in.left_speed+=0.002f*(u.left-2.2f*in.left_speed);
    in.right_mm+=0.002f*in.right_speed; in.left_mm+=0.002f*in.left_speed;
    if(u.right>1000||u.right<-1000||u.left>1000||u.left<-1000) return false;
    if(b.isAtPos()) return in.right_mm<-99&&in.left_mm<-99;
  }
  return false;
}
constexpr bool BounceAndWrap() {
  B b; b.configure(Config()); auto in=Input(); b.startHome(in,0xfffffff0u);
  in.right_limit=in.left_limit=true; b.update(in,0xfffffff1u);
  in.left_limit=false; b.update(in,0xfffffff5u);
  in.left_limit=true; b.update(in,0xfffffffau);
  b.update(in,3); if(b.state()!=B::State::Homing) return false;
  b.update(in,4); return b.state()==B::State::Holding;
}
static_assert(HomeAndSafety());
static_assert(Faults());
static_assert(SimulatedMotion());
static_assert(BounceAndWrap());
