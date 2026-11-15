#include "pid.hpp"
#include <limits>
constexpr bool Near(float a,float b,float tolerance=0.0001f) { return a-b<tolerance && b-a<tolerance; }
constexpr bool Proportional() {
  dart::Pid p({2,0,0,0,3,0.002f});
  return p.update(1,0)==2 && p.update(5,0)==3 && p.update(-5,0)==-3;
}
constexpr bool IntegralAndReset() {
  dart::Pid p({0,2,0,0.5f,10,0.002f});
  for(int i=0;i<500;++i) p.update(1,0);
  if(!Near(p.integral(),0.5f)) return false;
  p.update(0,0,false);
  if(p.integral()!=0 || p.output()!=0) return false;
  return Near(p.update(1,0),0.004f);
}
constexpr bool AntiWindup() {
  dart::Pid p({2,1,0,10,1,0.002f});
  for(int i=0;i<1000;++i) if(p.update(10,0)!=1) return false;
  if(p.integral()!=0 || p.update(0,0)!=0) return false;
  for(int i=0;i<1000;++i) if(p.update(-10,0)!=-1) return false;
  return p.integral()==0 && p.update(0,0)==0;
}
constexpr bool Derivative() {
  dart::Pid p({0,0,0.002f,0,10,0.002f});
  if(p.update(0,10)!=0) return false; // No startup impulse.
  if(p.update(100,10)!=0) return false; // No setpoint derivative kick.
  if(!Near(p.update(100,11),-1)) return false;
  p.reset();
  return p.update(0,100)==0;
}
constexpr bool InvalidInputs() {
  dart::Pid p;
  if(p.update(1,0)!=0) return false;
  if(p.configure({1,0,0,0,1,0})) return false;
  if(p.configure({-1,0,0,0,1,0.002f})) return false;
  if(p.configure({1,0,0,-1,1,0.002f})) return false;
  if(p.configure({1,0,0,0,-1,0.002f})) return false;
  if(p.configure({1,0,0,0,1,std::numeric_limits<float>::infinity()})) return false;
  p.configure({1,1,1,1,1,0.002f}); p.update(1,0);
  if(p.update(std::numeric_limits<float>::quiet_NaN(),0)!=0 || p.integral()!=0) return false;
  if(p.update(0,std::numeric_limits<float>::infinity())!=0) return false;
  p.configure({1,1,1,1,0,0.002f});
  return p.update(100,0)==0;
}
constexpr bool ClosedLoop() {
  // Illustrative first-order plant dy/dt = -y + u; not an identified motor model.
  dart::Pid p({2,1,0,2,3,0.002f});
  float y=0;
  for(int i=0;i<10000;++i) {
    const float u=p.update(1,y);
    if(u>3 || u<-3) return false;
    y += 0.002f*(-y+u);
  }
  if(!Near(y,1,0.005f)) return false;
  for(int i=0;i<10000;++i) y += 0.002f*(-y+p.update(-1,y));
  return Near(y,-1,0.005f);
}
static_assert(Proportional());
static_assert(IntegralAndReset());
static_assert(AntiWindup());
static_assert(Derivative());
static_assert(InvalidInputs());
static_assert(ClosedLoop());
