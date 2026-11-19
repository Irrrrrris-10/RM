#include "motor_protocol.hpp"
constexpr dart::MotorFrame Feedback(uint32_t id, int angle, int rpm=0, int current=0) {
  dart::MotorFrame f; f.id=id;
  f.data = {uint8_t(angle>>8),uint8_t(angle),uint8_t(uint16_t(rpm)>>8),uint8_t(rpm),uint8_t(uint16_t(current)>>8),uint8_t(current),70,0};
  return f;
}
constexpr bool Parse() {
  dart::Motor m{dart::MotorType::M3508,1};
  if(m.online(0)) return false;
  if(!m.decode(Feedback(0x201,8191,-60,-16384),0)) return false;
  if(m.speed_rpm()!=-60 || m.current_raw()!=-16384 || m.current_a()!=-20 || m.temperature_c()!=70) return false;
  m.decode(Feedback(0x201,0),1);
  if(m.rotor_counts()!=8192) return false;
  m.decode(Feedback(0x201,8191),2);
  if(m.rotor_counts()!=8191) return false;
  auto f=Feedback(0x201,8192);
  if(m.decode(f,3)) return false;
  f=Feedback(0x201,1); f.extended=true;
  if(m.decode(f,3)) return false;
  f.extended=false; f.remote=true;
  if(m.decode(f,3)) return false;
  f.remote=false; f.dlc=7;
  if(m.decode(f,3) || m.decode(Feedback(0x202,1),3)) return false;
  if(!m.online(51) || m.online(52)) return false;
  m.decode(Feedback(0x201,100),52);
  if(m.position_valid()) return false;
  return m.zero_position(52) && m.angle_rad()==0;
}
constexpr bool Packing() {
  dart::DartMotorBus b;
  b.right.setInput(2147483647); b.left.setInput(-2147483647-1); b.screw.setInput(20000);
  for(unsigned id=1;id<=3;++id) b.receive(Feedback(0x200+id,0),0xfffffff0u);
  const auto f=b.command(0x10,true);
  if(f.id!=0x200 || f.data!=std::array<uint8_t,8>{0x40,0,0x40,0,0xd8,0xf0,0,0}) return false;
  if(b.command(0x22,true).data!=std::array<uint8_t,8>{}) return false;
  if(b.command(0x10,false).data!=std::array<uint8_t,8>{}) return false;
  return !b.receive(Feedback(0x204,0),0) && !b.screw.has_temperature();
}
constexpr bool Units() {
  dart::Motor m{dart::MotorType::M2006,3,true};
  m.decode(Feedback(0x203,4096,60,10000),0);
  return m.angle_rad() < -0.08726 && m.angle_rad()>-0.08727 &&
    m.velocity_rad_s() < -0.17453 && m.velocity_rad_s()>-0.17454 && m.current_a()==-10;
}
static_assert(Parse());
static_assert(Packing());
static_assert(Units());
constexpr bool MultiTurnAndFaults() {
  dart::Motor m{dart::MotorType::M3508,1};
  m.decode(Feedback(0x201,0),0);
  for (unsigned i=1;i<=100;++i) m.decode(Feedback(0x201,(i*1000)%8192),i);
  if(m.rotor_counts()!=100000) return false;
  for (unsigned i=1;i<=100;++i) m.decode(Feedback(0x201,((100-i)*1000)%8192),100+i);
  if(m.rotor_counts()!=0) return false;
  m.decode(Feedback(0x201,4096),201);
  if(m.position_valid()) return false;
  m.setInput(1);
  if(m.wire_input(201,true)!=0) return false;
  if(!m.zero_position(201)) return false;
  dart::Motor invalid{dart::MotorType::M2006,0};
  if(invalid.decode(Feedback(0x200,0),0)) return false;
  dart::DartMotorBus b;
  b.right.decode(Feedback(0x201,0),0); b.left.decode(Feedback(0x202,0),0);
  b.right.setInput(100);
  return b.command(0,true).data==std::array<uint8_t,8>{};
}
static_assert(MultiTurnAndFaults());
