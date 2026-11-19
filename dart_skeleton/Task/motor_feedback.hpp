/*
 * 电机协议基础（考核自行实现）。8 字节标准数据帧，大端角度、带符号转速和电流；在线窗口为 50 ms，索引 0/1/2 对应 ID 1/2/3。
 */
#pragma once
#include <cstdint>
namespace dart {
struct MotorFeedback {
  uint32_t frames=0, tick=0;
  uint16_t angle=0;
  int32_t rpm=0, current_raw=0;
  uint8_t temperature=0;
  constexpr bool Online(uint32_t now) const { return frames && uint32_t(now-tick)<50; }
};
constexpr int32_t SignedBe(const uint8_t* p) {
  const int32_t n=(uint32_t(p[0])<<8)|p[1];
  return n>=32768 ? n-65536 : n;
}
constexpr bool DecodeMotor(MotorFeedback& out, const uint8_t* p, unsigned size, uint32_t now) {
  if (!p || size!=8) return false;
  const unsigned angle=(unsigned(p[0])<<8)|p[1];
  if(angle>8191) return false;
  out.angle=angle; out.rpm=SignedBe(p+2); out.current_raw=SignedBe(p+4);
  out.temperature=p[6]; out.tick=now; out.frames+=1; return true;
}
constexpr int MotorIndex(uint32_t id, bool extended, bool remote) {
  return !extended && !remote && id>=0x201 && id<=0x203 ? int(id-0x201) : -1;
}
}
