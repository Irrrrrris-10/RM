/*
 * 遥控校验与触发辅助（新增）。DBUS 无 CRC，仅做长度、通道、拨杆等合理性校验；100 ms 无更新视为离线。完整发射和恢复的最终授权还在 Sequence/MainTask。
 */
#pragma once
#include <cstdint>
#include <cstddef>
namespace dart {
struct RcSample {
  bool valid = false;
  uint32_t tick = 0;
  float lh = 0, lv = 0, rh = 0, rv = 0, wheel = 0;
  uint8_t left = 0, right = 0;
};
// DBUS has no CRC; plausibility checks cannot detect every bit error.
constexpr unsigned DbusRejectReason(const uint8_t* b, std::size_t size) {
  if (!b || size != 18) return 2;
  const unsigned channels[] = {
    (b[0] | b[1] << 8) & 2047u,
    (b[1] >> 3 | b[2] << 5) & 2047u,
    (b[2] >> 6 | b[3] << 2 | b[4] << 10) & 2047u,
    (b[4] >> 1 | b[5] << 7) & 2047u,
    static_cast<unsigned>(b[16] | b[17] << 8)};
  for (auto c : channels) if (c < 364 || c > 1684) return 3;
  if (((b[5] >> 6) & 3) == 0 || ((b[5] >> 4) & 3) == 0) return 4;
  if (b[12] > 1 || b[13] > 1) return 5;
  return 0;
}
constexpr bool ValidDbus(const uint8_t* b, std::size_t size) {
  return DbusRejectReason(b, size) == 0;
}
constexpr bool RcOnline(const RcSample& rc, uint32_t now) {
  return rc.valid && static_cast<uint32_t>(now - rc.tick) < 100u;
}
class RcSafety {
 public:
  constexpr bool Update(const RcSample& rc, uint32_t now, bool working) {
    one_shoot = RcOnline(rc, now) && working && rc.left == 2 && rc.right == 2;
    if (!one_shoot) { armed_ = false; return false; }
    const bool gesture = rc.lh > .8f && rc.lv < -.8f && rc.rh < -.8f && rc.rv < -.8f;
    if (!gesture) { armed_ = true; return false; }
    const bool trigger = armed_;
    armed_ = false;
    return trigger;
  }
  bool one_shoot = false;
 private:
  bool armed_ = false;
};
}
