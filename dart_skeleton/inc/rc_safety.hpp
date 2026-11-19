#pragma once
#include "DT7.hpp"
#include <cstddef>
#include <cstdint>

namespace dart {
// Independent of HAL so frame validation and timeout policy can be tested offline.
class RcSafety {
 public:
  static constexpr uint32_t kTimeoutMs = 100;
  constexpr bool accept(const uint8_t* bytes, size_t size, uint32_t now) {
    if (size != 18) { invalidate(); return false; }
    const uint16_t channels[] = {
      uint16_t((bytes[0] | bytes[1] << 8) & 0x7ff),
      uint16_t((bytes[1] >> 3 | bytes[2] << 5) & 0x7ff),
      uint16_t((bytes[2] >> 6 | bytes[3] << 2 | bytes[4] << 10) & 0x7ff),
      uint16_t((bytes[4] >> 1 | bytes[5] << 7) & 0x7ff),
      uint16_t(bytes[16] | bytes[17] << 8)};
    for (auto ch : channels) {
      if (ch < 364 || ch > 1684) { invalidate(); return false; }
    }
    const unsigned left = (bytes[5] >> 6) & 3;
    const unsigned right = (bytes[5] >> 4) & 3;
    if (!left || !right || bytes[12] > 1 || bytes[13] > 1) {
      invalidate(); return false;
    }
    valid_ = true;
    last_tick_ = now;
    return true;
  }
  constexpr void invalidate() { valid_ = false; }
  constexpr bool online(uint32_t now) const {
    return valid_ && uint32_t(now - last_tick_) < kTimeoutMs;
  }
  constexpr uint32_t last_tick() const { return last_tick_; }
 private:
  bool valid_ = false;
  uint32_t last_tick_ = 0;
};
struct RcSnapshot {
  hello_world::remote_control::DT7 rc;
  bool online = false;
  uint32_t last_valid_tick = 0;
};
}  // namespace dart

