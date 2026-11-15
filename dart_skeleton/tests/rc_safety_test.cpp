#include "rc_safety.hpp"
#include <array>
constexpr auto Frame(unsigned value = 1024, unsigned left = 2, unsigned right = 2) {
  std::array<uint8_t, 18> b{};
  for (unsigned ch = 0; ch < 4; ++ch)
    for (unsigned bit = 0; bit < 11; ++bit)
      b[(ch*11+bit)/8] |= ((value >> bit)&1) << ((ch*11+bit)%8);
  b[5] |= (left << 6) | (right << 4);
  b[16] = value & 255; b[17] = value >> 8;
  return b;
}
constexpr bool Timeout() {
  dart::RcSafety s;
  auto b = Frame();
  if (s.online(0) || !s.accept(b.data(),18,10)) return false;
  if (!s.online(109) || s.online(110)) return false;
  if (s.accept(b.data(),17,111) || s.online(111)) return false;
  s.accept(b.data(),18,0xfffffff0u);
  return s.online(0x53) && !s.online(0x54);
}
constexpr bool Validate() {
  dart::RcSafety s;
  for (unsigned channel : {364u, 1024u, 1684u}) {
    auto b = Frame(channel);
    if (!s.accept(b.data(),18,0)) return false;
  }
  for (unsigned channel : {0u, 363u, 1685u, 2047u}) {
    auto b = Frame(channel);
    if (s.accept(b.data(),18,0)) return false;
  }
  for (unsigned left : {1u,2u,3u})
    for (unsigned right : {1u,2u,3u}) {
      auto b = Frame(1024,left,right);
      if (!s.accept(b.data(),18,0)) return false;
    }
  auto bad = Frame(1024,0,2);
  if (s.accept(bad.data(),18,0)) return false;
  bad = Frame(1024,2,0);
  if (s.accept(bad.data(),18,0)) return false;
  bad = Frame(); bad[12] = 2;
  if (s.accept(bad.data(),18,0)) return false;
  bad = Frame(); bad[17] = 255;
  return !s.accept(bad.data(),18,0);
}
static_assert(Timeout(), "offline, malformed length, timeout boundary or tick wrap failed");
static_assert(Validate(), "DBUS bounds or switches validation failed");
