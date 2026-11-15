#pragma once
#include <array>
#include <cstdint>
#include <cstddef>

namespace dart {
struct MotorFrame {
  uint32_t id = 0;
  bool extended = false;
  bool remote = false;
  uint8_t dlc = 8;
  std::array<uint8_t,8> data{};
};
enum class MotorType { M3508, M2006 };
class Motor {
 public:
  constexpr Motor(MotorType type, uint8_t id, bool reversed = false)
      : type_(type), id_(id), sign_(reversed ? -1 : 1) {}
  constexpr bool configured() const { return id_ >= 1 && id_ <= 4; }
  constexpr bool online(uint32_t now) const { return received_ && uint32_t(now-last_tick_) < 50; }
  constexpr bool position_valid() const { return position_valid_; }
  constexpr uint8_t id() const { return id_; }
  constexpr int limit() const { return type_ == MotorType::M3508 ? 16384 : 10000; }
  constexpr double ratio() const { return type_ == MotorType::M3508 ? 3591.0/187.0 : 36.0; }
  constexpr bool decode(const MotorFrame& f, uint32_t now) {
    if (!configured() || f.extended || f.remote || f.dlc != 8 || f.id != 0x200u+id_) return false;
    const int angle = (f.data[0]<<8)|f.data[1];
    if (angle > 8191) return false;
    if (!received_) { counts_ = angle; position_valid_ = true; }
    else if (!online(now)) { position_valid_ = false; }
    else if (position_valid_) {
      int delta = angle - raw_angle_;
      // Encoder values 0..8191 form a modulo-8192 counter.
      if (delta > 4096) delta -= 8192;
      if (delta < -4096) delta += 8192;
      if (delta == 4096 || delta == -4096) position_valid_ = false;
      else counts_ += delta;
    }
    raw_angle_ = angle;
    rpm_ = Signed(f.data[2],f.data[3]);
    current_ = Signed(f.data[4],f.data[5]);
    temperature_ = f.data[6];
    last_tick_ = now; received_ = true;
    return true;
  }
  // Explicit re-reference after a feedback gap; caller must establish a safe origin.
  constexpr bool zero_position(uint32_t now) {
    if (!online(now)) return false;
    counts_ = 0; position_valid_ = true; return true;
  }
  constexpr double angle_rad() const { return sign_*counts_*kTau/8192.0/ratio(); }
  constexpr double velocity_rad_s() const { return sign_*rpm_*kTau/60.0/ratio(); }
  constexpr double current_a() const { return sign_*current_*(type_ == MotorType::M3508 ? 20.0/16384.0 : 0.001); }
  constexpr bool has_temperature() const { return type_ == MotorType::M3508; }
  constexpr uint8_t temperature_c() const { return has_temperature() ? temperature_ : 0; }
  constexpr int64_t rotor_counts() const { return counts_; }
  constexpr int speed_rpm() const { return rpm_; }
  constexpr int current_raw() const { return current_; }
  constexpr void setInput(int32_t input) { input_ = input > limit() ? limit() : input < -limit() ? -limit() : input; }
  constexpr int wire_input(uint32_t now, bool authorized) const {
    return configured() && authorized && online(now) && position_valid_ ? sign_*input_ : 0;
  }
 private:
  static constexpr double kTau = 6.2831853071795864769;
  static constexpr int Signed(uint8_t high, uint8_t low) {
    const int value = (high<<8)|low;
    return value >= 32768 ? value-65536 : value;
  }
  MotorType type_;
  uint8_t id_;
  int sign_;
  bool received_ = false, position_valid_ = false;
  uint32_t last_tick_ = 0;
  int raw_angle_ = 0, rpm_ = 0, current_ = 0, input_ = 0;
  uint8_t temperature_ = 0;
  int64_t counts_ = 0;
};

// Fixed robot topology; avoids duplicate IDs and overwriting shared frame slots.
class DartMotorBus {
 public:
  Motor right{MotorType::M3508,1};
  Motor left{MotorType::M3508,2,true};
  Motor screw{MotorType::M2006,3,true};
  constexpr bool receive(const MotorFrame& frame, uint32_t now) {
    switch (frame.id) {
      case 0x201: return right.decode(frame,now);
      case 0x202: return left.decode(frame,now);
      case 0x203: return screw.decode(frame,now);
      default: return false;
    }
  }
  constexpr MotorFrame command(uint32_t now, bool authorized) const {
    MotorFrame frame; frame.id = 0x200;
    // Any missing motor inhibits the entire mechanism.
    const bool ready = authorized && right.online(now) && left.online(now) && screw.online(now)
      && right.position_valid() && left.position_valid() && screw.position_valid();
    const int values[] = {right.wire_input(now,ready),left.wire_input(now,ready),screw.wire_input(now,ready)};
    for (size_t i=0;i<3;++i) {
      const auto value = static_cast<uint16_t>(values[i]);
      frame.data[2*i] = value>>8; frame.data[2*i+1] = value&255;
    }
    return frame;
  }
};
}
