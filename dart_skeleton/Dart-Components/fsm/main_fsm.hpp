#pragma once
#include "fsm.hpp"
#include "rc_safety.hpp"

namespace dart {
class MainFsm : public Fsm {
 public:
  enum class Mode : uint8_t { Standby, OneShoot };
  struct Commands {
    int16_t belt_right = 0;
    int16_t belt_left = 0;
    int16_t screw = 0;
    float shoot_angle = 95.0f;
  };
  void update(const RcSnapshot& input) {
    // Stage B keeps all requested outputs safe; physical modules arrive later.
    commands_ = Commands{};
    const auto& rc = input.rc;
    const bool selected = input.online && rc.rc_l_switch() == 2 && rc.rc_r_switch() == 2;
    mode_ = selected ? Mode::OneShoot : Mode::Standby;
    if (!selected) resetWorkState();
    else updateWorkState();
    motion_authorized_ = selected && work_state() == kWorkStateWorking &&
      rc.rc_lh() > 0.8f && rc.rc_lv() < -0.8f && rc.rc_rh() < -0.8f && rc.rc_rv() < -0.8f;
  }
  Mode mode() const { return mode_; }
  bool motion_authorized() const { return motion_authorized_; }
  const Commands& commands() const { return commands_; }
 private:
  Mode mode_ = Mode::Standby;
  bool motion_authorized_ = false;
  Commands commands_{};
};
}
