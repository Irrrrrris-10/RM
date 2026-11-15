#pragma once
#include "fsm.hpp"
#include "belt_controller.hpp"
namespace dart {
// Adapter keeps lifecycle in the supplied Fsm. Input limits are normalized booleans.
class BeltFsm : public Fsm {
 public:
  bool configure(BeltController::Config config) { resetWorkState(); return controller_.configure(config); }
  void update(BeltController::Input input,uint32_t now) {
    if(!input.authorized || !input.feedback_ok || !input.limits_valid) resetWorkState();
    else updateWorkState();
    input.authorized=input.authorized&&work_state()==kWorkStateWorking;
    controller_.update(input,now);
  }
  bool startHome(BeltController::Input input,uint32_t now) {
    input.authorized=input.authorized&&work_state()==kWorkStateWorking;
    return controller_.startHome(input,now);
  }
  bool moveTo(float mm,uint32_t now) { return work_state()==kWorkStateWorking&&controller_.moveTo(mm,now); }
  bool isAtPos() const { return controller_.isAtPos(); }
  BeltController::Output output() const { return controller_.output(); }
  BeltController::State state() const { return controller_.state(); }
  void reset() { resetWorkState();controller_.reset(); }
 private:
  BeltController controller_;
};
}
