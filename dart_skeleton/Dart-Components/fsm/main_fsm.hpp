/*
 * 主状态机适配（考核自行实现）。复用骨架 Fsm 的生命状态，向 Sequence 提供 Working 状态；正常单发和恢复分支的权限判断位于 Sequence。
 */
#pragma once
#include "fsm.hpp"
#include "sequence.hpp"
namespace dart {
class MainFsm:public Fsm {
 public:
 ControlOutput Update(ControlInput in,bool rc_online) {
  if(rc_online)updateWorkState();else resetWorkState();
  in.working=work_state()==kWorkStateWorking;
  return sequence.Tick(in);
 }
 Sequence sequence;
};
}
