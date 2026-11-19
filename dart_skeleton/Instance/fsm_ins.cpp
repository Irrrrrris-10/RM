/*
 * 主状态机实例化（考核自行实现）。提供整个程序使用的同一个主流程对象。
 */
#include "fsm_ins.hpp"
#include "main_fsm.hpp"
namespace { dart::MainFsm main_fsm; }
dart::MainFsm* CreateMainFsm(){return &main_fsm;}
dart::BeltFsm* CreateBeltFsm(){return &main_fsm.sequence.belt;}
dart::ScrewFsm* CreateScrewFsm(){return &main_fsm.sequence.screw;}
dart::ShootFsm* CreateShootFsm(){return &main_fsm.sequence.shoot;}
