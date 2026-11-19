/*
 * 遥控实例化（考核自行实现）。创建唯一 DT7 对象；解码类由骨架提供，本文件只负责实例接线。
 */
#include "rc_ins.hpp"
namespace { hw_rc::DT7 receiver; }
hw_rc::DT7* CreateRc() { return &receiver; }
