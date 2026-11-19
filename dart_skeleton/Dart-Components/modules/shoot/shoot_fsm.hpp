/*
 * 击发模块（考核自行实现）。只管理目标角度和击发计时，不直接操作定时器；角度通过 MainTask 和 motor_ins 写入已提供的 Servo 驱动。
 */
#pragma once
#include "control_config.hpp"
#include <cstdint>
namespace dart {
class ShootFsm {
 public:
 // 撤销击发标志和起始时间；实际PWM由上层写入。
 constexpr void Reset(){firing_=false;start_=0;}
 // 记录当前毫秒计时并设置击发标志，不阻塞等待。
 constexpr void Fire(uint32_t now){start_=now;firing_=true;}
 // 无符号时间差判断保持时间是否到达，兼容计时器自然回绕。
 constexpr bool Done(uint32_t now)const{return firing_ && uint32_t(now-start_)>=config::kShootMs;}
 // 按击发状态返回配置角度，不读取实际舵机位置。
 constexpr float angle()const{return firing_?config::kServoFireDeg:config::kServoSafeDeg;}
 private: bool firing_=false;uint32_t start_=0;
};
}
