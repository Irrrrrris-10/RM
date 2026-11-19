/*
 * 控制模块公共接口（新增）。ControlInput 是本周期输入快照；ControlOutput 默认零电流和安全舵机角度。raw 是 CAN 电流指令原始值，不是安培；recovery 标记受控恢复输出。
 */
#pragma once
#include "motor.hpp"
#include "control_config.hpp"
namespace dart {
enum class Fault : uint8_t {None, Offline, FeedbackReset, HomeTimeout, HomeTravel, Stall, Skew, Travel, MoveTimeout, ScrewTravel, SequenceTimeout, Can, Servo};
struct ControlInput { uint32_t now=0; MotorState motor[3]{}; bool rc_online=false; bool permitted=false,gesture=false,working=false,hardware_ready=false,can_ok=false; };
struct ControlOutput { float raw[3]{}; float servo_deg=config::kServoSafeDeg; bool active=false,recovery=false; };
constexpr bool AllOnline(const ControlInput& i) {return i.motor[0].online&&i.motor[1].online&&i.motor[2].online;}
}
