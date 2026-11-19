/*
 * 电机和舵机实例接口。业务层通过这些函数访问对象和申请舵机输出，避免重复创建设备实例。
 */
#pragma once
#include "motor.hpp"
#include "servo.hpp"
dart::Motor* CreateMotorBeltRhs();
dart::Motor* CreateMotorBeltLhs();
dart::Motor* CreateMotorScrew();
dart::Motor* MotorByIndex(unsigned index);
hello_world::servo::Servo* CreateServoShoot();
bool InitServoShoot(bool enable);
void ApplyServoShoot(float angle,bool permit_fire);
bool ServoShootReady();
