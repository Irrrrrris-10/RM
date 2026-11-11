/**
 *******************************************************************************
 * @file      : motor_configs.hpp
 * @brief     : 舵机参数（电机参数由新人自行定义）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：提供精简版舵机参数，
 *                                                 电机参数由新人定义
 *******************************************************************************
 * @attention :
 *  骨架不再提供电机类，因此这里不再有电机 OptionalParams / PID 配置。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_CONFIGS_MOTOR_CONFIGS_HPP_
#define DART_CONFIGS_MOTOR_CONFIGS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "base.hpp"
#include "servo.hpp"

namespace hw_servo = hello_world::servo;

/* Exported constants --------------------------------------------------------*/
//* 舵机相关配置（已提供） *//
const hw_servo::ServoInfo kServoShoot{
    .operating_freq = 200.0f,
    .min_pulse_duration = 50,
    .max_pulse_duration = 250,
    .angle_range = 180.0f,
};

/* ============================================================================
 * 【新人任务】电机参数由你在自己的电机设备类 / 配置中定义。
 *
 * 建议在此（或你新建的配置文件中）定义：
 *   1. 各电机的 ID、方向、减速比、输入类型等：
 *        同步带右 M3508（id=1，正向）、同步带左 M3508（id=2，反向）、
 *        丝杆 M2006（id=3，反向）；
 *        协议参数（反馈帧格式、单位换算、CAN ID）见考核说明 2.1 / 7.2 节与
 *        docs/ 手册。
 *   2. 各路控制环的 PID 参数（同步带位置环 ×2、丝杆位置环 ×1）。
 *
 * 提示：PID 类需要你自己实现（骨架不提供）。
 * ==========================================================================*/

#endif /* DART_CONFIGS_MOTOR_CONFIGS_HPP_ */
