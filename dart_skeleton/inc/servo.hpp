/**
 *******************************************************************************
 * @file      : servo.hpp
 * @brief     : 舵机驱动（精简版）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：由完整版精简而来，
 *                                                 只保留初始化 / 使能 / 设角
 *******************************************************************************
 * @attention :
 *  1. 本类按 1MHz 计数器换算脉宽（脉宽值直接作为比较值），沿用原工程约定；
 *     TIM2 的时钟/周期由 CubeMX 配置，改动前请与实车核对。
 *  2. 若实例作为全局变量，建议用无参构造 + init()，避免硬件句柄未初始化。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_
#define HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_

/* Includes ------------------------------------------------------------------*/
#include "stm32_hal.hpp"

/* 开启 TIM 才允许编译 */
#ifdef HAL_TIM_MODULE_ENABLED

namespace hello_world
{
namespace servo
{
/* Exported types ------------------------------------------------------------*/
struct ServoInfo {
  float operating_freq = 50;           ///< PWM 频率，单位 Hz
  uint16_t min_pulse_duration = 500;   ///< 最小脉宽，单位 us
  uint16_t max_pulse_duration = 2500;  ///< 最大脉宽，单位 us
  float angle_range = 180;             ///< 角度范围（从 0 开始），单位 deg
};

class Servo
{
  public:
    Servo() = default;
    Servo(TIM_HandleTypeDef* htim, uint32_t channel, const ServoInfo& info,
          float init_angle);

    /* 初始化并启动 PWM（可替代构造函数） */
    void init(TIM_HandleTypeDef* htim, uint32_t channel, const ServoInfo& info,
              float init_angle);

    void enable() const { HAL_TIM_PWM_Start(htim_, channel_); }
    void disable() const { HAL_TIM_PWM_Stop(htim_, channel_); }

    /* 设定角度（自动限幅到 0 ~ angle_range） */
    void setAngle(float angle) const;

  private:
    /* 角度转定时器比较值 */
    uint32_t angle2Cmp(float angle) const;

    TIM_HandleTypeDef* htim_ = nullptr;
    uint32_t channel_ = TIM_CHANNEL_1;
    ServoInfo info_;
};

}  // namespace servo
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */

#endif /* HW_COMPONENTS_DEVICES_SERVO_SERVO_HPP_ */
