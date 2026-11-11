/**
 *******************************************************************************
 * @file      : servo.cpp
 * @brief     : 舵机驱动实现（精简版）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：由完整版精简而来
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "servo.hpp"

#ifdef HAL_TIM_MODULE_ENABLED

#include "base.hpp"

namespace hello_world
{
namespace servo
{

Servo::Servo(TIM_HandleTypeDef* htim, uint32_t channel, const ServoInfo& info,
             float init_angle)
{
  init(htim, channel, info, init_angle);
}

void Servo::init(TIM_HandleTypeDef* htim, uint32_t channel, const ServoInfo& info,
                 float init_angle)
{
  htim_ = htim;
  channel_ = channel;
  info_ = info;

  uint32_t arr = static_cast<uint32_t>(1e6f / info_.operating_freq) - 1;
  __HAL_TIM_SET_AUTORELOAD(htim_, arr == 0 ? 1 : arr);
  setAngle(init_angle);
  enable();
}

void Servo::setAngle(float angle) const
{
  __HAL_TIM_SET_COMPARE(htim_, channel_, angle2Cmp(angle));
}

uint32_t Servo::angle2Cmp(float angle) const
{
  angle = Bound(angle, 0.0f, info_.angle_range);
  return static_cast<uint32_t>(
      angle * (info_.max_pulse_duration - info_.min_pulse_duration) /
          info_.angle_range +
      info_.min_pulse_duration);
}

}  // namespace servo
}  // namespace hello_world

#endif /* HAL_TIM_MODULE_ENABLED */
