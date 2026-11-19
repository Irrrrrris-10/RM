/*
 * 遥控最新采样的共享读取接口（新增）。业务模块读取一致快照，不直接访问 DMA 缓冲区。
 */
#pragma once
#include "rc_safety.hpp"
struct RcDebug {
  uint32_t valid_frames, invalid_frames, uart_errors, restart_errors;
  uint32_t last_size, last_valid_tick;
  float lh, lv, rh, rv, wheel;
  uint8_t left, right;
};
struct SafetyDebug {
  uint32_t cycles, trigger_events;
  uint8_t online, work_state, one_shoot, output_enabled, timer_started, diagnostic_mode;
};
extern volatile RcDebug g_rc_debug;
extern volatile SafetyDebug g_safety_debug;
dart::RcSample ReadRcSample();
