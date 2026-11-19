/*
 * CAN 接口与诊断结构。计数、反馈、发送状态用于判断数据链路；电机数组按右同步带、左同步带、丝杆排序。
 */
#pragma once
#include "motor_feedback.hpp"
#include "control_types.hpp"
struct CanDebug {
  uint32_t filter_status, start_status, notification_status, started;
  uint32_t rx_frames, rejected_frames, unknown_frames, read_errors;
  uint32_t last_id, last_dlc, error_code, esr, fifo_overruns;
  uint32_t bitrate, online_mask;
  uint32_t tx_frames, tx_failures, tx_busy, tx_aborts, tx_fault, tx_confirmed, tx_last_tsr;
  int32_t command_raw[3];
  dart::MotorFeedback motors[3]; // right belt, left belt, screw
  uint32_t nonzero_tx_frames; // Nonzero command frames successfully queued to CAN hardware.
};
extern volatile CanDebug g_can_debug;
void CanFeedbackInit();
void CanFeedbackPoll();

void ReadMotorStates(dart::MotorState (&out)[3],uint32_t now);
bool CanTransmit(const dart::ControlOutput& command,bool permission);
