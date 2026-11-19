/*
 * 集中控制参数（后续新增）。位置 mm、速度 mm/s、时间 Ms 为毫秒、舵机角度 deg；PID 输出采用电流报文原始值。参数需重新编译下载才生效，目标角度不是实际角度反馈。
 */
/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-11-16 00:09:29
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-11-18 20:46:39
 * @FilePath: \dart_skeleton\Dart-Configs\control_config.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include "pid.hpp"
namespace dart::config {
// Set true only after physical directions, stop geometry, current thresholds and PWM are verified.
#ifndef DART_HARDWARE_CALIBRATED
#define DART_HARDWARE_CALIBRATED 0
#endif
// 编译期硬件使能标志；ON 只是软件许可，不是实测校准结果。
inline constexpr bool kHardwareCalibrated=DART_HARDWARE_CALIBRATED != 0;
// 受控恢复的运动限速，单位 mm/s；用于返回下拉点、恢复找零和停靠。
inline constexpr float kRecoverySpeed=180;
// 恢复位置移动最长40秒；等待停止最长2秒，超时进入故障。
inline constexpr unsigned kRecoveryMoveTimeoutMs=40000,kRecoveryStopTimeoutMs=2000;
// 控制周期0.002秒，即500Hz；计时累计和PID计算依赖实际调度与之相符。
inline constexpr float kPeriod=.002f;
// 输出轴每转动1 rad，对应同步带移动16.8 mm。
inline constexpr float kBeltMmPerRad=16.8f;
// 下拉目标为-830 mm，上返/恢复停靠目标为-5 mm；零点在上端。
inline constexpr float kDownMm=-830, kUpMm=-5;
// 位置目标的下上边界，SetTarget 会限幅；实际越界保护另留5 mm余量。
inline constexpr float kBeltMinMm=-850,kBeltMaxMm=5;
// 正常位置移动的速度限幅，主要用于下拉，不保证实机一定达到该速度。
inline constexpr float kMaxBeltSpeed=255; // mm/s, position moves including pull-down
// 正常上返阶段速度限幅；位置环内部输出上限也采用该值。
inline constexpr float kReturnBeltSpeed=300; // mm/s, return stroke only
// 同步纠偏比例：每1 mm右减左位置差产生2 mm/s修正。
inline constexpr float kBeltSyncKp=2.0f; // (mm/s)/mm of right-minus-left error
// 同步修正每侧最多15 mm/s；最终目标仍受阶段总速度限幅。
inline constexpr float kBeltSyncMaxSpeed=15.0f; // bounded correction per side
// 正常堵转找零速度，单位 mm/s；此阶段未实现接近顶端的位置减速。
inline constexpr float kHomeSpeed=270;    // mm/s toward upper mechanical stop
// 从每次找零起点计算的最大绝对位移；任一侧超出则故障。
inline constexpr float kHomeTravelMax=850;
// Whole-cycle budget covers homing + three position moves + servo action/reset.
// 依次为找零80秒、普通位置移动15秒、完整流程130秒超时。
inline constexpr unsigned kHomeTimeoutMs=80000,kMoveTimeoutMs=15000,kShotTimeoutMs=130000;
// 启动找零先忽略堵转500ms；找零堵转需持续400ms；普通卡滞持续500ms。
inline constexpr unsigned kHomeGraceMs=500,kHomeStallMs=400,kJamMs=500;
// 找零判据：电流绝对值至少1A且速度绝对值小于1mm/s，再结合持续时间。
inline constexpr float kHomeCurrentA=1.0f,kHomeSpeedThreshold=1.0f; // feedback A and mm/s
// 找零/同步带/丝杆的原始电流指令限幅；这里不是直接的安培数。
inline constexpr float kHomeRawLimit=1200,kBeltRawLimit=2500,kScrewRawLimit=1500;
// 位置误差容差3mm、到位速度容差3mm/s、两侧最大位置差15mm。
inline constexpr float kPositionTolerance=3,kSpeedTolerance=3,kMaxSkewMm=15;
// 普通到位需稳定100ms；击发请求保持1000ms；复位收尾等待200ms。
inline constexpr unsigned kSettleMs=100,kShootMs=1000,kServoResetMs=200;
// 准备阶段将位置环结果放大2倍，再受速度限幅。
inline constexpr float kPreparationResponseScale=2.0f;
// 准备阶段连续满足到位条件30ms后允许进入下拉。
inline constexpr unsigned kPreparationSettleMs=30;
// 正常位置移动的速度限幅，主要用于下拉，不保证实机一定达到该速度。
inline constexpr PidConfig kBeltPosition{3,0,0,0,kMaxBeltSpeed};
// Allow more bounded integral effort near the target without raising total output.
// 速度环输入mm/s，输出原始电流指令；积分项限幅500，总输出限幅见配置。
inline constexpr PidConfig kBeltSpeed{25,5,0,500,kBeltRawLimit};
// 丝杆位置环输入rad，输出rad/s；速度目标上限0.3rad/s。
inline constexpr PidConfig kScrewPosition{3,0,0,0,.3f};
// 丝杆速度环输出原始电流指令，不直接发送角度给电调。
inline constexpr PidConfig kScrewSpeed{900,100,0,200,kScrewRawLimit};
// 相对捕获位置的最大角度误差0.25rad，超出触发丝杆行程故障。
inline constexpr float kScrewMaxError=.25f; // output shaft rad, no automatic travel on startup
// Existing skeleton pulse values are timer counts at 100kHz, not microseconds.
// Instance adapter preserves 50..250 counts (500..2500us), 200Hz with the current instance adapter.
// 复位角与击发角，单位度；所有调用点共享这两个常量，并非位置传感器反馈。
inline constexpr float kServoSafeDeg=20,kServoFireDeg=95;
}
