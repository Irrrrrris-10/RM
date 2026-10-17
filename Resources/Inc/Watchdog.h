#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

// 看门狗保护封装(基于 STM32 独立看门狗 IWDG)
//
// 关键参数(在 CubeMX/MX_IWDG_Init 中配置):
//   Prescaler = IWDG_PRESCALER_64, Reload = 400
//   F4 的 LSI ≈ 32kHz
//   溢出时间 Tout = (PSC * RLR) / LSI = (64 * 400) / 32000 = 0.8s
//   即: 800ms 内必须喂狗一次, 否则 MCU 复位。
//
// 本模块提供两种用法:
//   1) 简单方式: 在 1kHz 控制任务末尾调用 Watchdog_Feed()
//   2) 强保护 : 控制任务调用 Watchdog_NotifyControlLoop() 心跳,
//               主循环调用 Watchdog_Poll() —— 只有控制任务确实在跑时才喂狗,
//               任务卡死/中断停摆都会触发复位。
#ifdef __cplusplus
extern "C" {
#endif

void Watchdog_Feed(void);              // 无条件喂狗(IWDG 计数器重装)
void Watchdog_NotifyControlLoop(void); // 控制任务心跳(每个控制周期调用)
void Watchdog_Poll(void);              // 主循环调用: 仅在心跳推进时喂狗
void Watchdog_Init(void);              // 可选: 记录复位来源等

uint32_t Watchdog_GetFeedCount(void);
uint32_t Watchdog_GetControlLoopCount(void);

#ifdef __cplusplus
}
#endif

#endif // WATCHDOG_H
