#ifndef CONTROLTASK_H
#define CONTROLTASK_H

#include <stdint.h>   /* C/C++ 通用, 便于 main.c 也能包含 */

// 供 C 中断处理(stm32f4xx_it.c 及 HAL 弱回调)调用的接口
#ifdef __cplusplus
extern "C" {
#endif

// 控制模式: 速度环跟随正弦 / 位置环(串级)跟随正弦
typedef enum {
    CTRL_MODE_VELOCITY = 0,   // 速度跟随模式
    CTRL_MODE_POSITION = 1    // 位置跟随模式
} CtrlMode_t;

// 上电初始化: 配置 CAN 过滤器、启动 CAN 与 1kHz 控制定时器
// 在 main.c 中 MX_XXX_Init() 全部执行完后调用一次
void ControlTaskInit(void);

// 1kHz 控制入口, 在 TIM6 周期中断(HAL_TIM_PeriodElapsedCallback)中调用
void MainTask(void);

// CAN 接收回调, 在 HAL_CAN_RxFifo0MsgPendingCallback 中调用
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data);

// 运行中切换控制模式(内部会复位 PID 和参考曲线)
void ControlTaskSetMode(CtrlMode_t mode);
CtrlMode_t ControlTaskGetMode(void);

// 主循环轮询: 当前用于 C 板 KEY(PA0) 切换速度/位置模式
void ControlTaskPoll(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROLTASK_H
