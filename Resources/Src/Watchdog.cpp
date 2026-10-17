#include "Watchdog.h"

#include "stm32f4xx_hal.h"
#include "iwdg.h"

/* 统计量, 方便调试时由调试器观察 */
static uint32_t s_feed_count = 0;       // 喂狗次数
static uint32_t s_control_count = 0;    // 控制任务执行次数
static uint32_t s_last_polled_count = 0;// 上一次 Poll 时看到的心跳值

extern "C" void Watchdog_Init(void) {
    // IWDG 已由 CubeMX 生成的 MX_IWDG_Init() 启动, 这里只做状态清零。
    // 若未使用 CubeMX 的 MX_IWDG_Init(), 可在此处手动初始化:
    //   hiwdg.Instance = IWDG;
    //   hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
    //   hiwdg.Init.Reload = 400;
    //   HAL_IWDG_Init(&hiwdg);
    s_feed_count = 0;
    s_control_count = 0;
    s_last_polled_count = 0;
}

extern "C" void Watchdog_Feed(void) {
    HAL_IWDG_Refresh(&hiwdg);   // 重装 IWDG 计数器
    s_feed_count++;
}

extern "C" void Watchdog_NotifyControlLoop(void) {
    s_control_count++;
}

extern "C" void Watchdog_Poll(void) {
    // 只有控制任务心跳相比上次 Poll 有推进, 才认为系统健康并喂狗。
    // 若控制任务卡死(心跳不变), 则这里不喂狗, 800ms 后 IWDG 复位。
    if (s_control_count != s_last_polled_count) {
        s_last_polled_count = s_control_count;
        Watchdog_Feed();
    }
    // 注意: 本函数必须周期性地被主循环调用;
    //       若主循环自身卡死, 同样不会喂狗 -> 复位。
}

extern "C" uint32_t Watchdog_GetFeedCount(void) {
    return s_feed_count;
}

extern "C" uint32_t Watchdog_GetControlLoopCount(void) {
    return s_control_count;
}
