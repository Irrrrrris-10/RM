/**
 *******************************************************************************
 * @file      : ControlTask.cpp
 * @brief     : GM6020 正弦速度/位置跟随 + 独立看门狗(IWDG)保护
 *
 * 结构:
 *   1kHz(TIM6) 中断 -> MainTask()
 *        ├─ 反馈丢失检测
 *        ├─ 生成正弦参考 (SineRef)
 *        ├─ PID 运算 (Pid: 速度环 / 位置环串级)
 *        ├─ 下发 CAN 电流控制帧 (0x1FE)
 *        └─ 喂狗 (Watchdog)
 *   CAN0 RX 中断 -> HAL_CAN_RxFifo0MsgPendingCallback() -> CanFeedbackCallback()
 *        └─ 解析 0x205 反馈帧
 *******************************************************************************
 */
#include "ControlTask.h"

#include "Gm6020.h"
#include "Pid.h"
#include "SineRef.h"
#include "Watchdog.h"

#include <cmath>

extern "C" {
#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "iwdg.h"
}

/* ============================== J-Scope 调试变量 ============================== */
// 使用 C 链接，J-Scope 里可直接按名字添加这些全局变量。
extern "C" {
volatile float    g_dbg_v_ref     = 0.0f;  // 速度给定 rad/s
volatile float    g_dbg_vel       = 0.0f;  // 实际速度 rad/s（原始反馈）
volatile float    g_dbg_vel_filt  = 0.0f;  // 实际速度 rad/s（控制用滤波后）
volatile float    g_dbg_vel_disp  = 0.0f;  // 实际速度 rad/s（仅显示用平滑）
volatile float    g_dbg_theta_ref = 0.0f;  // 位置给定 rad
volatile float    g_dbg_angle     = 0.0f;  // 实际角度 rad
volatile float    g_dbg_command   = 0.0f;  // 当前电流给定
volatile float    g_dbg_current   = 0.0f;  // 实际转矩电流 A
volatile float    g_dbg_mode      = 0.0f;  // 0=速度, 1=位置
volatile uint32_t g_dbg_tick      = 0;     // 1ms 时基
}

/* ============================== 可调参数 ============================== */
namespace cfg {
// ---- 电机 ----
constexpr uint8_t kMotorId = 1;            // GM6020 拨码开关设置的 ID (1~7)

// ---- 控制模式 ----
// GM6020 使用电流控制帧 0x1FE / 0x2FE。


// 上电默认模式；运行中也可以按 C 板 KEY(PA0) 在速度/位置之间切换。
constexpr CtrlMode_t kDefaultMode = CTRL_MODE_POSITION;

// ---- 控制周期 ----
constexpr float kDt = 0.001f;              // 1kHz

// ---- 速度跟随曲线: v_ref(t) = offset + A*sin(2πft)  (单位 rad/s) ----
// 当前测试值: 幅值 8 rad/s，频率 0.2Hz，偏置 0，因此速度会在 ±8 rad/s 之间正反转。
constexpr float kVelAmp    =  8.0f;
constexpr float kVelFreq   = 0.2f;         // 0.2 Hz
constexpr float kVelOffset = 0.0f;

// ---- 位置跟随曲线: θ_ref(t) = θ0 + A*sin(2πft) + offset  (单位 rad) ----
// 位置跟随测试: ±90°，0.1Hz，周期 10s。
constexpr float kPosAmp    = 1.5707963f;
constexpr float kPosFreq   = 0.1f;         // 0.1 Hz
constexpr float kPosOffset = 0.0f;

// ---- 速度环 PID: 输入 rad/s, 输出电流给定 ----
// GM6020 电流给定范围 -16384 ~ +16384，实机调试时小幅调整。
constexpr float kVelKp = 300.0f;
constexpr float kVelKi = 8.0f;
constexpr float kVelKd = 0.0f;

// 速度反馈一阶低通滤波系数: alpha = 1 - exp(-2π*fc*T)
// 1kHz 控制周期下，0.15 约对应 26Hz 截止频率。
constexpr float kVelLpfAlpha = 1.0f;
// 仅用于调试显示的平滑系数，不影响控制回路。
constexpr float kDispAlpha = 0.05f;

// 速度前馈: 电流给定 += kVelFF * v_ref
// 用于提高正弦跟随的幅值/相位，并帮助越过零速死区。
constexpr float kVelFF = 30.0f;   // 电流前馈系数，单位 counts/(rad/s)

// 零速摩擦补偿: 参考速度非零但电机仍卡在 0 时，给一个小电流帮助启动。
constexpr bool  kUseFricComp = true;
constexpr float kFricComp    = 50.0f;   // counts，先试 40~60

// 速度环电流限幅: ±3000 约 ±0.55A，确认方向后再加大到 ±16384。
constexpr float kCurMax = 3000.0f, kCurMin = -3000.0f;

// ---- 位置环 PID: 输入 rad, 输出 速度给定 rad/s ----
constexpr float kPosKp = 12.0f, kPosKi = 0.5f, kPosKd = 0.0f;
constexpr float kPosVelMax = 10.0f, kPosVelMin = -10.0f;

constexpr bool kPosFeedforward = true;     // 位置模式是否叠加参考速度前馈

// ---- 反馈丢失保护 ----
constexpr uint32_t kOfflineTicks = 100;    // 连续 100ms 无反馈 -> 判掉线
}

/* ============================== 内部对象 ============================== */
static Gm6020  g_motor(cfg::kMotorId);

static Pid g_vel_pid(cfg::kVelKp, cfg::kVelKi, cfg::kVelKd,
                     cfg::kCurMax, cfg::kCurMin);
static Pid g_pos_pid(cfg::kPosKp, cfg::kPosKi, cfg::kPosKd,
                     cfg::kPosVelMax, cfg::kPosVelMin);

static SineRef g_vel_ref(cfg::kVelAmp, cfg::kVelFreq, cfg::kVelOffset,
                         0.0f, cfg::kDt);
static SineRef g_pos_ref(cfg::kPosAmp, cfg::kPosFreq, cfg::kPosOffset,
                         0.0f, cfg::kDt);

static volatile uint32_t  g_tick         = 0;   // 1ms 计数(时基)
static volatile uint32_t  g_last_fb_tick = 0;   // 最近一次反馈时刻
static volatile uint8_t   g_has_fb       = 0;   // 是否收到过反馈
static volatile CtrlMode_t g_mode        = cfg::kDefaultMode;

// 位置模式相对零点: 首次反馈/重新上线/切换模式时记录当前角度，
// 位置参考写成 theta0 + A*sin(...), 避免上电时向绝对 0 位置跳变。
static float              g_theta0       = 0.0f;
static volatile uint8_t   g_reinit_pos   = 1;   // 1 = 下一拍重新记录 theta0

/* ============================== CAN 过滤器 ============================== */
static void canFilterInit(CAN_HandleTypeDef *hcan) {
    CAN_FilterTypeDef f = {0};
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = 0x0000;
    f.FilterIdLow = 0x0000;
    f.FilterMaskIdHigh = 0x0000;   // 掩码全 0, 所有标准帧都进 FIFO0
    f.FilterMaskIdLow = 0x0000;
    f.FilterFIFOAssignment = CAN_FilterFIFO0;
    f.FilterBank = 0;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(hcan, &f) != HAL_OK) {
        Error_Handler();
    }
}

/* ============================== 控制帧发送 ============================== */
// 把本电机电流给定写入 8 字节帧并发送(0x1FE)
static void sendControlFrame(void) {
    uint8_t data[8] = {0};
    g_motor.encode(data);

    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox = 0;
    tx.StdId = g_motor.txId();
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    (void)HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

/* ============================== 对外接口 ============================== */
extern "C" void ControlTaskInit(void) {
    Watchdog_Init();

    // 位置参考在上电后第一次进入位置模式时再对准当前角度。
    g_reinit_pos = 1;

    canFilterInit(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    HAL_TIM_Base_Start_IT(&htim6);   // 启动 1kHz 控制中断
}

extern "C" void MainTask(void) {
    g_tick = g_tick + 1;   // 避免 C++20 对 volatile++ 的弃用告警

    /* 1. 反馈健康检查: 上电到首次收到反馈之前一律输出 0(安全) */
    const bool online =
        (g_has_fb != 0) && ((g_tick - g_last_fb_tick) <= cfg::kOfflineTicks);

    /* 速度反馈一阶低通: 抑制 1rpm 量化噪声，同时保留 0.2Hz 正弦 */
    const float vel_raw = g_motor.vel();
    static float vel_lpf = 0.0f;
    static bool  vel_lpf_init = false;

    if (!online) {
        vel_lpf = 0.0f;
        vel_lpf_init = false;
    } else if (!vel_lpf_init) {
        vel_lpf = vel_raw;          // 首次上线直接用当前值初始化
        vel_lpf_init = true;
    } else {
        vel_lpf += cfg::kVelLpfAlpha * (vel_raw - vel_lpf);
    }

    static float vel_disp = 0.0f;
    static bool  vel_disp_init = false;

    if (!online) {
        vel_disp = 0.0f;
        vel_disp_init = false;
    } else if (!vel_disp_init) {
        vel_disp = vel_raw;
        vel_disp_init = true;
    } else {
        vel_disp += cfg::kDispAlpha * (vel_raw - vel_disp);
    }

    // command 为电流给定（-16384 ~ +16384）
    float command = 0.0f;
    float speed_ref = 0.0f;               // 当前速度环参考，用于摩擦补偿
    static bool fric_active = false;

    if (online) {
        if (g_mode == CTRL_MODE_VELOCITY) {
            /* ---- 速度跟随: 单速度环 ---- */
            const float v_ref = g_vel_ref.update();          // rad/s
            speed_ref = v_ref;
            g_dbg_v_ref = v_ref;
            command = g_vel_pid.calc(v_ref, vel_lpf)
                    + cfg::kVelFF * v_ref;   // 速度前馈
        } else {
            /* ---- 位置跟随: 位置环 -> 速度环 串级 ---- */
            // 首次进入位置模式、重新上线或刚切换模式时，以当前角度为零点。
            if (g_reinit_pos != 0) {
                g_theta0 = g_motor.angle();
                g_pos_ref.reset();
                g_pos_pid.reset();
                g_reinit_pos = 0;
            }

            const float p_ref = g_theta0 + g_pos_ref.update();  // rad
            float v_ref = g_pos_pid.calc(p_ref, g_motor.angle());
            if (cfg::kPosFeedforward) {
                v_ref += g_pos_ref.derivative();             // 速度前馈, 减小相位滞后
            }
            speed_ref = v_ref;
            g_dbg_theta_ref = p_ref;
            g_dbg_v_ref     = v_ref;
            command = g_vel_pid.calc(v_ref, vel_lpf)
                    + cfg::kVelFF * v_ref;   // 速度前馈
        }
    } else {
        /* 掉线保护: 清零输出并复位控制器/参考, 防止积分饱和与解饱和冲击 */
        g_vel_pid.reset();
        g_pos_pid.reset();
        g_vel_ref.reset();
        g_pos_ref.reset();
        g_reinit_pos = 1;   // 重新上线后位置参考重新对准当前角度
        g_dbg_v_ref = 0.0f;
        fric_active = false;
        command = 0.0f;
    }

    /* 零速摩擦补偿: 参考非零但电机仍卡在 0 时，短时给一个小电流 */
    if (cfg::kUseFricComp && online) {
        if (fabsf(speed_ref) > 0.2f && fabsf(vel_lpf) < 0.15f) {
            fric_active = true;
        }
        if (fabsf(vel_lpf) > 0.4f) {
            fric_active = false;
        }
        if (fric_active) {
            command += (speed_ref > 0.0f) ? cfg::kFricComp : -cfg::kFricComp;
        }
    }

    // 前馈叠加后仍限制在电流范围内
    if (command > cfg::kCurMax) {
        command = cfg::kCurMax;
    } else if (command < cfg::kCurMin) {
        command = cfg::kCurMin;
    }

    g_motor.setCurrent(static_cast<int16_t>(command));

    /* J-Scope 调试变量 */
    g_dbg_vel      = vel_raw;
    g_dbg_vel_filt = vel_lpf;
    g_dbg_vel_disp = vel_disp;
    g_dbg_angle    = g_motor.angle();
    g_dbg_command = command;
    g_dbg_current = g_motor.current();
    g_dbg_mode    = static_cast<float>(g_mode);
    g_dbg_tick    = g_tick;

    /* 2. 下发 CAN 控制帧(每 1ms 一帧) */
    sendControlFrame();

    /* 3. 看门狗: 控制任务正常执行到这里才会喂狗 */
    Watchdog_NotifyControlLoop();
    Watchdog_Feed();
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data) {
    if (std_id == g_motor.rxId()) {       // 0x205 (ID=1)
        g_motor.decode(data);
        g_last_fb_tick = g_tick;
        g_has_fb = 1;
    }
}

/* HAL 弱回调: CAN1 RX0 收到帧 */
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        CAN_RxHeaderTypeDef rx;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) == HAL_OK) {
            CanFeedbackCallback(rx.StdId, data);
        }
    }
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* HAL 弱回调: TIM6 1kHz 周期中断 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim6) {
        MainTask();
    }
}

/* ============================== 模式切换 ============================== */
extern "C" void ControlTaskSetMode(CtrlMode_t mode) {
    g_mode = mode;
    g_vel_pid.reset();
    g_pos_pid.reset();
    g_vel_ref.reset();
    g_pos_ref.reset();
    g_reinit_pos = 1;   // 切换模式后，位置参考重新对准当前角度
}

extern "C" CtrlMode_t ControlTaskGetMode(void) {
    return g_mode;
}

/* ============================== 主循环轮询 ============================== */
extern "C" void ControlTaskPoll(void) {
    // C 板 KEY 接 PA0，按下为低电平；20ms 消抖，按一次切换一次模式。
    static GPIO_PinState stable_level = GPIO_PIN_SET;
    static GPIO_PinState last_level   = GPIO_PIN_SET;
    static uint32_t      last_change  = 0;
    static bool          key_armed    = false;

    const GPIO_PinState level = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    const uint32_t      now   = HAL_GetTick();

    if (level == GPIO_PIN_SET) {
        key_armed = true;   // 必须先松开，才允许下一次按下切换
    }

    if (level != last_level) {
        last_level  = level;
        last_change = now;
    }

    if ((now - last_change) >= 20 && level != stable_level) {
        stable_level = level;

        if (stable_level == GPIO_PIN_RESET && key_armed) {
            key_armed = false;
            if (g_mode == CTRL_MODE_VELOCITY) {
                ControlTaskSetMode(CTRL_MODE_POSITION);
            } else {
                ControlTaskSetMode(CTRL_MODE_VELOCITY);
            }
        }
    }
}
