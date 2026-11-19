/**
 *******************************************************************************
 * @file      : fsm.hpp
 * @brief     : 状态机基类（生命状态机，已提供）
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      Immortal-Fates  1. <note>
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：提供生命状态机
 *                                                 （Dead/Resurrection/Working），
 *                                                 新人只需实现 Working 态工作逻辑
 *******************************************************************************
 * @attention :
 *  本文件为**已提供组件**，不需要新人修改。
 *  生命状态机（Death / Resurrection / Working）由基类统一处理：
 *    - 上电默认 Dead；
 *    - 调用 updateWorkState() 后：Dead → Resurrection（默认 1s 自检）→ Working；
 *    - resetWorkState() 可把状态打回 Dead。
 *  新人的模块 / 顶层状态机可直接继承 dart::Fsm，在 update() 中调用
 *  updateWorkState()，然后只处理 Working 态下的工作逻辑。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_COMPONENTS_FSM_HPP_
#define DART_COMPONENTS_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "allocator.hpp"
#include "tick.hpp"

namespace dart
{
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
class Fsm : public hello_world::MemMgr
{
   public:
    // 整车 / 模块生命状态
    enum WorkState : uint8_t {
        kWorkStateDead,          ///< 死亡状态（上电 / 复位后）
        kWorkStateResurrection,  ///< 复活状态（自检中）
        kWorkStateWorking,       ///< 工作状态
    };

    static constexpr float kCtrlPeriod = 0.002f;  ///< 控制周期，单位 s

    WorkState work_state() const { return work_state_; }
    WorkState last_work_state() const { return last_work_state_; }
    uint32_t getCurrentTickMs() const { return hello_world::tick::GetTickMs(); }

    /**
     * @brief       复位生命状态（回到 Dead，重新走一遍复活自检）
     * @retval      None
     */
    void resetWorkState()
    {
        work_state_ = kWorkStateDead;
        last_work_state_ = kWorkStateDead;
        resurrection_tick_ = 0.0f;
    }

    /**
     * @brief       生命状态迁移（已提供，直接周期调用即可）
     * @retval      None
     * @note        Dead → Resurrection（kResurrectionTime_ 秒）→ Working；
     *              状态切换会记录 last_work_state_。
     */
    void updateWorkState()
    {
        WorkState current_state = work_state_;
        if (current_state == kWorkStateDead) {
            current_state = kWorkStateResurrection;
            resurrection_tick_ = 0.0f;
        } else if (current_state == kWorkStateResurrection) {
            resurrection_tick_ += kCtrlPeriod;
            if (resurrection_tick_ >= kResurrectionTime_) {
                current_state = kWorkStateWorking;
            }
        } else if (current_state == kWorkStateWorking) {
            // 工作状态下保持
        } else {
            // 其他状态视为死亡
            current_state = kWorkStateDead;
        }
        setWorkState(current_state);
    }

   protected:
    void setWorkState(WorkState state)
    {
        if (state != work_state_) {
            last_work_state_ = work_state_;
            work_state_ = state;
        }
    }

    WorkState work_state_ = kWorkStateDead;       ///< 当前生命状态
    WorkState last_work_state_ = kWorkStateDead;  ///< 上一生命状态
    float resurrection_tick_ = 0.0f;              ///< 复活计时
    float kResurrectionTime_ = 1.0f;  ///< 复活自检时长，单位 s（子类可覆盖）
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dart

#endif /* DART_COMPONENTS_FSM_HPP_ */
