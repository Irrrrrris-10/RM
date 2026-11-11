/**
 *******************************************************************************
 * @file      : shoot_fsm.hpp
 * @brief     : 击发模块（舵机）——新人填写
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空击发舵机控制逻辑
 *******************************************************************************
 * @attention :
 *  本文件已掏空。新人需自行设计并声明击发模块类（建议 namespace dart，类名
 *  ShootFsm），负责击发舵机的开 / 关控制。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_COMPONENTS_SHOOT_FSM_HPP_
#define DART_COMPONENTS_SHOOT_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

/* ============================================================================
 * 【新人任务】设计击发模块类 ShootFsm（namespace dart）。
 *
 * 硬件：
 *   - 击发舵机接 TIM2 CH3，参数见 Dart-Configs/motor_configs.hpp 的 kServoShoot；
 *     精简版舵机驱动已提供：inc/servo.hpp（Servo::init/setAngle/enable/disable）。
 *   - 角度含义（新镖架）：180° = 扳机下拉（击发）；95° = 扳机复位（准备）。
 *
 * 你要做：
 *   1. 注册击发舵机对象（由 Instance/motor_ins.cpp 创建），实现击发 / 复位动作切换。
 *   2. 提供 genCmd() 形式的目标更新接口与 run() 周期执行接口。
 *   3. 安全：待机 / 安全模式下舵机必须回到复位角（95°），不得保持在击发位。
 *
 * 约束 / 提示：
 *   - 生命状态（Dead/Resurrection/Working）已由 Dart-Components/fsm/fsm.hpp 的
 *     dart::Fsm 基类提供：可继承并调用 updateWorkState()，不需要自己实现；
 *     你只需写工作状态下的舵机动作逻辑。
 *   - 舵机驱动已提供（精简版）：inc/servo.hpp。
 *   - 舵机的开 / 关时序（下拉保持多久）由顶层状态机控制，本模块只负责执行。
 *   - 参考：code_control/Dart2025/Dart-Components/modules/shoot/shoot_fsm.cpp。
 * ==========================================================================*/

#endif /* DART_COMPONENTS_SHOOT_FSM_HPP_ */
