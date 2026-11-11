/**
 *******************************************************************************
 * @file      : fsm_ins.hpp
 * @brief     : 状态机实例化接口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空状态机实例化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_INSTANCE_FSM_INS_HPP_
#define DART_INSTANCE_FSM_INS_HPP_

/* Includes ------------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* 前向声明：类由你在 Dart-Components/ 下自行设计（名字可改，注意同步修改本文件）。 */
namespace dart
{
class MainFsm;
class BeltFsm;
class ScrewFsm;
class ShootFsm;
}  // namespace dart

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
dart::MainFsm *CreateMainFsm(void);    ///< 顶层单发状态机
dart::BeltFsm *CreateBeltFsm(void);    ///< 同步带模块
dart::ScrewFsm *CreateScrewFsm(void);  ///< 丝杆模块
dart::ShootFsm *CreateShootFsm(void);  ///< 击发模块

#endif /* DART_INSTANCE_FSM_INS_HPP_ */
