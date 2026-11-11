/**
 *******************************************************************************
 * @file      : fsm_ins.cpp
 * @brief     : 状态机实例化实现——新人填写
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空状态机实例化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "fsm_ins.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
dart::MainFsm *CreateMainFsm(void)
{
  /* TODO【新人任务】:
   * 返回顶层状态机单例。
   * 1. 在你的 Dart-Components/fsm/ 下设计 MainFsm 类（见 main_fsm.hpp TODO）。
   * 2. 静态定义对象（或用 getInstance），返回其地址。
   */
  return nullptr;
}

dart::BeltFsm *CreateBeltFsm(void)
{
  /* TODO【新人任务】: 返回同步带模块单例，做法同 CreateMainFsm()。 */
  return nullptr;
}

dart::ScrewFsm *CreateScrewFsm(void)
{
  /* TODO【新人任务】: 返回丝杆模块单例，做法同 CreateMainFsm()。 */
  return nullptr;
}

dart::ShootFsm *CreateShootFsm(void)
{
  /* TODO【新人任务】: 返回击发模块单例，做法同 CreateMainFsm()。 */
  return nullptr;
}

/* Private function definitions ----------------------------------------------*/
