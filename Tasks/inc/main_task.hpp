/**
*******************************************************************************
* @file      :main_task.hpp
* @brief     :电控第一次作业 业务层头文件（GPIO / 定时器 tick / 看门狗）
* @history   :
*  Version     Date            Author          Note
*  V1.0.0      2026-10-09      <your name>     1. 完成电控第一次作业
*******************************************************************************
* @attention :
*******************************************************************************
*  Copyright (c) 2026 Hello World Team, Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_TASK_HPP__
#define __MAIN_TASK_HPP__

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Exported variables --------------------------------------------------------*/

/**
 * @brief  1 ms 自增一次的全局计数变量。
 * @note   作业硬性要求：变量名必须叫 tick，类型必须是全局的 volatile uint32_t，
 *         否则 Ozone 的 Watched Data / Data Sampling 看不到它的实时变化。
 */
extern volatile uint32_t tick;

/* Exported function prototypes ----------------------------------------------*/

/**
 * @brief  业务初始化。
 * @note   在 main.c 的 USER CODE BEGIN 2 处调用。
 *         第 1 题：把 PC13 写成低电平点亮板载 LED；
 *         第 2 题：启动 TIM2 的 1 ms 更新中断。
 */
void MainInit(void);

/**
 * @brief  主循环任务。
 * @note   本次作业要求 while(1) 保持为空，此函数保留以备后续扩展，
 *         当前不调用即可（定义保留不会报错）。
 */
void MainTask(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MAIN_TASK_HPP__ */
