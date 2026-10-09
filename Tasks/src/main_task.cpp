/**
*******************************************************************************
* @file      :main_task.cpp
* @brief     :电控第一次作业 业务层源文件（GPIO / 定时器 tick / 看门狗）
* @history   :
*  Version     Date            Author          Note
*  V1.0.0      2026-10-09      <your name>     1. 完成电控第一次作业
*******************************************************************************
* @attention :
*   第 2 题（定时器）与第 3 题（看门狗）只差一行：
*     - 第 2 题：回调里保留 HAL_IWDG_Refresh(&hiwdg);   → tick 持续增长
*     - 第 3 题：把 HAL_IWDG_Refresh(&hiwdg); 这行去掉   → tick 涨到约 2000 后复位归零
*   两题分开编译、分开下载、分开截图。
*******************************************************************************
*  Copyright (c) 2026 Hello World Team, Zhejiang University.
*  All Rights Reserved.
*******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "main_task.hpp"

#include "main.h" /* CubeMX 生成：含 stm32f1xx_hal.h、GPIO 定义、TIM/IWDG 句柄类型 */

/*
 * 本工程未勾选「Generate peripheral initialization as a pair of .c/.h files per peripheral」，
 * 因此 TIM/IWDG 句柄（htim2 / hiwdg）是定义在 Core/Src/main.c 里的，
 * 这里用 extern 声明后直接使用 —— 不要 #include "tim.h" / "iwdg.h"，本工程没有这两个头文件。
 * （若以后勾了 per-peripheral 文件，这两行 extern 声明也依然有效，不必改。）
 */
extern "C" {
extern TIM_HandleTypeDef htim2;
extern IWDG_HandleTypeDef hiwdg;

/*
 * 全局 tick：1 ms 自增 1。
 * 用 extern "C" 包住，保证导出符号名就叫 "tick"（不做 C++ name mangling），
 * Ozone / GDB 里才能直接按名字监视。
 * 定义在此处（Tasks 的源文件），不要在 main.c 里重复定义。
 */
volatile uint32_t tick = 0;
}

/* Exported function definitions ---------------------------------------------*/

/**
 * @brief  定时器更新中断回调（整个工程只写这一份）。
 * @note   用 C++ 编译时必须加 extern "C"，否则 HAL 的弱符号回调不会被覆盖，
 *         会出现“函数写了但没进中断”的现象。
 *         若 CubeMX 在 main.c / stm32f1xx_it.c 里也生成了同名函数，
 *         请删掉那一份，否则链接报重复定义。
 */
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    tick = tick + 1; /* 每 1 ms 加 1，Ozone 里应看到约 1000/s */

    /* ---- 第 3 题：不喂狗 ---- */
    /* 第 2 题原本在此调用 HAL_IWDG_Refresh(&hiwdg); 喂狗，
     * 第 3 题把它注释掉：IWDG 约 2 s 溢出复位，tick 涨到约 2000 后归零、循环。
     * 若要做到第 2 题的效果，把下面这行取消注释即可。 */
    // HAL_IWDG_Refresh(&hiwdg);
  }
}

void MainInit(void)
{
  /*
   * 第 1 题 GPIO：
   * CubeMX 已把 PC13 配成推挽输出（MX_GPIO_Init 里已使能 GPIOC 时钟），
   * 这里在 Tasks 的初始化里把它写成低电平。
   * F103 蓝板载 LED 为低电平点亮，所以写 RESET → 灯亮。
   * 注意：不要放在 while(1) 里翻转。
   */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*
   * 第 2 题 定时器：
   * 启动 TIM2 的更新中断，周期 1 ms（PSC=71, ARR=999）。
   * 不要用 HAL_Delay / 空循环去加 tick。
   */
  HAL_TIM_Base_Start_IT(&htim2);
}

void MainTask(void)
{
  /* 作业要求 while(1) 保持为空，此处留空 */
}
