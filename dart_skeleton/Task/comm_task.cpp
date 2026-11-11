/**
 *******************************************************************************
 * @file      : comm_task.cpp
 * @brief     : 通讯任务实现——新人填写
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空通讯分发与接线，
 *                                                 CAN 底层也由新人自己实现
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "comm_task.hpp"

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported function definitions ---------------------------------------------*/
void CommTaskInit()
{
  /* TODO【新人任务】:
   * 通讯层一次性初始化（**骨架不提供 CAN 组件，CAN 底层要自己写**）：
   *   1. CPU 串口（DBUS）接收接线：对 rc_uart_ptr（&huart1，见 Dart-Configs/
   *      basic_configs.hpp）调用 HAL_UARTEx_ReceiveToIdle_DMA，缓冲区长度 18。
   *   2. CAN1 底层初始化（自己实现，可直接用 HAL）：
   *        配置滤波器（全收）→ HAL_CAN_ConfigFilter(&hcan1, ...) →
   *        HAL_CAN_Start(&hcan1) →
   *        HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING)。
   *   3. 独立看门狗 IWDG 已在 CubeMX 中初始化（MX_IWDG_Init），无需重复初始化。
   *
   * 提示：可以新建自己的目录（如 Comm/）放 CAN / 电机协议代码；新目录记得加入
   *       CMakeLists.txt 的 set(user_folders ...)。
   */
}

void CommTask()
{
  /* TODO【新人任务】:
   * 每个通讯周期（2ms，与 MainTask 一起由 TIM3 调用）执行：
   *   1. 把你自己的电机协议 encode 出的 8 字节填入 0x200 共享帧并发送
   *      （同步带左/右、丝杆同属 0x200，每台占 2 字节，注意先 encode 后一起发；
   *      发送可直接 HAL_CAN_AddTxMessage，或封装成自己的发送函数）。
   *   2. 其他需要周期下发的通讯（本骨架无）。
   *
   * !!! 安全红线 !!!
   *   - 这里发送的只能是控制逻辑按遥控器指令生成的目标值；
   *     不得无条件发送常值/递增输入让 M3508 / M2006 持续转动。
   *   - 遥控器离线 / 安全档时，控制逻辑应把目标清零后再发送。
   */
}

/* Private function definitions ----------------------------------------------*/
/* TODO【新人任务】:
 * 实现以下两个中断回调（都需要自己写，建议放在本文件）：
 *
 * 1) CAN 接收：HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
 *    - hcan == &hcan1 时取一帧（HAL_CAN_GetRxMessage），按 StdId 分发：
 *      0x201/0x202/0x203 → 调用你自己的电机反馈解析函数；
 *    - 处理完重新使能接收中断（HAL_CAN_ActivateNotification）。
 *
 * 2) UART 空闲接收：HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
 *    - huart == rc_uart_ptr 时收满 18 字节 → CreateRc()->decode(rx_buf)；
 *    - 刷新 IWDG：HAL_IWDG_Refresh(&hiwdg)（含 "iwdg.h"）；
 *    - 重新启动下一轮空闲 + DMA 接收。
 *
 * 注意：
 *   - HAL 回调是 C 链接（包含 "usart.h" / "can.h" 即可获得正确链接），
 *     不要写在 class 内；
 *   - 安全：若遥控器数据长时间未更新（离线），必须让控制逻辑把电机输出归零，
 *     不得在失联状态下继续驱动 M3508 / M2006。
 */
