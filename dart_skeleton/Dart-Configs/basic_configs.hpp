/**
 *******************************************************************************
 * @file      : basic_configs.hpp
 * @brief     : 基础配置（控制周期 / 串口句柄 / CAN）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：只保留单发所需基础配置
 *******************************************************************************
 * @attention :
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_CONFIGS_BASIC_CONFIGS_HPP_
#define DART_CONFIGS_BASIC_CONFIGS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "gpio.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"

/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
static const float kCtrlPeriod = 0.002f;  ///< 控制周期，单位 s（TIM3，500Hz）

static const uint32_t kMotorOfflineTickThres = 50;  ///< 电机离线检测阈值，单位 ms

static UART_HandleTypeDef* const rc_uart_ptr = &huart1;  ///< 遥控器 DBUS 使用的串口

const uint8_t kCanBufLen = 8;

/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

#endif /* DART_CONFIGS_BASIC_CONFIGS_HPP_ */
