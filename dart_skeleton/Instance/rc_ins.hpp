/**
 *******************************************************************************
 * @file      : rc_ins.hpp
 * @brief     : 遥控器实例化接口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空遥控器实例化
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_INSTANCE_RC_INS_HPP_
#define DART_INSTANCE_RC_INS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"

namespace hw_rc = hello_world::remote_control;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
hw_rc::DT7 *CreateRc(void);

#endif /* DART_INSTANCE_RC_INS_HPP_ */
