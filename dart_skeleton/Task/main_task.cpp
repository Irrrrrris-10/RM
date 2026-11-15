#include "main_task.hpp"
#include "comm_task.hpp"
#include "fsm_ins.hpp"
#include "main_fsm.hpp"
#include "tim.h"
#include "main.h"

void MainTaskInit() {
  CreateMainFsm();
  CommTaskInit();
  if (HAL_TIM_Base_Start_IT(&htim3) != HAL_OK) Error_Handler();
}
void MainTask() {
  CreateMainFsm()->update(GetRcSnapshot());
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer) {
  if (timer != &htim3) return;
  MainTask();
  CommTask();
}
