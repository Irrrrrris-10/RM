/*
 * 设备实例化（考核自行实现）。创建右/左同步带和丝杆对象；舵机复用已提供驱动，适配当前定时器计数频率。requested_deg 是请求，不是舵机位置反馈。
 */
#include "motor_ins.hpp"
#include "motor_configs.hpp"
#include "control_config.hpp"
#include "tim.h"
struct ServoDebug {
 uint32_t version,ready,requests,writes,safe_requests,skipped,ccr3,arr,psc,ccer,cr1;
 float requested_deg;
};
volatile ServoDebug g_servo_debug{};
namespace {
dart::Motor right(1,1,3591.0f/187,20,16384),left(2,-1,3591.0f/187,20,16384),screw(3,-1,36,10,10000);
hw_servo::Servo shoot;
bool servo_ready=false;
}
dart::Motor* CreateMotorBeltRhs(){return &right;}
dart::Motor* CreateMotorBeltLhs(){return &left;}
dart::Motor* CreateMotorScrew(){return &screw;}
dart::Motor* MotorByIndex(unsigned i){return i==0?&right:(i==1?&left:(i==2?&screw:nullptr));}
hw_servo::Servo* CreateServoShoot(){return &shoot;}
bool InitServoShoot(bool enable) {
 g_servo_debug.version=1;
 if(!enable)return false;
 // Preserve current hardware's 100kHz timer and 50..250 pulse counts.
 // Adapt frequency to the provided driver's 1MHz ARR arithmetic without editing it.
 const uint32_t timer_clock=HAL_RCC_GetPCLK1Freq()*((RCC->CFGR&RCC_CFGR_PPRE1)?2u:1u);
 if(timer_clock/(htim2.Init.Prescaler+1)!=100000u)return false;
 auto effective=kServoShoot;
 effective.operating_freq=kServoShoot.operating_freq*10; // 100kHz counts versus driver's1MHz assumption
 shoot.init(&htim2,TIM_CHANNEL_3,effective,dart::config::kServoSafeDeg);
 __HAL_TIM_SET_COUNTER(&htim2,0);
 // Servo::init() already starts PWM. A second HAL start returns HAL_ERROR
 // because the channel is BUSY; verify the running channel instead.
 servo_ready=HAL_TIM_GetChannelState(&htim2,TIM_CHANNEL_3)==HAL_TIM_CHANNEL_STATE_BUSY &&
   (htim2.Instance->CCER&TIM_CCER_CC3E)!=0 &&
   (htim2.Instance->CR1&TIM_CR1_CEN)!=0;
 g_servo_debug.ready=servo_ready;
 return servo_ready;
}
// 即使未 ready 也记录请求角度；writes/skipped 才能区分是否真正写入比较寄存器。
void ApplyServoShoot(float angle,bool permit_fire){
 g_servo_debug.requests=g_servo_debug.requests+1;
 const float target=permit_fire?angle:dart::config::kServoSafeDeg;
 g_servo_debug.requested_deg=target;
 if(target==dart::config::kServoSafeDeg)g_servo_debug.safe_requests=g_servo_debug.safe_requests+1;
 if(servo_ready){shoot.setAngle(target);g_servo_debug.writes=g_servo_debug.writes+1;}
 else g_servo_debug.skipped=g_servo_debug.skipped+1;
 g_servo_debug.ready=servo_ready;
 g_servo_debug.ccr3=htim2.Instance->CCR3;g_servo_debug.arr=htim2.Instance->ARR;
 g_servo_debug.psc=htim2.Instance->PSC;g_servo_debug.ccer=htim2.Instance->CCER;
 g_servo_debug.cr1=htim2.Instance->CR1;
}
bool ServoShootReady(){return servo_ready;}
