/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。h文件定义相关全局宏定义以及
  *             typedef 一些常用数据类型
  *             stm32 initialization and start tasks in freeRTOS. h file defines
  *             relavent global macros and some common data types(defined using
  *             typedef)
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "main.h"

#include "stm32f4xx.h"

#include "adc.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "flash.h"
#include "fric.h"
#include "laser.h"
#include "led.h"
#include "power_ctrl.h"
#include "rc.h"
#include "rng.h"
#include "sys.h"
#include "timer.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "calibrate_task.h"
#include "remote_control.h"
#include "start_task.h"

void BSP_init(void);

int main(void)
{
    BSP_init();
    delay_ms(100);
    startTast();
    vTaskStartScheduler();
    while (1)
    {
        ;
    }
}

//四个24v 输出 依次开启 间隔 709us
//turn on 4 24v outputs consequtively with 709us interval
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //中断组 4
    //interrupt group 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //初始化滴答时钟
    //initialize the ticking clock
    delay_init(configTICK_RATE_HZ);
    //流水灯，红绿灯初始化
    //initialize the led that changes between red and green colors
    led_configuration();
    //stm32 板载温度传感器初始化
    //initialize the stm32 onboard temperature sensor
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 随机数发生器初始化
    //initialize the stm32 random number generator
    RNG_init();
#endif
    //24输出控制口 初始化
    // initialize the 24 output control ports
    power_ctrl_configuration();
    //摩擦轮电机PWM初始化
    //initialize the PWM of the friction wheel motor
    fric_PWM_configuration();
    //蜂鸣器初始化
    //initialize the buzzer
    buzzer_init(30000, 90);
    //激光IO初始化
    //initialize the laser IO
    laser_configuration();
    //定时器6 初始化
    //initialize timer 6
    TIM6_Init(60000, 90);
    //CAN接口初始化
    //initialize the CAN interface/port
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v 输出 依次上电
    //power the 24v outputs consequtively
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //遥控器初始化
    //initialize the remote control
    remote_control_init();
    //flash读取函数，把校准值放回对应参数
    //flash reading function, assign caliberated values to corresponding parameters
    cali_param_init();
}
