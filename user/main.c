/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32��ʼ���Լ���ʼ����freeRTOS��h�ļ��������ȫ�ֺ궨���Լ�
  *             typedef һЩ������������
  *             stm32 initialization and start tasks in freeRTOS. h file defines
  *             relavent global macros and some common data types(defined using
  *             typedef)
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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

//�ĸ�24v ��� ���ο��� ��� 709us
//turn on 4 24v outputs consequtively with 709us interval
#define POWER_CTRL_ONE_BY_ONE_TIME 709

void BSP_init(void)
{
    //�ж��� 4
    //interrupt group 4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    //��ʼ���δ�ʱ��
    //initialize the ticking clock
    delay_init(configTICK_RATE_HZ);
    //��ˮ�ƣ����̵Ƴ�ʼ��
    //initialize the led that changes between red and green colors
    led_configuration();
    //stm32 �����¶ȴ�������ʼ��
    //initialize the stm32 onboard temperature sensor
    temperature_ADC_init();
#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    //stm32 �������������ʼ��
    //initialize the stm32 random number generator
    RNG_init();
#endif
    //24������ƿ� ��ʼ��
    // initialize the 24 output control ports
    power_ctrl_configuration();
    //Ħ���ֵ��PWM��ʼ��
    //initialize the PWM of the friction wheel motor
    fric_PWM_configuration();
    //��������ʼ��
    //initialize the buzzer
    buzzer_init(30000, 90);
    //����IO��ʼ��
    //initialize the laser IO
    laser_configuration();
    //��ʱ��6 ��ʼ��
    //initialize timer 6
    TIM6_Init(60000, 90);
    //CAN�ӿڳ�ʼ��
    //initialize the CAN interface/port
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
    CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);

    //24v ��� �����ϵ�
    //power the 24v outputs consequtively
    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_on(i);
        delay_us(POWER_CTRL_ONE_BY_ONE_TIME);
    }
    //ң������ʼ��
    //initialize the remote control
    remote_control_init();
    //flash��ȡ��������У׼ֵ�Żض�Ӧ����
    //flash reading function, assign caliberated values to corresponding parameters
    cali_param_init();
}
