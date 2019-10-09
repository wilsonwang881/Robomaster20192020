/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
  * @brief      һ����ͨ������������豸�޴����̵�1Hz��˸,Ȼ���ȡ��̬��//a normal heart beats program, if the equipement has no error, green light on and off with 1HZ, and obtains positional angle
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
 
#include "User_Task.h"
#include "main.h"
 
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
 
#include "led.h"
 
#include "Detect_Task.h"
#include "INS_Task.h"
 
#define user_is_error() toe_is_error(errorListLength)
 
#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t UserTaskStack;
#endif
 
//��̬�� ��λ�� //positional angle, the unit is degree
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};
 
void UserTask(void *pvParameters)
{
 
    const volatile fp32 *angle;
    //��ȡ��̬��ָ�� //obtain the pointer of positional angle
    angle = get_INS_angle_point();
    while (1)
    {
 
        //��̬�� ��rad ��� �ȣ����������̬�ǵĵ�λΪ�ȣ������ط�����̬�ǣ���λ��Ϊ���� //positional angle change radian to degree, only the unit of potitional angle here is degree, all other positional angles�� unit is radian
        angle_degree[0] = (*(angle + INS_YAW_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[1] = (*(angle + INS_PITCH_ADDRESS_OFFSET)) * 57.3f;
        angle_degree[2] = (*(angle + INS_ROLL_ADDRESS_OFFSET)) * 57.3f;
 
        if (!user_is_error())
        {
            led_green_on();
        }
        vTaskDelay(500);
        led_green_off();
        vTaskDelay(500);
#if INCLUDE_uxTaskGetStackHighWaterMark
        UserTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
 
