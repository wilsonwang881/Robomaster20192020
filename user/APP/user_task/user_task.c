/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       user_task.c/h
<<<<<<< HEAD
  * @brief      一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角//a normal heart beats program, if the equipement has no error, green light on and off with 1HZ, and obtains positional angle
  * @note       
=======
  * @brief      一个普通心跳程序，如果设备无错误，绿灯1Hz闪烁,然后获取姿态角
  *             a normal heartbeat program
  *             if the device has no errors, the green light blinks at 1Hz,
  *             then acquire the posture angle
  * @note
>>>>>>> 65bd981e23dda075f84854d36b7c6abccdc87ffa
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成 complete
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
<<<<<<< HEAD
 
//姿态角 单位度 //positional angle, the unit is degree
=======

//姿态角 单位度
//posture angle in degrees
>>>>>>> 65bd981e23dda075f84854d36b7c6abccdc87ffa
fp32 angle_degree[3] = {0.0f, 0.0f, 0.0f};
 
void UserTask(void *pvParameters)
{
 
    const volatile fp32 *angle;
<<<<<<< HEAD
    //获取姿态角指针 //obtain the pointer of positional angle
    angle = get_INS_angle_point();
    while (1)
    {
 
        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度 //positional angle change radian to degree, only the unit of potitional angle here is degree, all other positional angles’ unit is radian
=======
    //获取姿态角指针
    //acquire the posture angle pointer
    angle = get_INS_angle_point();
    while (1)
    {

        //姿态角 将rad 变成 度，除这里的姿态角的单位为度，其他地方的姿态角，单位均为弧度
        //
>>>>>>> 65bd981e23dda075f84854d36b7c6abccdc87ffa
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
 
