/******************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief      设备离线判断任务，通过freeRTOS滴答时间作为系统时间，设备获取数据后
  *             调用DetectHook记录对应设备的时间，在该任务会通过判断记录时间与系统
  *             时间之差来判断掉线，同时将最高的优先级的任务通过LED的方式改变，包括
  *             八个流水灯显示SBUB遥控器，三个云台上的电机，4个底盘电机，另外也通过
  *             红灯闪烁次数来显示错误码。
  *
  *
  * 		     		Set to off-line mode, using the freeRTOS tick time as the 
	*							internal clock of the system timing, after getting the data, using 
	*							DetectHook to record the corresponding time, this task will detect 
	*							and compare recorded time with the system time to determine if it is off-line. 
	*							At the same time, change the most prioritized task by changing the LEDs, 
	*							including 8 Flow LEDS to display the SBUB controller, 3 motors on the gimbal, 
	*							4 motors on the chassis. In addition, showing the error by flashing the Red LEDs.  
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI*****************************/
	
	
#include "Detect_Task.h"

#include "led.h"

#include "CAN_Receive.h"
#include "Remote_Control.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//红灯闪，灭函数，切换闪灭
//Switch, On, Off
#define DETECT_LED_R_TOGGLE() led_red_toggle()// for switching
#define DETECT_LED_R_ON() led_red_on()// for on
#define DETECT_LED_R_OFF() led_red_off() // off

//流水灯闪灭函数
//Flow LEDs On/Off
#define DETECT_FLOW_LED_ON(i) flow_led_on(i) //flow_led on
#define DETECT_FLOW_LED_OFF(i) flow_led_off(i) //flow_led off 

#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//初始化错误列表
//Init errorList 
static void DetectInit(uint32_t time);

//显示优先级最高的错误，传入的参数为 显示的错误的错误码
//Display the prioritized error, the parameter that is being past is number of the error
static void DetectDisplay(uint8_t num);

static error_t errorList[errorListLength + 1];

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t DetectTaskStack;
#endif

//掉线判断任务
//Detect if online or offline
void DetectTask(void *pvParameters)
{
    static uint32_t systemTime;
    systemTime = xTaskGetTickCount();
    
    //初始化 
		//Initialization
    DetectInit(systemTime);

    //空闲一段时间 
		//Delay
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        static uint8_t error_num_display = 0;
        systemTime = xTaskGetTickCount(); //setting the clock to tick time 

        error_num_display = errorListLength;
        errorList[errorListLength].isLost = 0;
        errorList[errorListLength].errorExist = 0;

        for (int i = 0; i < errorListLength; i++)
        {
            //未使能，跳过
            //If this error isnt happening, continue
            if (errorList[i].enable == 0)
            {
                continue;
            }

            //判断掉线
            //Detetmine if offline
            if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].errorExist == 0)
                {
                    //记录错误以及掉线时间
                    //Record the error and offline time
                    errorList[i].isLost = 1;
                    errorList[i].errorExist = 1;
                    errorList[i].Losttime = systemTime;
                }
                //判断错误优先级， 保存优先级最高的错误码
                //Determine the priority level of the error, store the error number of highest priority error
                if (errorList[i].Priority > errorList[error_num_display].Priority)
                {
                    error_num_display = i;
                }
                //记录列表的存在错误，
                //Record the error in the list
                errorList[errorListLength].isLost = 1;
                errorList[errorListLength].errorExist = 1;
                //如果提供解决函数，运行解决函数
                // If solvelostFun is provided, run the function
                if (errorList[i].solveLostFun != NULL)
                {
                    errorList[i].solveLostFun();
                }
            }
            else if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)
            {
                //刚刚上线，可能存在数据不稳定，只记录不丢失，
                //If online data is not stable, only record but don't delete
                errorList[i].isLost = 0;
                errorList[i].errorExist = 1;
            }
            else
            {
                errorList[i].isLost = 0;
                //判断是否存在数据错误
                //Determine if error exists in the data
                if (errorList[i].dataIsError)
                {
                    errorList[i].errorExist = 1;
                }
                else
                {
                    errorList[i].errorExist = 0;
                }
                //计算频率
                //Calculate frequency
                if (errorList[i].newTime > errorList[i].lastTime)
                {
                    errorList[i].frequency = configTICK_RATE_HZ / (fp32)(errorList[i].newTime - errorList[i].lastTime);
                }
            }
        }

        DetectDisplay(error_num_display + 1);
        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        DetectTaskStack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

//返回对应的设备是否存在错误
//Return the corresponding error of the part
bool_t toe_is_error(uint8_t err)
{
    return (errorList[err].errorExist == 1);
}

//设备接收数据钩子函数
//The parts recieve the data from the hook 
void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime = xTaskGetTickCount();
    //更新丢失情况
    //Update what isLost
    if (errorList[toe].isLost)
    {
        errorList[toe].isLost = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
    //判断数据是否错误
    //Determine if the data is wrong
    if (errorList[toe].dataIsErrorFun != NULL)
    {
        if (errorList[toe].dataIsErrorFun())
        {
            errorList[toe].errorExist = 1;
            errorList[toe].dataIsError = 1;

            if (errorList[toe].solveDataErrorFun != NULL)
            {
                errorList[toe].solveDataErrorFun();
            }
        }
        else
        {
            errorList[toe].dataIsError = 0;
        }
    }
    else
    {
        errorList[toe].dataIsError = 0;
    }
}
const error_t *getErrorListPoint(void)
{
    return errorList;
}
static void DetectDisplay(uint8_t num)
{
    static uint8_t last_num = errorListLength + 1;
    uint8_t i = 0;

    //8个流水显示前八的错误码的情况，包括SBUS遥控器，yaw.pitch,triger云台，4个底盘电机
	//The 8 Flow LEDs show the first 8 errors- serial bus controller, gimbal:( the yaw, pitch, trigger),chassis:( and 4 chassis motors)
    for (i = 0; i <= ChassisMotor4TOE; i++)
    {
        if (errorList[i].errorExist)
        {
            DETECT_FLOW_LED_OFF(i);
        }
        else
        {
            DETECT_FLOW_LED_ON(i);
        }
    }

    //错误码 通过红灯闪烁次数来判断
    //Show the error code by flashing Red LED
    if (num == errorListLength + 1)
    {
        DETECT_LED_R_OFF();
        last_num = errorListLength + 1;
    }
    else
    {
        static uint8_t i = 0, led_flag = 0, cnt_num = 0, time = 0;
        //记录最新的最高优先级的错误码，等下一轮闪烁
        //Record the newest, highest priority error code, wait for next round of LEDs
        if (last_num != num)
        {
            last_num = num;
        }

        if (cnt_num == 0)
        {
            //cnt_num 记录还有几次闪烁，到0后，灭一段时间才开始下一轮
            //Record the remaining flashes, when it hits 0, delay some time, then do again
            time++;
            if (time > 50)
            {
                time = 0;
                cnt_num = last_num;
            }
            return;
        }

        if (i == 0)
        {

            DETECT_LED_R_TOGGLE();
            if (led_flag)
            {
                //红灯闪灭各一次，将要剩余次数减一
                //When red toggles once, remove one from reamining flahses
                led_flag = 0;
                cnt_num--;
            } 
            else
            {
                led_flag = 1;
            }
        }

        //i为计时次数，20为半个周期，切换一次红灯闪灭
        //i is how many times it flahses, i = 20 is half of the period, toggle red light from on to off 
        i++;

        if (i > 20)
        {
            i = 0;
        }
    }
}

static void DetectInit(uint32_t time)
{
    //设置离线时间，上线稳定工作时间，优先级 offlineTime onlinetime priority
    //Init offline time, stabilize onlinetime, Priority
    uint16_t setItem[errorListLength][3] =
        {
						//offlinetime, onlinetime, priority
            {30, 40, 15}, //SBUS Highest Priority!
            {2, 3, 14},   //yaw
            {2, 3, 13},   //pitch
            {10, 10, 12}, //trigger
            {10, 10, 11}, //motor1
            {10, 10, 10}, //motor2
            {10, 10, 9},  //motor3
            {10, 10, 8},  //motor4 Lowest Priority

        };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime = setItem[i][1];
        errorList[i].Priority = setItem[i][2];
        errorList[i].dataIsErrorFun = NULL;
        errorList[i].solveLostFun = NULL;
        errorList[i].solveDataErrorFun = NULL;

        errorList[i].enable = 1;
        errorList[i].errorExist = 1;
        errorList[i].isLost = 1;
        errorList[i].dataIsError = 1;
        errorList[i].frequency = 0.0f;
        errorList[i].newTime = time;
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

    errorList[DBUSTOE].dataIsErrorFun = RC_data_is_error;
    errorList[DBUSTOE].solveLostFun = slove_RC_lost;
    errorList[DBUSTOE].solveDataErrorFun = slove_data_error;

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
    errorList[YawGimbalMotorTOE].solveLostFun = GIMBAL_lose_slove;
    errorList[PitchGimbalMotorTOE].solveLostFun = GIMBAL_lose_slove;
#endif
}






