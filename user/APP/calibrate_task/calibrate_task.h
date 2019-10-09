/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      完成校准相关设备的数据，包括云台，陀螺仪，加速度计，磁力计
  *             云台校准主要是中值，最大最小相对角度，陀螺仪主要校准零漂
  *             加速度计和磁力计只是写好接口函数，加速度计目前没有必要校准，
  *             磁力计尚未使用在解算算法中。
  *             Finish calibration-related devices' data, including the tripod,
  *             gyroscope, accelerometer, magnetometer
  *             Tripod calibration: middle value, min and max relative angle
  *             Gyroscope caliberation: zero shift
  *             Accelerometer and magnetometer: have the interface function ready
  *             Accelerometer: not necessary for calibration for now
  *             Magnetometer: not currently used in the decoding algorithm
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
#ifndef CALIBRATE_TASK_H
#define CALIBRATE_TASK_H
#include "main.h"

#define imu_start_buzzer() buzzer_on(95, 10000) //IMU元件校准的蜂鸣器的频率以及强度
//the frequency and the intensity of the buzzer for IMU component calibration

#define gimbal_start_buzzer() buzzer_on(31, 20000) //云台校准的蜂鸣器的频率以及强度
//the frequency and the intensity of the buzzer for tripod calibration

#define cali_buzzer_off() buzzer_off() //关闭蜂鸣器
//turn off the buzzer

#define cali_get_mcu_temperature() get_temprate() //获取stm32上的温度 作为IMU校准的环境温度
//get the temperature of stm32
//use it as the environment temperature for IMU calibration

#define cali_flash_read(address, buf, len) flash_read((address), (buf), (len))                  //flash 读取函数
//reading function for flash

#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len)) //flash 写入函数
//writing function for flash

#define get_remote_ctrl_point_cali() get_remote_control_point() //获取遥控器的结构体指针
//get the pointer for the struct of the remote control

#define gyro_cali_disable_control() RC_unable()                 //陀螺仪校准时候掉线遥控器
//offline the remote control during gyroscope calibra

//陀螺仪校准函数
//gyroscope calibration function
#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))

//设置陀螺仪校准值
//set the calibration value for the gyroscope
#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_23 //要写入flash扇区，目前是最后一块，
//the sector to be written to flash,currently it is the last sector

#define GYRO_CONST_MAX_TEMP 45.0f //陀螺仪控制恒温 最大控制温度
//a constant temperature for gyroscope control
//the max control temperature

#define CALI_FUNC_CMD_ON 1   //校准函数，使能校准
//calibration function enable

#define CALI_FUNC_CMD_INIT 0 //校准函数，传递flash中的校准参数
//calibration function, passing calibration parameters in flash

#define CALIBRATE_CONTROL_TIME 1 //校准任务函数运行的周期 为1ms
//period for running the calibration function, set to 1ms

#define CALI_SENSOR_HEAD_LEGHT 1 //校准结构体的表头 为cali_sensor_t的前部 详看 cali_sensor_t的描述 大小 1 代表一个32位数据
//calibration struct header
//it is the front part of cali_sensor_t
//refer to cali_sensor_t for more information
//size one represents 1 32-bit number

#define SELF_ID 0              //表头中的ID
//header id

#define FIRMWARE_VERSION 12345 //表头中的硬件版本号 目前随意写的
//hardware version number in the header, currently it is a random number

#define CALIED_FLAG 0x55       //代表标记已经校准完毕
//represents the calibration is complete

#define CALIBRATE_END_TIME 20000 //遥控器校准时长20s 超过20s需要重新操作
//remote control calibration lasts for 20 seconds
//require restarting the operation if it takes longer than 20 seconds

#define RC_CALI_BUZZER_MIDDLE_TIME 10000 //校准时，改变蜂鸣器频率成云台的高频声音，有助于提醒20s校准时间马上完毕
//during calibration, change the buzzer frequency to the high frequency sound of the tripod
//this helps with alarming when the 20 seconds for calibration is due soon

#define rc_cali_buzzer_middle_on() gimbal_start_buzzer()

#define RC_CALI_BUZZER_START_TIME 0 //校准时，改变蜂鸣器频率成IMU的低频声音，代表校准时间20s开始计时
//during calibration, change the buzzer frequency to the low frequency of IMU
//this represents the 20 seconds timing for calibration starts

#define rc_cali_buzzer_start_on() imu_start_buzzer()

#define RCCALI_BUZZER_CYCLE_TIME 400  //校准选择时间20s，蜂鸣器断续发声周期时间
//calibration select time, 20 seconds
//the peroid for the buzzer to make a sound

#define RC_CALI_BUZZER_PAUSE_TIME 200 //校准选择时间20s，蜂鸣器断续发声停声时间
//calibration seleect time, 20 seconds
//the period for the buzzer to make a sound

#define RC_CALI_VALUE_HOLE 600        //遥控器外八或者内八 阈值判定， 遥控器摇杆最大是 660 只要大于630 就认为到最大
//the threshold value of determining whether the remote control joystick leans
//towards inner or outter directions
//the maximum value for the remote control joystick is 660
//if the value is greater than 630
//then a maximum value is considered to be reached


#define RC_CMD_LONG_TIME 2000 //遥控器使能校准的时间，即保持内八的时间
//remote control calibration enable time
//aka the time for the remote control's joystick to lean towards inner direction

#define GYRO_CALIBRATE_TIME 20000 //陀螺仪校准的时间 20s
//the calibration time for the gyroscope
//20 seconds

//校准设备名
//device names for calibration
typedef enum
{
    CALI_HEAD,
    CALI_GIMBAL,
    CALI_GYRO,
    CALI_ACC,
    CALI_MAG,
    CALI_LIST_LENGHT,
    //add more...
} cali_id_e;

//校准设备前部，通过flash_buf链接到对应的校准设备变量地址
//the front part/header for the calibration devices
//use flash_buf to link the address for the corresponding devices for calibration
typedef __packed struct
{
    uint8_t name[3];
    uint8_t cali_done;
    uint8_t flash_len : 7;
    uint8_t cali_cmd : 1;
    uint32_t *flash_buf;
    bool_t (*cali_hook)(uint32_t *point, bool_t cmd);
} cali_sensor_t;

//头设备的校准数据
//head device's calibration data
typedef __packed struct
{
    uint8_t self_id;
    int8_t temperate;
    uint16_t firmware_version;
    fp32 latitude;
} head_cali_t;
//云台设备的校准数据
//tripod device calibration data
typedef struct
{
    uint16_t yaw_offset;
    uint16_t pitch_offset;
    fp32 yaw_max_angle;
    fp32 yaw_min_angle;
    fp32 pitch_max_angle;
    fp32 pitch_min_angle;
} gimbal_cali_t;
//陀螺仪，加速度计，磁力计通用校准数据
//calibration data for the gyroscope, accelerometer and magnetometer
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

//初始化，以及读取flash校准值
//initialization and read the calibration values from flash
extern void cali_param_init(void);
//返回mpu6500控制的温度
//return the temperature controlled by mpu6500
extern int8_t get_control_temperate(void);
//校准任务
//calibration tasks
extern void calibrate_task(void *pvParameters);
#endif
