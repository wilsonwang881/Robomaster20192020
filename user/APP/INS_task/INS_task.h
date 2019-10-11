/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       INSTask.c/h
  * @brief      主要利用陀螺仪mpu6500，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过mpu6500的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间，提供注释对应的宏定义，关闭DMA，
  *             DR的外部中断的方式.
  * 
  *             Seems to use the MPU6500(gyro+accelerometer) and IST8310(magnetometer)
  *             to calculate 3d position of turret, and the euler angle. Through the SPI
  *             of DMA transmission save CPU time, provide the annotation corresponding 
  *             macro definition, close the DMA
  * 
  * @note       SPI 在陀螺仪初始化的时候需要低于2MHz，之后读取数据需低于20MHz
  *             SPI needs to be lower than 2MHz during gyroscope initialization
  *             After that SPI needs to be lower than 20 MHz when reading data
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成    complete
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef INS_Task_H
#define INS_Task_H
#include "main.h"

#define USE_IST8310 //是否使用IST8310磁力计，不使用注释定义
//whether to use IST8310 magnenometer
//if not used, comment out the definition

#define MPU6500_USE_DATA_READY_EXIT //是否使用MPU6500的外部中断，不使用注释定义
//whether to use MPU6500's external interrupt
//if not used, comment out the definition

#define MPU6500_USE_SPI_DMA //是否使用SPI的DMA传输，不使用注释定义
//whether to use SPI's DMA transmission
//if not used, comment out the definition

//如果用了IST8310，DMA传输23个字节，如果不用，少7个字节，为16个字节
//if IST8310 is used, DMA transmits 23 bytes
//if not used, DMA transmits 7 bytes less, which is 16 bytes
#if defined(USE_IST8310)
#define DMA_RX_NUM 23
#else
#define DMA_RX_NUM 16
#endif

//mpu6500原始数据在缓冲区buf的位置
//the location of mpu6500's raw data in the buffer buf
#ifdef MPU6500_USE_SPI_DMA
#define MPU6500_RX_BUF_DATA_OFFSET 1
#else
#define MPU6500_RX_BUF_DATA_OFFSET 0
#endif

//ist8310原始数据在缓冲区buf的位置
//the location of ist8310's raw data in the buffer buf
#ifdef MPU6500_USE_SPI_DMA
#define IST8310_RX_BUF_DATA_OFFSET 16
#else
#define IST8310_RX_BUF_DATA_OFFSET 15
#endif

#define MPU6500_TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp    temperature control's PID's kp
#define MPU6500_TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki    temperature control's PID's ki
#define MPU6500_TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd    temperature control's PID's kd

#define MPU6500_TEMPERATURE_PID_MAX_OUT 4500.0f  //温度控制PID的max_out    max_out of temperature control's PID
#define MPU6500_TEMPERATURE_PID_MAX_IOUT 4400.0f //温度控制PID的max_iout    max_iout of temperature control's PID

#define INS_DELTA_TICK 1 //任务调用的间隔    interval of calling tasks

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间    during the initialization phase of tasks, delay some time

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1
//the reloading value of setting TIM of mpu6500 temperature control
//that is, the maximum value of PWM is MPU6500_TEMP_PWM_MAX - 1

//获取姿态角指针地址后，对应姿态角的地址偏移量 fp32类型
//the adress shift of the corresponding position angle after acquiring the pointer of the position angle
//type fp32
#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

extern void INSTask(void *pvParameters);

extern void INS_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3], uint16_t *time_count);
extern void INS_set_cali_gyro(fp32 cali_scale[3], fp32 cali_offset[3]);
extern const fp32 *get_INS_angle_point(void);
extern const fp32 *get_MPU6500_Gyro_Data_Point(void);
extern const fp32 *get_MPU6500_Accel_Data_Point(void);

#endif
