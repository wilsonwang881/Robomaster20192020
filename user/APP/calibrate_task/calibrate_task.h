/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       Calibrate_task.c/h
  * @brief      ���У׼����豸�����ݣ�������̨�������ǣ����ٶȼƣ�������
  *             ��̨У׼��Ҫ����ֵ�������С��ԽǶȣ���������ҪУ׼��Ư
  *             ���ٶȼƺʹ�����ֻ��д�ýӿں��������ٶȼ�Ŀǰû�б�ҪУ׼��
  *             ��������δʹ���ڽ����㷨�С�
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
  *  V1.0.0     Dec-26-2018     RM              1. ���
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

#define imu_start_buzzer() buzzer_on(95, 10000) //IMUԪ��У׼�ķ�������Ƶ���Լ�ǿ��
//the frequency and the intensity of the buzzer for IMU component calibration

#define gimbal_start_buzzer() buzzer_on(31, 20000) //��̨У׼�ķ�������Ƶ���Լ�ǿ��
//the frequency and the intensity of the buzzer for tripod calibration

#define cali_buzzer_off() buzzer_off() //�رշ�����
//turn off the buzzer

#define cali_get_mcu_temperature() get_temprate() //��ȡstm32�ϵ��¶� ��ΪIMUУ׼�Ļ����¶�
//get the temperature of stm32
//use it as the environment temperature for IMU calibration

#define cali_flash_read(address, buf, len) flash_read((address), (buf), (len))                  //flash ��ȡ����
//reading function for flash

#define cali_flash_write(address, buf, len) flash_write_single_address((address), (buf), (len)) //flash д�뺯��
//writing function for flash

#define get_remote_ctrl_point_cali() get_remote_control_point() //��ȡң�����Ľṹ��ָ��
//get the pointer for the struct of the remote control

#define gyro_cali_disable_control() RC_unable()                 //������У׼ʱ�����ң����
//offline the remote control during gyroscope calibra

//������У׼����
//gyroscope calibration function
#define gyro_cali_fun(cali_scale, cali_offset, time_count) INS_cali_gyro((cali_scale), (cali_offset), (time_count))

//����������У׼ֵ
//set the calibration value for the gyroscope
#define gyro_set_cali(cali_scale, cali_offset) INS_set_cali_gyro((cali_scale), (cali_offset))

#define FLASH_USER_ADDR ADDR_FLASH_SECTOR_23 //Ҫд��flash������Ŀǰ�����һ�飬
//the sector to be written to flash,currently it is the last sector

#define GYRO_CONST_MAX_TEMP 45.0f //�����ǿ��ƺ��� �������¶�
//a constant temperature for gyroscope control
//the max control temperature

#define CALI_FUNC_CMD_ON 1   //У׼������ʹ��У׼
//calibration function enable

#define CALI_FUNC_CMD_INIT 0 //У׼����������flash�е�У׼����
//calibration function, passing calibration parameters in flash

#define CALIBRATE_CONTROL_TIME 1 //У׼���������е����� Ϊ1ms
//period for running the calibration function, set to 1ms

#define CALI_SENSOR_HEAD_LEGHT 1 //У׼�ṹ��ı�ͷ Ϊcali_sensor_t��ǰ�� �꿴 cali_sensor_t������ ��С 1 ����һ��32λ����
//calibration struct header
//it is the front part of cali_sensor_t
//refer to cali_sensor_t for more information
//size one represents 1 32-bit number

#define SELF_ID 0              //��ͷ�е�ID
//header id

#define FIRMWARE_VERSION 12345 //��ͷ�е�Ӳ���汾�� Ŀǰ����д��
//hardware version number in the header, currently it is a random number

#define CALIED_FLAG 0x55       //�������Ѿ�У׼���
//represents the calibration is complete

#define CALIBRATE_END_TIME 20000 //ң����У׼ʱ��20s ����20s��Ҫ���²���
//remote control calibration lasts for 20 seconds
//require restarting the operation if it takes longer than 20 seconds

#define RC_CALI_BUZZER_MIDDLE_TIME 10000 //У׼ʱ���ı������Ƶ�ʳ���̨�ĸ�Ƶ����������������20sУ׼ʱ���������
//during calibration, change the buzzer frequency to the high frequency sound of the tripod
//this helps with alarming when the 20 seconds for calibration is due soon

#define rc_cali_buzzer_middle_on() gimbal_start_buzzer()

#define RC_CALI_BUZZER_START_TIME 0 //У׼ʱ���ı������Ƶ�ʳ�IMU�ĵ�Ƶ����������У׼ʱ��20s��ʼ��ʱ
//during calibration, change the buzzer frequency to the low frequency of IMU
//this represents the 20 seconds timing for calibration starts

#define rc_cali_buzzer_start_on() imu_start_buzzer()

#define RCCALI_BUZZER_CYCLE_TIME 400  //У׼ѡ��ʱ��20s��������������������ʱ��
//calibration select time, 20 seconds
//the peroid for the buzzer to make a sound

#define RC_CALI_BUZZER_PAUSE_TIME 200 //У׼ѡ��ʱ��20s����������������ͣ��ʱ��
//calibration seleect time, 20 seconds
//the period for the buzzer to make a sound

#define RC_CALI_VALUE_HOLE 600        //ң������˻����ڰ� ��ֵ�ж��� ң����ҡ������� 660 ֻҪ����630 ����Ϊ�����
//the threshold value of determining whether the remote control joystick leans
//towards inner or outter directions
//the maximum value for the remote control joystick is 660
//if the value is greater than 630
//then a maximum value is considered to be reached


#define RC_CMD_LONG_TIME 2000 //ң����ʹ��У׼��ʱ�䣬�������ڰ˵�ʱ��
//remote control calibration enable time
//aka the time for the remote control's joystick to lean towards inner direction

#define GYRO_CALIBRATE_TIME 20000 //������У׼��ʱ�� 20s
//the calibration time for the gyroscope
//20 seconds

//У׼�豸��
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

//У׼�豸ǰ����ͨ��flash_buf���ӵ���Ӧ��У׼�豸������ַ
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

//ͷ�豸��У׼����
//head device's calibration data
typedef __packed struct
{
    uint8_t self_id;
    int8_t temperate;
    uint16_t firmware_version;
    fp32 latitude;
} head_cali_t;
//��̨�豸��У׼����
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
//�����ǣ����ٶȼƣ�������ͨ��У׼����
//calibration data for the gyroscope, accelerometer and magnetometer
typedef struct
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;

//��ʼ�����Լ���ȡflashУ׼ֵ
//initialization and read the calibration values from flash
extern void cali_param_init(void);
//����mpu6500���Ƶ��¶�
//return the temperature controlled by mpu6500
extern int8_t get_control_temperate(void);
//У׼����
//calibration tasks
extern void calibrate_task(void *pvParameters);
#endif
