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

//����У׼�豸�����֣�У׼��ʶ����У׼����flash��С��У׼�����Ӧ��У׼���ݵ�ַ
//contains the names, standard identification, calibration flash size,
//calibration commands and the corresponding data addresses
//for the calibration devices
#include "calibrate_Task.h"

#include "adc.h"
#include "buzzer.h"
#include "flash.h"
#include "Remote_Control.h"
#include "INS_Task.h"
#include "string.h"
#include "gimbal_task.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t calibrate_task_stack;
#endif

static void cali_data_read(void);                           //��ȡ����У׼����
//read all calibration data

static void cali_data_write(void);                          //д�뵱ǰУ׼����������
//write the data of the current calibration parameters

static bool_t cali_head_hook(uint32_t *cali, bool_t cmd);   //��ͷУ׼���ݺ���
//header calibration data function

static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd);   //������У׼���ݺ���
//gyroscope calibration data function

static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd); //��̨У׼���ݺ���
//tripod calibration data function

static const RC_ctrl_t *calibrate_RC; //ң�����ṹ��ָ��
//struct pointer for the remote control

static head_cali_t head_cali;         //��ͷУ׼����
//header calibration data

static gimbal_cali_t gimbal_cali;     //��̨У׼����
//tripod calibration data

static imu_cali_t gyro_cali;          //������У׼����
//gyroscope calibration data

static imu_cali_t accel_cali;         //���ٶ�У׼����
//accelerometer calibration data

static imu_cali_t mag_cali;           //������У׼����
//magnetometer calibration data

static cali_sensor_t cali_sensor[CALI_LIST_LENGHT]; //У׼�豸���飬
//calibration device number array

static const uint8_t cali_name[CALI_LIST_LENGHT][3] = {"HD", "GM", "GYR", "ACC", "MAG"}; //У׼�豸������
//names of the calibration devices

//У׼�豸��Ӧ����ṹ�������ַ
//calibration devices correspond to the variable addresses put in the struct
static uint32_t *cali_sensor_buf[CALI_LIST_LENGHT] =
    {
        (uint32_t *)&head_cali, (uint32_t *)&gimbal_cali,
        (uint32_t *)&gyro_cali, (uint32_t *)&accel_cali,
        (uint32_t *)&mag_cali};

//У׼�豸��Ӧ�������ݴ�С
//data size corresponds to the calibration devices
static uint8_t cali_sensor_size[CALI_LIST_LENGHT] =
    {
        sizeof(head_cali_t) / 4, sizeof(gimbal_cali_t) / 4,
        sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4, sizeof(imu_cali_t) / 4};

//У׼�豸��Ӧ��У׼����
//calibration fcuntions correspond to the calibration devices
void *cali_hook_fun[CALI_LIST_LENGHT] = {cali_head_hook, cali_gimbal_hook, cali_gyro_hook, NULL, NULL};

//У׼��Ӧʱ���������freeRTOS��tick��ɡ�
//corresponding calibration timestamps
//complete use freeRTOS's tick
static uint32_t calibrate_systemTick;

//ң��������У׼�豸��У׼
//calibration for the remote-control-controlled calibration devices
static void RC_cmd_to_calibrate(void);

void calibrate_task(void *pvParameters)
{
    static uint8_t i = 0;
    calibrate_RC = get_remote_ctrl_point_cali();

    while (1)
    {

        //ң��������У׼����
        //steps for caliberation using the remote control
        RC_cmd_to_calibrate();

        for (i = 0; i < CALI_LIST_LENGHT; i++)
        {
            //У׼����Ϊ1 ��ʾ��ҪУ׼
            //caliberation command is 1
            //this means caliberation is needed
            if (cali_sensor[i].cali_cmd)
            {
                if (cali_sensor[i].cali_hook != NULL)
                {
                    //����У׼����
                    //call the caliberation function
                    if (cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_ON))
                    {
                        //У׼���
                        //caliberation complete
                        cali_sensor[i].name[0] = cali_name[i][0];
                        cali_sensor[i].name[1] = cali_name[i][1];
                        cali_sensor[i].name[2] = cali_name[i][2];

                        cali_sensor[i].cali_done = CALIED_FLAG;

                        cali_sensor[i].cali_cmd = 0;
                        //д��flash
                        //write to flash
                        cali_data_write();
                    }
                }
            }
        }
        vTaskDelay(CALIBRATE_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        calibrate_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
//����mpu6500��Ҫ���Ƶ����¶�
//return the control temperature for mpu6500
int8_t get_control_temperate(void)
{
    return head_cali.temperate;
}

//ң��������У׼��̨��������
//remote control operates the tripod and the gyroscope
static void RC_cmd_to_calibrate(void)
{
    static uint8_t i;
    static uint32_t rc_cmd_systemTick = 0;
    static uint16_t buzzer_time = 0;
    static uint16_t rc_cmd_time = 0;
    static uint8_t rc_action_falg = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_cmd)
        {
            buzzer_time = 0;
            rc_cmd_time = 0;
            rc_action_falg = 0;
            return;
        }
    }

    if (rc_action_falg == 0 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //�ж�ң����2s�ڰ˿�ʼ20sУ׼ѡ��ʱ�䣬rc_action_falg��rc_cmd_time���·��߼��ж�
        //determine whether the remote control 2s leans towards inner direction
        //start the 20 second caliberation selection time
        //rc_action_falg and rc_cmd_time below for determining the logic
        rc_cmd_systemTick = xTaskGetTickCount();
        rc_action_falg = 1;
        rc_cmd_time = 0;
    }
    else if (rc_action_falg == 2 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //�ж�ң������20sУ׼ѡ��ʱ�䣬�����ʹ����̨У׼�����ұ���2s,rc_action_falg��rc_cmd_time���·��߼��ж�
        //determine the remote control within the 20 second caliberation select time
        //leaning towards outter direction can caliberate the tripod, and maintain 2s
        //rc_action_falg and rc_cmd_time below for determining the logic
        rc_action_falg = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GIMBAL].cali_cmd = 1;
    }
    else if (rc_action_falg == 3 && rc_cmd_time > RC_CMD_LONG_TIME)
    {
        //�ж�ң������20sУ׼ѡ��ʱ�䣬�����ʹ��������У׼�����ұ���2s��rc_action_falg��rc_cmd_time���·��߼��ж�
        //determine the remote control within the 20 seconds caliberation selection time
        //leaning towards the outter direction can caliberate the gyroscope, and maintain 2s
        //rc_action_falg and rc_cmd_time below for determining the logic
        rc_action_falg = 0;
        rc_cmd_time = 0;
        cali_sensor[CALI_GYRO].cali_cmd = 1;
        //����MPU6500��Ҫ���Ƶ��¶�
        //update the the temperature required by MPU6500 for controlling
        head_cali.temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
        if (head_cali.temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
        {
            head_cali.temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
        }
    }

    if (calibrate_RC->rc.ch[0] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg == 0)
    {
        //�ж�ң����2s�ڰ� ��ʱ��ʱ�䣬 ��rc_cmd_time > 2000 Ϊ����2s
        //determine the time that the remote control 2s leans towards the inner direction
        //if rc_cmd_time > 2000 -> maintain 2s
        rc_cmd_time++;
    }
    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] > RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg != 0)
    {
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ��̨ʹ��
        //determine the time that the remote control 2s leans towards the outter direction
        //tripod enable
        rc_cmd_time++;
        rc_action_falg = 2;
    }

    else if (calibrate_RC->rc.ch[0] > RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[1] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[2] < -RC_CALI_VALUE_HOLE && calibrate_RC->rc.ch[3] < -RC_CALI_VALUE_HOLE && switch_is_down(calibrate_RC->rc.s[0]) && switch_is_down(calibrate_RC->rc.s[1]) && rc_action_falg != 0)
    {
        //�ж�ң����2s����� ��ʱ��ʱ�䣬 ������ʹ��
        //determine the time that the remote control 2s leans towards the outter direction
        //gyroscope enable
        rc_cmd_time++;
        rc_action_falg = 3;
    }
    else
    {
        rc_cmd_time = 0;
    }

    calibrate_systemTick = xTaskGetTickCount();

    if (calibrate_systemTick - rc_cmd_systemTick > CALIBRATE_END_TIME)
    {
        //�ж�ң����20sУ׼ѡ��ʱ�䣬�޲���
        //determine the remote control 20s' caliberation selection time
        //no operation
        rc_action_falg = 0;
        return;
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > RC_CALI_BUZZER_MIDDLE_TIME && rc_cmd_systemTick != 0 && rc_action_falg != 0)
    {

        //�ж�ң����10s��У׼ѡ��ʱ���л���������Ƶ����
        //determine the high frequency sound to be switched to on the buzzer
        //after the remote control's 10s caliberation selection time
        rc_cali_buzzer_middle_on();
    }
    else if (calibrate_systemTick - rc_cmd_systemTick > 0 && rc_cmd_systemTick != 0 && rc_action_falg != 0)
    {

        //ң����10sǰ ��ʼ��������Ƶ����
        //in the previous 10s of the remote control
        //start playing the low frequency sound on the buzzer
        rc_cali_buzzer_start_on();
    }

    if (rc_action_falg != 0)
    {
        buzzer_time++;
    }
    //��������������
    //buzzer makes sound discontinuously
    if (buzzer_time > RCCALI_BUZZER_CYCLE_TIME && rc_action_falg != 0)
    {
        buzzer_time = 0;
    }
    if (buzzer_time > RC_CALI_BUZZER_PAUSE_TIME && rc_action_falg != 0)
    {
        cali_buzzer_off();
    }
}

//��ʼ��У׼�ṹ�����飬��ȡflashֵ�����δУ׼��ʹ��У׼����,ͬʱ��ʼ����ӦУ׼����
//initialize the caliberation struct array
//read flash values
//if not caliberated, enable the caliberation command
//at the same time, initialize the corresponding caliberation data
void cali_param_init(void)
{
    uint8_t i = 0;

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        cali_sensor[i].flash_len = cali_sensor_size[i];
        cali_sensor[i].flash_buf = cali_sensor_buf[i];
        cali_sensor[i].cali_hook = (bool_t(*)(uint32_t *, bool_t))cali_hook_fun[i];
    }

    cali_data_read();

    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        if (cali_sensor[i].cali_done == CALIED_FLAG)
        {
            if (cali_sensor[i].cali_hook != NULL)
            {
                //���У׼��ϣ���У׼ֵ���ݵ���Ӧ��У׼����
                //if caliberation finished
                //pass the caliberation values to the corresponding caliberation parameters
                cali_sensor[i].cali_hook(cali_sensor_buf[i], CALI_FUNC_CMD_INIT);
            }
        }
    }
}

void cali_data_read(void)
{
    uint8_t flash_read_buf[CALI_SENSOR_HEAD_LEGHT * 4];
    uint8_t i = 0;
    uint16_t offset = 0;
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //��ȡУ׼�豸��ǰ������
        //read the front data of the caliberation devices
        cali_flash_read(FLASH_USER_ADDR + offset, (uint32_t *)flash_read_buf, CALI_SENSOR_HEAD_LEGHT);
        //�����֣�У׼��ʶ�����и�ֵ
        //assign values to names and caliberation identification
        cali_sensor[i].name[0] = flash_read_buf[0];
        cali_sensor[i].name[1] = flash_read_buf[1];
        cali_sensor[i].name[2] = flash_read_buf[2];
        cali_sensor[i].cali_done = flash_read_buf[3];

        //flashλ��ƫ��
        //flash position shift
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //��flash��������ݣ����ݵ��豸��Ӧ�ı�����ַ��
        //pass the data saved in flash to the devices' corresponding varibale addresses
        cali_flash_read(FLASH_USER_ADDR + offset, cali_sensor[i].flash_buf, cali_sensor[i].flash_len);
        //ƫ��У׼�����ֽڴ�С
        //size of the shift caliberation data byte
        offset += cali_sensor[i].flash_len * 4;
        //����豸δУ׼��ʹ��У׼
        //if the device is not caliberated, enable caliberation
        if (cali_sensor[i].cali_done != CALIED_FLAG && cali_sensor[i].cali_hook != NULL)
        {
            cali_sensor[i].cali_cmd = 1;
        }
    }
}

static void cali_data_write(void)
{
    uint8_t i = 0;
    uint16_t offset = 0;
    const uint16_t len = (sizeof(head_cali_t) + sizeof(gimbal_cali_t) + sizeof(imu_cali_t) * 3) / 4 + 5;
    uint8_t buf[len * 4];
    for (i = 0; i < CALI_LIST_LENGHT; i++)
    {
        //�����豸ǰ���������������֣� ���ݴ�С
        //copy the front part parameters of the device
        //i.e. name, size of data
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].name, CALI_SENSOR_HEAD_LEGHT * 4);
        offset += CALI_SENSOR_HEAD_LEGHT * 4;
        //�����豸У׼����
        //copy caliberation data of the device
        memcpy((void *)(buf + offset), (void *)cali_sensor[i].flash_buf, cali_sensor[i].flash_len * 4);

        offset += cali_sensor[i].flash_len * 4;
    }

    //д��flash
    //write to flash
    cali_flash_write(FLASH_USER_ADDR, (uint32_t *)buf, len);
}

//ͷ�豸У׼��������Ҫ����γ�ȣ�MPU6500���Ƶ��¶ȣ�Ӳ���汾��
//head device caliberation function
//mainly for saving the latitude, temperature controlled by MPU6500, hardware version number
static bool_t cali_head_hook(uint32_t *cali, bool_t cmd)
{
    if (cmd == 0)
        return 1;
    head_cali.self_id = SELF_ID;
    head_cali.temperate = (int8_t)(cali_get_mcu_temperature()) + 10;
    if (head_cali.temperate > (int8_t)(GYRO_CONST_MAX_TEMP))
    {
        head_cali.temperate = (int8_t)(GYRO_CONST_MAX_TEMP);
    }
    head_cali.firmware_version = FIRMWARE_VERSION;
    head_cali.latitude = Latitude_At_ShenZhen;
    return 1;
}

//У׼�������豸����ҪУ׼��Ư
//caliberate the gyroscope
//mainly for caliberating the zero shift
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
    imu_cali_t *local_cali_t = (imu_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        gyro_set_cali(local_cali_t->scale, local_cali_t->offset);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        static uint16_t count_time = 0;
        gyro_cali_fun(local_cali_t->scale, local_cali_t->offset, &count_time);
        if (count_time > GYRO_CALIBRATE_TIME)
        {
            count_time = 0;
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gyro_cali_disable_control(); //����ң�����Է������
            //offline the remote control to avoid accidental operation

            imu_start_buzzer();
            return 0;
        }
    }
    return 0;
}

static bool_t cali_gimbal_hook(uint32_t *cali, bool_t cmd)
{

    gimbal_cali_t *local_cali_t = (gimbal_cali_t *)cali;
    if (cmd == CALI_FUNC_CMD_INIT)
    {
        set_cali_gimbal_hook(local_cali_t->yaw_offset, local_cali_t->pitch_offset,
                             local_cali_t->yaw_max_angle, local_cali_t->yaw_min_angle,
                             local_cali_t->pitch_max_angle, local_cali_t->pitch_min_angle);
        return 0;
    }
    else if (cmd == CALI_FUNC_CMD_ON)
    {
        if (cmd_cali_gimbal_hook(&local_cali_t->yaw_offset, &local_cali_t->pitch_offset,
                                 &local_cali_t->yaw_max_angle, &local_cali_t->yaw_min_angle,
                                 &local_cali_t->pitch_max_angle, &local_cali_t->pitch_min_angle))
        {
            cali_buzzer_off();
            return 1;
        }
        else
        {
            gimbal_start_buzzer();
            return 0;
        }
    }
    return 0;
}

//����γ����Ϣ
//return the latitude information 
void getFlashLatitude(float *latitude)
{

    if (latitude == NULL)
    {
        return;
    }
    if (cali_sensor[CALI_HEAD].cali_done == CALIED_FLAG)
    {
        *latitude = head_cali.latitude;
    }
    else
    {
        *latitude = Latitude_At_ShenZhen;
    }
}
