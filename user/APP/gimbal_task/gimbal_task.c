/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ����������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  *             Complete tripod control tasks. Due to the fact that the tripod 
  *             uses angle computed from the gyroscope, its range is (-pi, pi).
  *             Hence the set target angles are all ranges. Many functions exist
  *             for computing relative angle. Tripod has two modes. Gyroscope control
  *             state is to use the position angle computed using the onboard 
  *             gyroscope to control. The coder control mode is to use the feedback
  *             value on the coding value given by the motor. Other modes such as
  *             calibration mode and stopping mode exist.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ��� complete
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Gimbal_Task.h"

#include "main.h"

#include "arm_math.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "INS_Task.h"
#include "remote_control.h"
#include "shoot.h"
#include "CAN_Receive.h"
#include "Detect_Task.h"
#include "pid.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//�������ֵ���� 0��8191
//motor coding value 0-8191
#define ECD_Format(ecd)         \
    {                           \
        if ((ecd) > ecd_range)  \
            (ecd) -= ecd_range; \
        else if ((ecd) < 0)     \
            (ecd) += ecd_range; \
    }

#define gimbal_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                          \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);   \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);   \
        PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                    \
                                                                                               \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid); \
        Gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid); \
        PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                  \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

//��̨���������������
//all relavent data for tripod control
static Gimbal_Control_t gimbal_control;

//���͵�can ָ��
//send can command
static int16_t Yaw_Can_Set_Current = 0, Pitch_Can_Set_Current = 0, Shoot_Can_Set_Current = 0;

//��̨��ʼ��
//tripod initialization
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init);
//��̨pid����
//tripod pid rest
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);
//��̨״̬����
//tripod state set
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//��̨���ݸ���
//tripod data update
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//��̨״̬�л��������ݣ������������״̬�л���������״̬����Ŀ��ֵ
//save data when tripod state switches
//for instance, save the target value when the gyroscope state switches to coder state
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//������̨��������ֵ����ԽǶ�
//calculate the relative angle of the relative middle value of the tripod motor
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//������̨������
//set the control value for the tripod
static void GIMBAL_Set_Control(Gimbal_Control_t *gimbal_set_control);
//��̨����pid����
//tripod pid calculation
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);

static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor);
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor);

//�������ǽǶȿ����£��Կ��Ƶ�Ŀ��ֵ�������Է��������ԽǶ�
//restrict the controlled target value to avoid exceeding the maximum relative angle
//under the control of the gyroscope
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add);
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 intergral_limit, fp32 kp, fp32 ki, fp32 kd);
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);

static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch);
#if GIMBAL_TEST_MODE
//j-scope ����pid����
//j-scipe help tune the parameters of the pid
static void J_scope_gimbal_test(void);
#endif

void GIMBAL_task(void *pvParameters)
{
    //�ȴ������������������������
    //wait for the gyroscope task to update gyroscope data
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //��̨��ʼ��
    //tripod initialization
    GIMBAL_Init(&gimbal_control);
    //�����ʼ��
    //shoot initializating
    shoot_init();
    //�жϵ���Ƿ�����
    //check whether all motors are online
    while (toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE) || toe_is_error(TriggerMotorTOE) )
    {
        vTaskDelay(GIMBAL_CONTROL_TIME);
        GIMBAL_Feedback_Update(&gimbal_control);             //��̨���ݷ���    tripod data feedback
    }

    while (1)
    {
        GIMBAL_Set_Mode(&gimbal_control);                    //������̨����ģʽ    set the control mode of the tripod
        GIMBAL_Mode_Change_Control_Transit(&gimbal_control); //����ģʽ�л� �������ݹ���    control mode switch, control the data transition
        GIMBAL_Feedback_Update(&gimbal_control);             //��̨���ݷ���    tripod data feedback
        GIMBAL_Set_Control(&gimbal_control);                 //������̨������    set the control value for the tripod
        GIMBAL_Control_loop(&gimbal_control);                //��̨����PID����    tripod control PID calculation
        Shoot_Can_Set_Current = shoot_control_loop();        //����������ѭ��    loop the shooting task
#if YAW_TURN
        Yaw_Can_Set_Current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        Yaw_Can_Set_Current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        Pitch_Can_Set_Current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        Pitch_Can_Set_Current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

        //��̨��ң��������״̬��relax ״̬��canָ��Ϊ0����ʹ��current����Ϊ��ķ������Ǳ�֤ң��������һ��ʹ����ֹ̨ͣ
        //can command is zero when the tripod is in relax state(remote control offline)
        //do not use the method of setting to zero via current
        //to ensure the tripod stops when the remote control is offline
        if (!(toe_is_error(YawGimbalMotorTOE) && toe_is_error(PitchGimbalMotorTOE) && toe_is_error(TriggerMotorTOE)))
        {
            if (toe_is_error(DBUSTOE))
            {
                CAN_CMD_GIMBAL(0, 0, 0, 0);
            }
            else
            {
                CAN_CMD_GIMBAL(Yaw_Can_Set_Current, Pitch_Can_Set_Current, Shoot_Can_Set_Current, 0);
            }
        }

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          ��̨У׼���ã���У׼����̨��ֵ�Լ���С����е��ԽǶ�
  *                 Tripod calibration set
  *                 will caliberate the tripod middle value and the maximum and the minumum value
  *                 of the mechanical relative angle
  * @author         RM
  * @param[in]      yaw ��ֵ    yaw middle value
  * @param[in]      pitch ��ֵ    pitch middle value
  * @param[in]      yaw �����ԽǶ�    yaw maximum relative angle
  * @param[in]      yaw ��С��ԽǶ�    yaw minimum relative angle
  * @param[in]      pitch �����ԽǶ�    pitch maximum relative angle
  * @param[in]      pitch ��С��ԽǶ�    pitch minimum relative angle
  * @retval         ���ؿ�    return null
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  *                 The function uses gimbal_control
  *                 static variable makes the function not suitable for the above generic pointer reusing
  */
void set_cali_gimbal_hook(const uint16_t yaw_offset, const uint16_t pitch_offset, const fp32 max_yaw, const fp32 min_yaw, const fp32 max_pitch, const fp32 min_pitch)
{
    gimbal_control.gimbal_yaw_motor.offset_ecd = yaw_offset;
    gimbal_control.gimbal_yaw_motor.max_relative_angle = max_yaw;
    gimbal_control.gimbal_yaw_motor.min_relative_angle = min_yaw;

    gimbal_control.gimbal_pitch_motor.offset_ecd = pitch_offset;
    gimbal_control.gimbal_pitch_motor.max_relative_angle = max_pitch;
    gimbal_control.gimbal_pitch_motor.min_relative_angle = min_pitch;
}


/**
  * @brief          ��̨У׼���㣬��У׼��¼����� ��Сֵ ��������̨ ��ֵ�������С��е�Ƕ�
  *                 Tripod calibration calculation
  *                 will caliberate the recorded maximum, minimum values
  *                 for calculating the middle, the maximum and the minimum values of the
  *                 the mechanical angles of the tripod
  * @author         RM
  * @param[in]      yaw ��ֵ ָ��    yaw middle value pointer
  * @param[in]      pitch ��ֵ ָ��    pitch middle value pointer
  * @param[in]      yaw �����ԽǶ� ָ��    yaw maximum relative angle pointer
  * @param[in]      yaw ��С��ԽǶ� ָ��    yaw minimum relative angle pointer
  * @param[in]      pitch �����ԽǶ� ָ��    pitch maximum relative angle pointer
  * @param[in]      pitch ��С��ԽǶ� ָ��    pitch minimum relative angle pointer
  * @retval         ����1 �����ɹ�У׼��ϣ� ����0 ����δУ׼��    
  *                 return 1: calibration finishes successfully
  *                 return 0: calibration not finished
  * @waring         �������ʹ�õ�gimbal_control ��̬�������º�������������ͨ��ָ�븴��
  *                 The function uses gimbal_control
  *                 Static variable makes the function not suitable for the reuse of the above generic pointers
  */
bool_t cmd_cali_gimbal_hook(uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_control.gimbal_cali.step == 0)
    {
        gimbal_control.gimbal_cali.step = GIMBAL_CALI_START_STEP;
        //�������ʱ������ݣ���Ϊ��ʼ���ݣ����ж������Сֵ
        //save the data when entering
        //as the initial data
        //to determine the maximum and the minimum values
        gimbal_control.gimbal_cali.max_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.max_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.max_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_pitch = gimbal_control.gimbal_pitch_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_pitch_ecd = gimbal_control.gimbal_pitch_motor.gimbal_motor_measure->ecd;
        gimbal_control.gimbal_cali.min_yaw = gimbal_control.gimbal_yaw_motor.absolute_angle;
        gimbal_control.gimbal_cali.min_yaw_ecd = gimbal_control.gimbal_yaw_motor.gimbal_motor_measure->ecd;
        return 0;
    }
    else if (gimbal_control.gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        calc_gimbal_cali(&gimbal_control.gimbal_cali, yaw_offset, pitch_offset, max_yaw, min_yaw, max_pitch, min_pitch);
        gimbal_control.gimbal_yaw_motor.offset_ecd = *yaw_offset;
        gimbal_control.gimbal_yaw_motor.max_relative_angle = *max_yaw;
        gimbal_control.gimbal_yaw_motor.min_relative_angle = *min_yaw;
        gimbal_control.gimbal_pitch_motor.offset_ecd = *pitch_offset;
        gimbal_control.gimbal_pitch_motor.max_relative_angle = *max_pitch;
        gimbal_control.gimbal_pitch_motor.min_relative_angle = *min_pitch;

        return 1;
    }
    else
    {
        return 0;
    }
}

//У׼���㣬������Ƕȣ���̨��ֵ
//calibration calculation
//relative maximum angle
//middle value of the tripod
static void calc_gimbal_cali(const Gimbal_Cali_t *gimbal_cali, uint16_t *yaw_offset, uint16_t *pitch_offset, fp32 *max_yaw, fp32 *min_yaw, fp32 *max_pitch, fp32 *min_pitch)
{
    if (gimbal_cali == NULL || yaw_offset == NULL || pitch_offset == NULL || max_yaw == NULL || min_yaw == NULL || max_pitch == NULL || min_pitch == NULL)
    {
        return;
    }

    int16_t temp_max_ecd = 0, temp_min_ecd = 0, temp_ecd = 0;

#if YAW_TURN
    temp_ecd = gimbal_cali->min_yaw_ecd - gimbal_cali->max_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd + (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = -motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = -motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#else

    temp_ecd = gimbal_cali->max_yaw_ecd - gimbal_cali->min_yaw_ecd;

    if (temp_ecd < 0)
    {
        temp_ecd += ecd_range;
    }
    temp_ecd = gimbal_cali->max_yaw_ecd - (temp_ecd / 2);

    ECD_Format(temp_ecd);
    *yaw_offset = temp_ecd;
    *max_yaw = motor_ecd_to_angle_change(gimbal_cali->max_yaw_ecd, *yaw_offset);
    *min_yaw = motor_ecd_to_angle_change(gimbal_cali->min_yaw_ecd, *yaw_offset);

#endif

#if PITCH_TURN

    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
    temp_max_ecd = gimbal_cali->max_pitch_ecd + temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
    temp_min_ecd = gimbal_cali->min_pitch_ecd + temp_ecd;

    ECD_Format(temp_max_ecd);
    ECD_Format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > Half_ecd_range)
    {
        temp_ecd -= ecd_range;
    }
    else if (temp_ecd < -Half_ecd_range)
    {
        temp_ecd += ecd_range;
    }

    if (temp_max_ecd > temp_min_ecd)
    {
        temp_min_ecd += ecd_range;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ECD_Format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = -motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = -motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);

#else
    temp_ecd = (int16_t)(gimbal_cali->max_pitch / Motor_Ecd_to_Rad);
    temp_max_ecd = gimbal_cali->max_pitch_ecd - temp_ecd;
    temp_ecd = (int16_t)(gimbal_cali->min_pitch / Motor_Ecd_to_Rad);
    temp_min_ecd = gimbal_cali->min_pitch_ecd - temp_ecd;

    ECD_Format(temp_max_ecd);
    ECD_Format(temp_min_ecd);

    temp_ecd = temp_max_ecd - temp_min_ecd;

    if (temp_ecd > Half_ecd_range)
    {
        temp_ecd -= ecd_range;
    }
    else if (temp_ecd < -Half_ecd_range)
    {
        temp_ecd += ecd_range;
    }

    temp_ecd = temp_max_ecd - temp_ecd / 2;

    ECD_Format(temp_ecd);

    *pitch_offset = temp_ecd;

    *max_pitch = motor_ecd_to_angle_change(gimbal_cali->max_pitch_ecd, *pitch_offset);
    *min_pitch = motor_ecd_to_angle_change(gimbal_cali->min_pitch_ecd, *pitch_offset);
#endif
}

const Gimbal_Motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

const Gimbal_Motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

//��ʼ��pid ����ָ��
//initialize pid data pointer
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{

    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
    //�������ָ���ȡ
    //acquire the motor data pointer
    gimbal_init->gimbal_yaw_motor.gimbal_motor_measure = get_Yaw_Gimbal_Motor_Measure_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_measure = get_Pitch_Gimbal_Motor_Measure_Point();
    //����������ָ���ȡ
    //acquire the gyroscope data pointer
    gimbal_init->gimbal_INT_angle_point = get_INS_angle_point();
    gimbal_init->gimbal_INT_gyro_point = get_MPU6500_Gyro_Data_Point();
    //ң��������ָ���ȡ
    //acquire the remote control data pointer
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();
    //��ʼ�����ģʽ
    //motor initializing mode
    gimbal_init->gimbal_yaw_motor.gimbal_motor_mode = gimbal_init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    gimbal_init->gimbal_pitch_motor.gimbal_motor_mode = gimbal_init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    //��ʼ��yaw���pid
    //initialize yaw motor pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    //��ʼ��pitch���pid
    //initialize pitch motor pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    //�������PID
    //clear all PID
    gimbal_total_pid_clear(gimbal_init);

    GIMBAL_Feedback_Update(gimbal_init);

    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = gimbal_init->gimbal_yaw_motor.relative_angle;
    gimbal_init->gimbal_yaw_motor.motor_gyro_set = gimbal_init->gimbal_yaw_motor.motor_gyro;


    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = gimbal_init->gimbal_pitch_motor.relative_angle;
    gimbal_init->gimbal_pitch_motor.motor_gyro_set = gimbal_init->gimbal_pitch_motor.motor_gyro;


}

static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}

static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }
    //��̨���ݸ���
    //tripod data update
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          gimbal_feedback_update->gimbal_pitch_motor.offset_ecd);
    gimbal_feedback_update->gimbal_pitch_motor.motor_gyro = *(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        gimbal_feedback_update->gimbal_yaw_motor.offset_ecd);

    gimbal_feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(gimbal_feedback_update->gimbal_pitch_motor.relative_angle) * (*(gimbal_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}
//������ԽǶ�
//calculate relative angle
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

//��̨״̬�л����棬����״̬�л�����
//save the tripod state switch, for state switch transition
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw���״̬���л���������
    //save the data of yaw motor state switch
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch���״̬���л���������
    //save the data of pitch motor state switch
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

//��̨����������
//set the control value of the tripod
static void GIMBAL_Set_Control(Gimbal_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);
    //yaw���ģʽ����
    //yaw motor mode control
    if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        //send the control value directly under raw mode
        gimbal_set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        //gyroscope angle control under gyro mode
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (gimbal_set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        //motor coding angle control under enconde mode
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch���ģʽ����
    //pitch motor mode control
    if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //rawģʽ�£�ֱ�ӷ��Ϳ���ֵ
        //send the control value directly under raw mode
        gimbal_set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyroģʽ�£������ǽǶȿ���
        //gyroscope angle control under gyro mode
        GIMBAL_absolute_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (gimbal_set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //encondeģʽ�£��������Ƕȿ���
        //motor coding angle control under enconde mode
        GIMBAL_relative_angle_limit(&gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}
//������ ����������
//gyroscope control value restriction
static void GIMBAL_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    static fp32 bias_angle;
    static fp32 angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }
    //��ǰ�������Ƕ�
    //current control error angle
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
    //��̨��ԽǶ�+ ���Ƕ� + �����Ƕ� ������� ����е�Ƕ�
    //tripod relative angle + error angle + newly added angle
    //if greater than the maximum mechanical angle
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //�����������е�Ƕȿ��Ʒ���
        //if towards the maximum mechanical angle control direction
        if (add > 0.0f)
        {
            //�����һ���������ӽǶȣ�
            //calculate a maximum added angle
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        if (add < 0.0f)
        {
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;
    //�Ƿ񳬹���� ��Сֵ
    //wheter exceed the maximum or the minimum values
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}
//��̨����״̬ʹ�ò�ͬ����pid
//tripod control state uses different control pid
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop)
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
    //yaw��ͬģʽ���ڲ�ͬ�Ŀ��ƺ���
    //different modes for yaw correspond to different control function
    if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw����
        //raw control
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro�Ƕȿ���
        //gyro angle control
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }
    else if (gimbal_control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde�Ƕȿ���
        //enconde angle control
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_yaw_motor);
    }

    //pitch��ͬģʽ���ڲ�ͬ�Ŀ��ƺ���
    //different modes for pitch correspond to different control functions
    if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //raw����
        //raw control
        gimbal_motor_raw_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //gyro�Ƕȿ���
        //gyro angle control
        gimbal_motor_absolute_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
    else if (gimbal_control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //enconde�Ƕȿ���
        //enconde angle control
        gimbal_motor_relative_angle_control(&gimbal_control_loop->gimbal_pitch_motor);
    }
}

static void gimbal_motor_absolute_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    //�ǶȻ����ٶȻ�����pid����
    //angle loop
    //speed loop serial level pid testing
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    //assign control value
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

static void gimbal_motor_relative_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //�ǶȻ����ٶȻ�����pid����
    //angle look
    //speed look serial level pid testing
    gimbal_motor->motor_gyro_set = GIMBAL_PID_Calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_Calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);
    //����ֵ��ֵ
    //assign control values
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}
static void gimbal_motor_raw_angle_control(Gimbal_Motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid��������
//pid data cleanup
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
