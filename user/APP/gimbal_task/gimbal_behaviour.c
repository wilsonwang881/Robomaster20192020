/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  *             Finish tripod control task. Due to the fact that the tripod use
  *             the angle computed from the gyroscope, its range is between -pi
  *             and pi. Therefore its set targets are all ranges. Many functions
  *             for computing the angles exist. The tripod has two modes, the 
  *             gyroscope-control mode uses the angle computed by the onboard 
  *             gyroscope. The coder-control mode is the calibration controlled
  *             by the code that is feedback from the motor. Other modes include
  *             calibration mode, stopping mode, ect
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

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "buzzer.h"
#include "Detect_Task.h"

#include "user_lib.h"

////��̨У׼����������
// tripod calibration beeping
//#define GIMBALWarnBuzzerOn() buzzer_on(31, 20000)
//#define GIMBALWarnBuzzerOFF() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))
/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ���Ƿ���1024������
  *                 remote control dead zone control. It is because when the joystick
  *                 of the remote control is in the middle position, it is not necessarily
  *                 sending 1024.
  * @author         RM
  * @param[in]      �����ң����ֵ input value from the remote control
  * @param[in]      ��������������ң����ֵ processed output dead zone value
  * @param[in]      ����ֵ dead zone value
  * @retval         ���ؿ� return null
  */
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          ��̨У׼��ͨ���жϽ��ٶ����ж���̨�Ƿ񵽴Ｋ��λ��
  *                 calibrated by the tripod, use the angular speed to 
  *                 determine whether the tripod is at extreme positions
  * @author         RM
  * @param[in]      ��Ӧ��Ľ��ٶȣ���λrad/s record axial angular speed in rad/s
  * @param[in]      ��ʱʱ�䣬����GIMBAL_CALI_STEP_TIME��ʱ������
  *                 recording time, reset to 0 after it reaches GIMBAL_CALT_STEP_TIME
  * @param[in]      ��¼�ĽǶ� rad    recorded angle, in rad
  * @param[in]      �����ĽǶ� rad    feedback angle, in rad
  * @param[in]      ��¼�ı���ֵ raw  recorded coding value, raw
  * @param[in]      �����ı���ֵ raw  feedback coding calue, raw
  * @param[in]      У׼�Ĳ��� ���һ�� ��һ   calibration steps, add one when it is completed once
  * @retval         ���ؿ�            return null
  */
#define GIMBAL_CALI_GYRO_JUDGE(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }

/**
  * @brief          ��̨��Ϊ״̬�����ã���Ϊ��cali��ģʽ��ʹ����return���ʶ�������һ������
  *                 tripod behaviour state machine set. Because return is used under modes
  *                 like cali, another function is used.
  * @author         RM
  * @param[in]      ��̨����ָ��    tripod data pointer
  * @retval         ���ؿ�          return null
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set);

/**
  * @brief          ��̨�������ƣ������ģʽ�·��͵�yaw��pitch �ǵ������ԭʼֵ����̨�������can���������ʹ����̨����
  *                 tripod no-power control. Under this mode, the yaw and the pitch sent
  *                 are the raw motor control value. Tripod motor sends can zero control
  *                 value and no-powers the tripod
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 send the raw value of the yaw motor via can to the motor directly
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 send pitch motor raw value via can to the motor directly
  * @param[in]      ��̨����ָ��    tripod data pointer
  * @retval         ���ؿ�    return  null
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  *                 Tripod initialization control. Motoris the gyroscope angle control.
  *                 Tripod lifts the pitch axis then rotates the yaw axis.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 yaw axis angle control. It is the increment of the angle in rad.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 pitch axis angle control. It is the increment of the angle in rad.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  *                 Tripod calibration control. Motor is controlled using raw.
  *                 Tripod lifts up pitch first, lowers pitch, then rotates yaw clockwisely and finally rotates yaw
  *                 anticlockwisely, recording the angle and the coding value at that time.
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 Send the raw value of the yaw motor via can to the motor directly.
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 Send the raw value of the pitch motor via can to the motor directly.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  *                 Tripod gyroscope control. The motor is gyroscope angle control.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 yaw axis angle control. It is the increment of the angle in rad.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 pitch axis angle control. It is the increment of the angle in raw.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  *                 Tripod coding value control. Motor is the relative angle control.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 yaw axis angle control. It is the increment of the angle in rad.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 pitch axis angle control. It is the increment of the angle in rad.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null./\
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  *                 Tripod enters remote control no input control mode.
  *                 The motor is relative angle control.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 yaw axis angle control. It is the increment of the angle in radians.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 pitch axis angle control. It is the increment of the angle in radians.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set);

//��̨��Ϊ״̬��
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          ��̨��Ϊ״̬���Լ����״̬������
  *                 Tripod behaviour state machine and motor state machine setup.
  * @author         RM
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */

gimbal_motor_mode_e test = GIMBAL_MOTOR_ENCONDE;
void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //��̨��Ϊ״̬������
    //tripod behaviour state machine setup.
    gimbal_behavour_set(gimbal_mode_set);

    //������̨��Ϊ״̬�����õ��״̬��
    //setup the motor state machine based on the tripod behaviour state machine.
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
}

/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  *                 Tripod bahaviour control. Use different control functions based on different behaviour.
  * @author         RM
  * @param[in]      ���õ�yaw�Ƕ�����ֵ����λ rad
  *                 Set the yaw angle increment in radians.
  * @param[in]      ���õ�pitch�Ƕ�����ֵ����λ rad
  *                 Set the pitch angle increment in radians.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null
  */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static fp32 rc_add_yaw, rc_add_pit;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //��ң���������ݴ������� int16_t yaw_channel,pitch_channel
    //remote control data processing dead zone
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YawChannel], yaw_channel, RC_deadband);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PitchChannel], pitch_channel, RC_deadband);

    rc_add_yaw = yaw_channel * Yaw_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * Yaw_Mouse_Sen;
    rc_add_pit = pitch_channel * Pitch_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * Pitch_Mouse_Sen;

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
    }
    //��������������ֵ
    //assign the increment value of control
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pit;
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  *                 Under certain conditions, the tripod needs the chassis to be stationary. 
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�    Return null.
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
  *                 Under certain conditions, the tripod needs to stop shooting.
  * @author         RM
  * @param[in]      void
  * @retval         ���ؿ�    Return null.
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
  * @brief          ��̨��Ϊ״̬�����ã���Ϊ��cali��ģʽ��ʹ����return���ʶ�������һ������
  *                 Tripod bebaviour state machine setup.
  *                 Because return is used under modes like cali, another function is used.
  * @author         RM
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_behavour_set(Gimbal_Control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //У׼��Ϊ��return ��������������ģʽ
    //Calibration behaviour, return will not set other modes.
    if (gimbal_behaviour == GIMBAL_CALI && gimbal_mode_set->gimbal_cali.step != GIMBAL_CALI_END_STEP)
    {
        return;
    }
    //����ⲿʹ��У׼�����0 ��� start�������У׼ģʽ
    //If external conditions change the calibration step from 0 to start, then enter calibration mode.
    if (gimbal_mode_set->gimbal_cali.step == GIMBAL_CALI_START_STEP)
    {
        gimbal_behaviour = GIMBAL_CALI;
        return;
    }

    //��ʼ��ģʽ�ж��Ƿ񵽴���ֵλ��
    //Initialization mode determines whether the middle value mode is reached.
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        //������ֵ ��ʱ
        //record time when the middle value is reached.
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            //�����ʼ��λ��
            //reach the initialization position.
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            //û�е����ʼ��λ�ã�ʱ���ʱ
            //time recording when the initialization position is not reached.
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //������ʼ�����ʱ�䣬�����Ѿ��ȶ�����ֵһ��ʱ�䣬�˳���ʼ��״̬���ش��µ������ߵ���
        //exit the initialization state, offline the switch
        //when the time exceeds the maximum initialization time
        //or when the middle value has been reached stably for a period of time.
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]) && !toe_is_error(DBUSTOE))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //���ؿ��� ��̨״̬
    //switch control, tripod state
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[ModeChannel]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }

    if( toe_is_error(DBUSTOE))
    {

        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    //�жϽ���init״̬��
    //determine wheter it is in init state machine
    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }

    static uint16_t motionless_time = 0;
    if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        //ң���� ���̾������룬����motionless״̬
        //enter motionless state when the remote control and the keyboard both have no input.
        if (int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[0]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[1]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[2]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->rc.ch[3]) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.x) < GIMBAL_MOTIONLESS_RC_DEADLINE && int_abs(gimbal_mode_set->gimbal_rc_ctrl->mouse.y) < GIMBAL_MOTIONLESS_RC_DEADLINE && gimbal_mode_set->gimbal_rc_ctrl->key.v == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_l == 0 && gimbal_mode_set->gimbal_rc_ctrl->mouse.press_r == 0)
        {
            if (motionless_time < GIMBAL_MOTIONLESS_TIME_MAX)
            {
                motionless_time++;
            }
        }
        else
        {
            motionless_time = 0;
        }

        if (motionless_time == GIMBAL_MOTIONLESS_TIME_MAX)
        {
            gimbal_behaviour = GIMBAL_MOTIONLESS;
        }
    }
    else
    {
        motionless_time = 0;
    }


}

/**
  * @brief          ��̨�������ƣ������ģʽ�·��͵�yaw��pitch �ǵ������ԭʼֵ����̨�������can���������ʹ����̨����
  *                 Tripod no-power control.
  *                 Yaw and pitch sent under this mode are the raw control value of the motor.
  *                 Tripod motor sends can zero control value to no-power the tripod.
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 Send yaw motor's raw value. It will be sent to the motor via can directly.
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 Send the raw value of the pitch motor via can to the motor directly.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  *                 Tripod initialization control. The motor is gyroscope angle control.
  *                 Tripod lifts up pitch axis first then rotates yaw axis.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Yaw axis angle control. It is the increment of the angle in radians.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Pitch axis angle control. It is the increment of the angle in radians.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //��ʼ��״̬����������
    //initialization state control value computation.
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  *                 Tripod calibration control. Motor is raw control. The tripod lifts up pitch first then lowers pitch.
  *                 Then rotates yaw clockwisely. Then rotates yaw anticlockwisely.
  *                 Record the angle and the coding value at that time.
  * @author         RM
  * @param[in]      ����yaw�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 Send the raw value of the yaw motor. It will be sent to the motor via can directly.
  * @param[in]      ����pitch�����ԭʼֵ����ֱ��ͨ��can ���͵����
  *                 Send the raw value of the pitch motor. It will be sent to the motor via can directly.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //�ж����������ݣ� ����¼�����С�Ƕ�����
        //determine gyroscope data and record and the max and min angle data.
        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        GIMBAL_CALI_GYRO_JUDGE(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}
/**
  * @brief          ��̨�����ǿ��ƣ�����������ǽǶȿ��ƣ�
  *                 Tripod gyroscope control. Motor is gyroscope angle control.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Yaw axis angle control. It is the increment of the angle in radians.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Pitch axis angle control. It is the increment of the angle in radians.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_absolute_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;
        static fp32 gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TurnKeyBoard) && !(last_turn_keyboard & TurnKeyBoard))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //�����ͷ��Ŀ��ֵ
                //save the target value for turning around.
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //���Ͽ��Ƶ���ͷ��Ŀ��ֵ����ת����װ�����
            //keep controlling the target value for turning around.
            //rotating clockwisely
            //rotating anticlockwisely is random.
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TurnSpeed;
            }
            else
            {
                *yaw -= TurnSpeed;
            }
        }
        //����pi ��180�㣩��ֹͣ
        //stop after reaching pi radians.
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}
/**
  * @brief          ��̨����ֵ���ƣ��������ԽǶȿ��ƣ�
  *                 Tripod coding value control. The motor is relative angle control.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Yaw axis angle control. It is the increment of the angle in radians.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Pitch axis angle control. It is the increment of the angle in radians.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_relative_angle_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //����Ҫ����
    //Do not need processing.
}
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  *                 Tripod enters remote control no input control.
  *                 Motor is the relative angle control.
  * @author         RM
  * @param[in]      yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 yaw axis angle control. It is the increment of the angle in radians.
  * @param[in]      pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  *                 Pitch axis angle control. It is the increment of the angle in radians.
  * @param[in]      ��̨����ָ��    Tripod data pointer.
  * @retval         ���ؿ�    Return null.
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}
