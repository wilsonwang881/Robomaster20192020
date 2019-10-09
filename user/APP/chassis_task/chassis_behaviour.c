/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      ��ɵ�����Ϊ����//complete the chasis��s behavirol missions.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. complete
  *
  @verbatim
  ==============================================================================
 
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#include "chassis_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"
 
#include "gimbal_behaviour.h"
 
/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0 //When the chasis is not enforced in behavioral conditon mode, the mode would be raw, then the set values would be set to 0.
//in the conditon that chasis does not have force, the mode of chassis is ��raw��, so all the set values that send to ��can�� wire is 0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������ //The forward  velocity of chassis, the set values would be sent to the main wire can.
//vx_set is the speed of moving forward, the parameter will send to ��can�� wire directly
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������//vy_set the speed of moving left and right, the set values will send to the ��can�� wire directly
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������ //wz_set the speed of rotation, the set values will send directly to the ��can�� wire
  * @param[in]      chassis_move_rc_to_vector�������� //chassis_move_rc_to_vector: the data of chassis
  * @retval         ���ؿ� //return null
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
//Under the behavioral conditon mode that chassis does not moving, the chassis mode is not following the angle.
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�//vx_set  the speed of moving forward, the parameter will send to ��can�� wire directly
  * @param[in]      vy_set���ҵ��ٶ� //vy_set  the speed of moving left and right, the parameter will send to the ��can�� wire directly
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ� //wz_set  the speed of rotation, the parameter will send directly to the ��can�� wire. Rotation speed is to control the angular speed of chassis
  * @param[in]      chassis_move_rc_to_vector�������� //chassis data
  * @retval         ���ؿ� //return null
  */
 
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ� //Under the conditon that the chassis is following the gimbal of behavioral conditon mode, the chassis mode is set to follow the angle of gimbal, the rotation speed of chassis would calculate the angular speed of chassis rotation according the difference of angles
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�//vx_set  the speed of moving forward, the parameter will send to ��can�� wire directly
  * @param[in]      vy_set���ҵ��ٶ� //vy_set  the speed of moving left and right, the parameter will send to the ��can�� wire directly
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ� //the relative angle between the chassis and controlled gimbal  
  * @param[in]      chassis_move_rc_to_vector�������� //chassis data
  * @retval         ���ؿ� //return null
  */
 
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ� //under the conditon that the chassis behavioral conditon mode is following the chassis ��yaw��, the chassis mode is set to follow the angle of chassis, the rotation speed of chassis would calculate the angular speed of chassis rotation according the difference of angles 
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�//vx_set  the speed of moving forward
  * @param[in]      vy_set���ҵ��ٶ� //vy_set  the speed of moving left and right
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI //the ��yaw�� set by the chassis, the range is from -PI to PI
  * @param[in]      chassis_move_rc_to_vector�������� //chassis data
  * @retval         ���ؿ� //return null
  */
 
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨 //under the conditon that the chassis behavioral conditon mode is following the chassis ��yaw��, the chassis mode is set to follow the angle of chassis,the rotation speed would be set directly by parameters
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�//vx_set  the speed of moving forward
  * @param[in]      vy_set���ҵ��ٶ�//vy_set  the speed of moving left and right
  * @param[in]      wz_set�������õ���ת�ٶ� //the rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector�������� //chassis data
  * @retval         ���ؿ� //return null
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������ // Under the condition of chassis behavioral mode is open loop, the chassis mode is the original condition, so the the set values would be sent to the main wire can directly
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�//vx_set  the speed of moving forward
  * @param[in]      vy_set���ҵ��ٶ�//vy_set  the speed of moving left and right
  * @param[in]      wz_set�������õ���ת�ٶ�//the rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector��������//chassis data
  * @retval         ���ؿ�//return null//return null
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
 
//������Ϊ״̬�� //chassis behavioral status mode
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
 
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
 
    //ң����������Ϊģʽ //RC control behavioral mode
    if (switch_is_mid(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
    }
    else if (switch_is_down(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
    else if (switch_is_up(chassis_move_mode->chassis_RC->rc.s[MODE_CHANNEL]))
    {
        chassis_behaviour_mode = CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW;
    }
 
    //��̨����ĳЩ״̬��ʱ�򣬵��̱��ֲ��� //When the gimbal entered some sort od conditon, the chasis stay static
 
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
 
    //������Ϊ״̬��ѡ�����״̬�� //select the chassis mode according to behavior conditon mode
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //����Ϊ�ǵ��������������õ���״̬��Ϊ raw��ԭ��״̬����when the behavior is that the chassis is not enforced, then the chassis condition will be set to raw, the original condition mode
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲��ƶ��������õ���״̬��Ϊ ���̲�����Ƕ� ״̬����when the behavior is that the chassis is static, then set the condition of chassis to that the chassis not following the angle condition mode.
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //����Ϊ����������������̨�������õ���״̬��Ϊ ���̸�����̨�Ƕ� ״̬����When the behavior is the normal standard following gimbal, then set the chassis condition mode to chassis following gimbal��s angle condition mode
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; //����Ϊ�ǹ��̸�����̽Ƕȣ������õ���״̬��Ϊ ���̸�����̽Ƕ� ״̬����//when behavior is ��Engineer�� follow the angle of chassis, set the chassis conditon mode to the conditon mode that chassis follow the chassis angle
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //����Ϊ�ǵ��̲�����Ƕȣ������õ���״̬��Ϊ ���̲�����Ƕ� ״̬����When the behavior is that the chassis not following angle, then set the chassis condition mode to  chassis not following angle  condition loop.
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //����Ϊ�ǵ��̿����������õ���״̬��Ϊ ����ԭ��raw ״̬����// when the behavior is chassis open-loop, then set the chassis condition mode to the origin ��raw�� condition mode
    }
}
void chassis_behaviour_control_set(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
 
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
 
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
        chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
        chassis_infantry_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
        chassis_engineer_follow_chassis_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
        chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
        chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
    }
}

/**
  * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
  *                 under the condition when chassis has no power, chassis mode is raw, thus the set value will directly be sent
  *                 to the master CAN cable and the set values will all be 0
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_can_set = 0.0f;
    *vy_can_set = 0.0f;
    *wz_can_set = 0.0f;
}
/**
  * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
  *                 UNder the condition where chasis is not moving, chassis mode does not follow angle
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ� foward speed
  * @param[in]      vy_set���ҵ��ٶ� left right speed
  * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
  *                 rotation speed, it is the chassis angle velocity that controls the angle
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  */

static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }
    *vx_set = 0.0f;
    *vy_set = 0.0f;
    *wz_set = 0.0f;
}

/**
  * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  *                 Under the mode where chasis follows the gimbal, chasis mode follows gimbal angle, the rotation speed of 
  *                 the chasis will use the difference of the angle to calculate the angular velocity of the chasis rotation speed
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  *                 vx_set forward speed
  * @param[in]      vy_set���ҵ��ٶ�
  *                 vy_set left right speed
  * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
  *                 angle_set the relative anle that can be controlled by chasis and gimbal
  * @param[in]      chassis_move_rc_to_vector��������
  *                 chassis_move_rc_to_vector chasis data
  * @retval         ���ؿ�
  *                 return null
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //ҡ�ڽǶ�������sin�������ɣ�swing_time ��sin����������ֵ
    //swing angle is generated using sin(), swing_time is the input of sin()
    static fp32 swing_time = 0.0f;
    //swing_time �Ǽ�������ĽǶ�
    //swing_time is the calcuated angle
    static fp32 swing_angle = 0.0f;
    //max_angle ��sin�����ķ�ֵ
    //max_angle is the value of sin() ???????
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time ��ҡ�ڽǶȸı�Ŀ��������Խ��
    //add_time is the rate the swing angle changes, the bigger the faster
    static fp32 const add_time = PI / 250.0f;
    //ʹ��ҡ�ڱ�־λ
    //????
    static uint8_t swing_flag = 0;

    //����ң������ԭʼ�����ź�
    //calculate original input signal of remote control

    //�ж��Ƿ�Ҫҡ��
    //determine whether swinging
    if (chassis_move_rc_to_vector->chassis_RC->key.v & SWING_KEY)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }

    //�жϼ��������ǲ����ڿ��Ƶ����˶����������˶���Сҡ�ڽǶ�
    //determin whether the keyboardd input is controlling the chassis movement, (if) chassis is moving decrease swinging angle
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY ||
        chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY || chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
    //sin�������ɿ��ƽǶ�
    //sin() generate control angle
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //sin����������2pi
    //sin() does not exceed 2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}

/**
  * @brief          ���̸������yaw����Ϊ״̬���£�����ģʽ�Ǹ�����̽Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
  *                 under the circumstance of chassis following chassis yaw, chassis mode follows chassis angle, 
  *                 chassis rotation speed will calculate chassis rotation angle based of angle difference
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  *                 vs_set forwardd speed
  * @param[in]      vy_set���ҵ��ٶ�
  *                 vy_set left right speed
  * @param[in]      angle_set�������õ�yaw����Χ -PI��PI
  *                 angle_set set the yaw of chassis, range from -pi to pi
  * @param[in]      chassis_move_rc_to_vector��������
  *                 return chassis data
  * @retval         ���ؿ�
  *                 return null
  */

static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    *angle_set = rad_format(chassis_move_rc_to_vector->chassis_yaw_set - CHASSIS_ANGLE_Z_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL]);
}

/**
  * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
  *                 under the state of chassis not following the angle
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  *                 vx_set forward speed
  * @param[in]      vy_set���ҵ��ٶ�
  *                 vy_set left righht speed
  * @param[in]      wz_set�������õ���ת�ٶ�
  *                 wz_set rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector��������
  *                 chassis data
  * @retval         ���ؿ�
  *                 return null
  */                 

static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
    *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
  * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
  *                 under the condition when chassis is open loop, chassis mode is raw and initial state, thus the set value
  *                 will directly send to the master cable of CAN
  * @author         RM
  * @param[in]      vx_setǰ�����ٶ�
  *                 vx_set forward speed
  * @param[in]      vy_set���ҵ��ٶ�
  *                 vy_set left right speed
  * @param[in]      wz_set�������õ���ת�ٶ�
  *                 wz_set the rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector��������
  * @retval         ���ؿ�
  *                 return null
  */

static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
    return;
}
