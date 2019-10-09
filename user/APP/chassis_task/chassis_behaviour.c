/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis_behaviour.c/h
  * @brief      完成底盘行为任务。//complete the chasis’s behavirol missions.
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
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0 //When the chasis is not enforced in behavioral conditon mode, the mode would be raw, then the set values would be set to 0.
//in the conditon that chasis does not have force, the mode of chassis is “raw”, so all the set values that send to “can” wire is 0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上 //The forward  velocity of chassis, the set values would be sent to the main wire can.
//vx_set is the speed of moving forward, the parameter will send to “can” wire directly
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上//vy_set the speed of moving left and right, the set values will send to the “can” wire directly
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上 //wz_set the speed of rotation, the set values will send directly to the “can” wire
  * @param[in]      chassis_move_rc_to_vector底盘数据 //chassis_move_rc_to_vector: the data of chassis
  * @retval         返回空 //return null
  */
static void chassis_zero_force_control(fp32 *vx_can_set, fp32 *vy_can_set, fp32 *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
//Under the behavioral conditon mode that chassis does not moving, the chassis mode is not following the angle.
  * @author         RM
  * @param[in]      vx_set前进的速度//vx_set  the speed of moving forward, the parameter will send to “can” wire directly
  * @param[in]      vy_set左右的速度 //vy_set  the speed of moving left and right, the parameter will send to the “can” wire directly
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度 //wz_set  the speed of rotation, the parameter will send directly to the “can” wire. Rotation speed is to control the angular speed of chassis
  * @param[in]      chassis_move_rc_to_vector底盘数据 //chassis data
  * @retval         返回空 //return null
  */
 
static void chassis_no_move_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度 //Under the conditon that the chassis is following the gimbal of behavioral conditon mode, the chassis mode is set to follow the angle of gimbal, the rotation speed of chassis would calculate the angular speed of chassis rotation according the difference of angles
  * @author         RM
  * @param[in]      vx_set前进的速度//vx_set  the speed of moving forward, the parameter will send to “can” wire directly
  * @param[in]      vy_set左右的速度 //vy_set  the speed of moving left and right, the parameter will send to the “can” wire directly
  * @param[in]      angle_set底盘与云台控制到的相对角度 //the relative angle between the chassis and controlled gimbal  
  * @param[in]      chassis_move_rc_to_vector底盘数据 //chassis data
  * @retval         返回空 //return null
  */
 
static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度 //under the conditon that the chassis behavioral conditon mode is following the chassis “yaw”, the chassis mode is set to follow the angle of chassis, the rotation speed of chassis would calculate the angular speed of chassis rotation according the difference of angles 
  * @author         RM
  * @param[in]      vx_set前进的速度//vx_set  the speed of moving forward
  * @param[in]      vy_set左右的速度 //vy_set  the speed of moving left and right
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI //the “yaw” set by the chassis, the range is from -PI to PI
  * @param[in]      chassis_move_rc_to_vector底盘数据 //chassis data
  * @retval         返回空 //return null
  */
 
static void chassis_engineer_follow_chassis_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定 //under the conditon that the chassis behavioral conditon mode is following the chassis “yaw”, the chassis mode is set to follow the angle of chassis,the rotation speed would be set directly by parameters
  * @author         RM
  * @param[in]      vx_set前进的速度//vx_set  the speed of moving forward
  * @param[in]      vy_set左右的速度//vy_set  the speed of moving left and right
  * @param[in]      wz_set底盘设置的旋转速度 //the rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector底盘数据 //chassis data
  * @retval         返回空 //return null
  */
static void chassis_no_follow_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
 
/**
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上 // Under the condition of chassis behavioral mode is open loop, the chassis mode is the original condition, so the the set values would be sent to the main wire can directly
  * @author         RM
  * @param[in]      vx_set前进的速度//vx_set  the speed of moving forward
  * @param[in]      vy_set左右的速度//vy_set  the speed of moving left and right
  * @param[in]      wz_set底盘设置的旋转速度//the rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector底盘数据//chassis data
  * @retval         返回空//return null//return null
  */
static void chassis_open_set_control(fp32 *vx_set, fp32 *vy_set, fp32 *wz_set, chassis_move_t *chassis_move_rc_to_vector);
 
//底盘行为状态机 //chassis behavioral status mode
static chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
 
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }
 
    //遥控器设置行为模式 //RC control behavioral mode
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
 
    //云台进入某些状态的时候，底盘保持不动 //When the gimbal entered some sort od conditon, the chasis stay static
 
    if (gimbal_cmd_to_chassis_stop())
    {
        chassis_behaviour_mode = CHASSIS_NO_MOVE;
    }
 
    //根据行为状态机选择底盘状态机 //select the chassis mode according to behavior conditon mode
    if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘无力，则设置底盘状态机为 raw，原生状态机。when the behavior is that the chassis is not enforced, then the chassis condition will be set to raw, the original condition mode
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不移动，则设置底盘状态机为 底盘不跟随角度 状态机。when the behavior is that the chassis is static, then set the condition of chassis to that the chassis not following the angle condition mode.
    }
    else if (chassis_behaviour_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW; //当行为是正常步兵跟随云台，则设置底盘状态机为 底盘跟随云台角度 状态机。When the behavior is the normal standard following gimbal, then set the chassis condition mode to chassis following gimbal’s angle condition mode
    }
    else if (chassis_behaviour_mode == CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW; //当行为是工程跟随底盘角度，则设置底盘状态机为 底盘跟随底盘角度 状态机。//when behavior is “Engineer” follow the angle of chassis, set the chassis conditon mode to the conditon mode that chassis follow the chassis angle
    }
    else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW; //当行为是底盘不跟随角度，则设置底盘状态机为 底盘不跟随角度 状态机。When the behavior is that the chassis not following angle, then set the chassis condition mode to  chassis not following angle  condition loop.
    }
    else if (chassis_behaviour_mode == CHASSIS_OPEN)
    {
 
        chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW; //当行为是底盘开环，则设置底盘状态机为 底盘原生raw 状态机。// when the behavior is chassis open-loop, then set the chassis condition mode to the origin “raw” condition mode
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
  * @brief          底盘无力的行为状态机下，底盘模式是raw，故而设定值会直接发送到can总线上故而将设定值都设置为0
  *                 under the condition when chassis has no power, chassis mode is raw, thus the set value will directly be sent
  *                 to the master CAN cable and the set values will all be 0
  * @author         RM
  * @param[in]      vx_set前进的速度 设定值将直接发送到can总线上
  * @param[in]      vy_set左右的速度 设定值将直接发送到can总线上
  * @param[in]      wz_set旋转的速度 设定值将直接发送到can总线上
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          底盘不移动的行为状态机下，底盘模式是不跟随角度，
  *                 UNder the condition where chasis is not moving, chassis mode does not follow angle
  * @author         RM
  * @param[in]      vx_set前进的速度 foward speed
  * @param[in]      vy_set左右的速度 left right speed
  * @param[in]      wz_set旋转的速度，旋转速度是控制底盘的底盘角速度
  *                 rotation speed, it is the chassis angle velocity that controls the angle
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
  * @brief          底盘跟随云台的行为状态机下，底盘模式是跟随云台角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  *                 Under the mode where chasis follows the gimbal, chasis mode follows gimbal angle, the rotation speed of 
  *                 the chasis will use the difference of the angle to calculate the angular velocity of the chasis rotation speed
  * @author         RM
  * @param[in]      vx_set前进的速度
  *                 vx_set forward speed
  * @param[in]      vy_set左右的速度
  *                 vy_set left right speed
  * @param[in]      angle_set底盘与云台控制到的相对角度
  *                 angle_set the relative anle that can be controlled by chasis and gimbal
  * @param[in]      chassis_move_rc_to_vector底盘数据
  *                 chassis_move_rc_to_vector chasis data
  * @retval         返回空
  *                 return null
  */

static void chassis_infantry_follow_gimbal_yaw_control(fp32 *vx_set, fp32 *vy_set, fp32 *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
    {
        return;
    }

    chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);

    //摇摆角度是利用sin函数生成，swing_time 是sin函数的输入值
    //swing angle is generated using sin(), swing_time is the input of sin()
    static fp32 swing_time = 0.0f;
    //swing_time 是计算出来的角度
    //swing_time is the calcuated angle
    static fp32 swing_angle = 0.0f;
    //max_angle 是sin函数的幅值
    //max_angle is the value of sin() ???????
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time 是摇摆角度改变的快慢，最大越快
    //add_time is the rate the swing angle changes, the bigger the faster
    static fp32 const add_time = PI / 250.0f;
    //使能摇摆标志位
    //????
    static uint8_t swing_flag = 0;

    //计算遥控器的原始输入信号
    //calculate original input signal of remote control

    //判断是否要摇摆
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

    //判断键盘输入是不是在控制底盘运动，底盘在运动减小摇摆角度
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
    //sin函数生成控制角度
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
    //sin函数不超过2pi
    //sin() does not exceed 2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }

    *angle_set = swing_angle;
}

/**
  * @brief          底盘跟随底盘yaw的行为状态机下，底盘模式是跟随底盘角度，底盘旋转速度会根据角度差计算底盘旋转的角速度
  *                 under the circumstance of chassis following chassis yaw, chassis mode follows chassis angle, 
  *                 chassis rotation speed will calculate chassis rotation angle based of angle difference
  * @author         RM
  * @param[in]      vx_set前进的速度
  *                 vs_set forwardd speed
  * @param[in]      vy_set左右的速度
  *                 vy_set left right speed
  * @param[in]      angle_set底盘设置的yaw，范围 -PI到PI
  *                 angle_set set the yaw of chassis, range from -pi to pi
  * @param[in]      chassis_move_rc_to_vector底盘数据
  *                 return chassis data
  * @retval         返回空
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
  * @brief          底盘不跟随角度的行为状态机下，底盘模式是不跟随角度，底盘旋转速度由参数直接设定
  *                 under the state of chassis not following the angle
  * @author         RM
  * @param[in]      vx_set前进的速度
  *                 vx_set forward speed
  * @param[in]      vy_set左右的速度
  *                 vy_set left righht speed
  * @param[in]      wz_set底盘设置的旋转速度
  *                 wz_set rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector底盘数据
  *                 chassis data
  * @retval         返回空
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
  * @brief          底盘开环的行为状态机下，底盘模式是raw原生状态，故而设定值会直接发送到can总线上
  *                 under the condition when chassis is open loop, chassis mode is raw and initial state, thus the set value
  *                 will directly send to the master cable of CAN
  * @author         RM
  * @param[in]      vx_set前进的速度
  *                 vx_set forward speed
  * @param[in]      vy_set左右的速度
  *                 vy_set left right speed
  * @param[in]      wz_set底盘设置的旋转速度
  *                 wz_set the rotation speed set by chassis
  * @param[in]      chassis_move_rc_to_vector底盘数据
  * @retval         返回空
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
