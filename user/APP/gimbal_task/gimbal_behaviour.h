#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "Gimbal_Task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE = 0, //��̨����   tripod no power
  GIMBAL_INIT,           //��̨��ʼ��    tripod initialization
  GIMBAL_CALI,           //��̨У׼    tripod calibration
  GIMBAL_ABSOLUTE_ANGLE, //��̨�����Ǿ��ԽǶȿ���    reipod gyroscope absolute angle control
  GIMBAL_RELATIVE_ANGLE, //��̨�������ֵ��ԽǶȿ���    tripod motor coder relative angle control
  GIMBAL_MOTIONLESS,     //��̨��ң����������һ��ʱ��󱣳ֲ���������������Ư��    tripod keeps stationary after receiving no input from the
                         // the remote control for a while
                         // to avoid gyroscope shifting.
} gimbal_behaviour_e;

extern void gimbal_behaviour_mode_set(Gimbal_Control_t *gimbal_mode_set);

extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, Gimbal_Control_t *gimbal_control_set);
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern bool_t gimbal_cmd_to_shoot_stop(void);
#endif
