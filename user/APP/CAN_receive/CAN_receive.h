/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       can_receive.c/h
  * @brief      ���can�豸�����շ����������ļ���ͨ��can�ж���ɽ���
  *             complete can devices' communication function
  *             the data are received via can interrupt in this file's implementation
  * @note       ���ļ�����freeRTOS����
  *             the file is not a freeRTOS task
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

#ifndef CANTASK_H
#define CANTASK_H
#include "main.h"

#define CHASSIS_CAN CAN2
#define GIMBAL_CAN CAN1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_YAW_MOTOR_ID = 0x205,
    CAN_PIT_MOTOR_ID = 0x206,
    CAN_TRIGGER_MOTOR_ID = 0x207,
    CAN_GIMBAL_ALL_ID = 0x1FF,

} can_msg_id_e;

//rm���ͳһ���ݽṹ��
//uniform data structure for rm motor data
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

extern void CAN_CMD_CHASSIS_RESET_ID(void);

//������̨�����������revΪ�����ֽ�
//send tripod control command, rev is the reserved byte
extern void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);
//���͵��̵����������
//send chassis motor control command
extern void CAN_CMD_CHASSIS(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//����yaw���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
//return the variable address of the yaw motor, use pointer to obtain raw data
extern const motor_measure_t *get_Yaw_Gimbal_Motor_Measure_Point(void);
//����pitch���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
//return the variable address of the pitch motor, use pointer to obtain raw data
extern const motor_measure_t *get_Pitch_Gimbal_Motor_Measure_Point(void);
//����trigger���������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����
//return the variable address of the trigger motor, use pointer to obtain raw data
extern const motor_measure_t *get_Trigger_Motor_Measure_Point(void);
//���ص��̵��������ַ��ͨ��ָ�뷽ʽ��ȡԭʼ����,i�ķ�Χ��0-3����Ӧ0x201-0x204,
//return the variable address of the chassis motor, use pointer to obtain raw data
//range for i is between 0 and 3, corresponding to 0x201 to 0x204
extern const motor_measure_t *get_Chassis_Motor_Measure_Point(uint8_t i);

#if GIMBAL_MOTOR_6020_CAN_LOSE_SLOVE
extern void GIMBAL_lose_slove(void);
#endif

#endif
