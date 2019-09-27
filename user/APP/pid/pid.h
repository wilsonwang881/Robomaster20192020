/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       pid.c/h
  * @brief      pid function, including initialization and calculation
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. Íê³É
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef PID_H
#define PID_H
#include "main.h"
enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //three coefficients of PID
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out;  //maximum output
    fp32 max_iout; //maximum integral output

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  //differential term, 0 is the latest, 1 is the last one, 2 is the one before the last one
    fp32 error[3]; //error term, 0 is the latest, 1 is the last one, 2 is the one before the last one

} PidTypeDef;
extern void PID_Init(PidTypeDef *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);
extern fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set);
extern void PID_clear(PidTypeDef *pid);
#endif
