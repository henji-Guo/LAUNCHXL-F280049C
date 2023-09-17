/**
 * @file monitor.h
 * @author henji-Guo (1012323780@qq.com)
 * @brief 
 * @version 0.1
 * @date 2023-08-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef MONITOR_H
#define MONITOR_H
#include <stdint.h>

union monitor
{
    float payload;
    uint32_t val;
};

enum CMD{
    MONITOR_SET_Speed_ref = 0,
    MONITOR_SET_Iq_ref,
    MONITOR_SET_Id_ref,
    MONITOR_SET_Speed_Kp,
    MONITOR_SET_Speed_Ki,
    MONITOR_SET_Iq_Kp,
    MONITOR_SET_Iq_Ki,
    MONITOR_SET_Id_Kp,
    MONITOR_SET_Id_Ki,
    MONITOR_SET_Ud,
    MONITOR_SET_Uq,
    MONITOR_SET_MOTOR_FREQUENCY,
    MONITOR_SET_MOTOR_Poles,
    MONITOR_SET_MOTOR_Rs,
    MONITOR_SET_MOTOR_Ld,
    MONITOR_SET_MOTOR_Lq,
    MONITOR_SET_MOTOR_Flux,
};


void monitor_cmd(uint16_t *buf);

#endif
