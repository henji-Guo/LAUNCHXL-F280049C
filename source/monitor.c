#include "monitor.h"
#include "foc.h"

/* 'S'(1字节) + motorID(1字节) + CMD(1字节) + PLAYLOAD + 'P'(1字节) */
void monitor_cmd(uint16_t *buf)
{
    union monitor pVar;
    struct foc *phandle;

    /* check frame 'S' and 'P' */
    if(buf[0] == 0x53 && buf[7] == 0x50){
        
        /* motorID */
        switch (buf[1]){
            case 1: phandle = &foc_motor1; break;
            case 2: phandle = &foc_motor1; break;
            default: return;
        }

        pVar.val = ((uint32_t)buf[6] << 24 | (uint32_t)buf[5] << 16 | buf[4] << 8 | buf[3] << 0 );

        /* CMD */
        switch (buf[2]){
            case MONITOR_SET_Speed_ref: phandle->Speed_ref = pVar.payload; break;
            case MONITOR_SET_Iq_ref: phandle->Iq_ref = pVar.payload; break;
            case MONITOR_SET_Id_ref: phandle->Id_ref = pVar.payload; break;
            case MONITOR_SET_Speed_Kp: phandle->Speed_Kp = pVar.payload; break;
            case MONITOR_SET_Speed_Ki: phandle->Speed_Ki = pVar.payload; break;
            case MONITOR_SET_Iq_Kp: phandle->Iq_Kp = pVar.payload; break;
            case MONITOR_SET_Iq_Ki: phandle->Iq_Ki = pVar.payload; break;
            case MONITOR_SET_Id_Kp: phandle->Id_Kp = pVar.payload; break;
            case MONITOR_SET_Id_Ki: phandle->Id_Ki = pVar.payload; break;
            case MONITOR_SET_Ud: phandle->Ud = pVar.payload; break;
            case MONITOR_SET_Uq: phandle->Uq = pVar.payload; break;
            case MONITOR_SET_MOTOR_FREQUENCY: phandle->motorFreq = pVar.payload; break;
            case MONITOR_SET_MOTOR_Poles: phandle->motorPoles = pVar.payload; break;
            case MONITOR_SET_MOTOR_Rs: phandle->motorRs = pVar.payload; break;
            case MONITOR_SET_MOTOR_Ld: phandle->motorLd = pVar.payload; break;
            case MONITOR_SET_MOTOR_Lq: phandle->motorLq = pVar.payload; break;
            case MONITOR_SET_MOTOR_Flux: phandle->motorFlux = pVar.payload; break;
        }

    }
    return;
}
