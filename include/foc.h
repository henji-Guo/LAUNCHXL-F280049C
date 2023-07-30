/*
 * foc.h
 *
 *  Created on: 2023年7月27日
 *      Author: GHJ
 */

#ifndef INCLUDE_FOC_H_
#define INCLUDE_FOC_H_

#include "stdint.h"
#include "math.h"

enum FOC_DIR{
    DIR_CW,
    DIR_CCW
};

struct foc {
    float       Id_ref;
    float       Iq_ref;
    float       Speed_ref;
    uint16_t    pwm_period;

    enum FOC_DIR direction;

    float       Iabc[3];
    float       Uabc[3];

    float       Ialpha;
    float       Ibeta;

    float       Ualpha;
    float       Ubeta;

    float       Id;
    float       Iq;

    float       Ud;
    float       Uq;

    float       Udc;

    float       thetaELEC;
    float       thetaMECH;
    float       thetaOLDMECH;
    float       thetaOFFSET;
    uint16_t    turns;

    float       motorSpeed;
    float       motorFreq;
    uint16_t    motorPoles;
    float       motorRs;
    float       motorLd;
    float       motorLq;

    float       Id_Kp;
    float       Id_Ki;
    float       Iq_Kp;
    float       Iq_Ki;
    float       Speed_Kp;
    float       Speed_Ki;

    float       Id_error;
    float       Iq_error;
    float       Speed_error;
    float       IdErrorAll;
    float       IqErrorAll;
    float       SpeedErrorAll;

    float       outMax;
    float       outMin;
    float       currentMax;
    float       currentMin;

    float       Tcm1;
    float       Tcm2;
    float       Tcm3;
    uint16_t       Sector;

    float       temperature;

    void (*clarke)(struct foc* foc_handle);
    void (*park)(struct foc* foc_handle);
    void (*ipark)(struct foc* foc_handle);
    void (*svpwm)(struct foc* foc_handle);
    void (*svpwm_setDuty)(struct foc* foc_handle);
    void (*Id_PI)(struct foc* foc_handle);
    void (*Iq_PI)(struct foc* foc_handle);
    void (*Speed_PID)(struct foc* foc_handle);
    void (*get_RadianAngle)(struct foc* foc_handle);
    void (*get_current_Iabc)(struct foc* foc_handle);
    void (*get_voltage_Uabc)(struct foc* foc_handle);
    void (*get_motorSpeed)(struct foc* foc_handle);
    void (*current_3R_reconstruction)(struct foc* foc_handle);
};
extern struct foc foc_motor1;
extern struct foc foc_motor2;

void foc_init(struct foc* foc_handle);
void foc_run(struct foc* foc_handle);

#endif /* INCLUDE_FOC_H_ */
