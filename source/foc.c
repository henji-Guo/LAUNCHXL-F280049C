/*
 * foc.c
 *
 *  Created on: 2023/07/23
 *      Author: GHJ
 */
#include "foc.h"

struct foc foc_motor1;
struct foc foc_motor2;

/**
 * @brief Clarke Transform
 *
 * @param foc_handle pointer to foc component handler
 */
void clarke(struct foc* foc_handle)
{
    foc_handle->Ialpha = foc_handle->Iabc[0];
    foc_handle->Ibeta  = ( foc_handle->Iabc[0] + 2 * foc_handle->Iabc[1] ) / sqrtf(3.0f);
}

/**
 * @brief Inverse Clarke Transform
 *
 * @param foc_handle pointer to foc component handler
 */
void iclarke(struct foc* foc_handle)
{
    foc_handle->Uabc[0] = foc_handle->Ualpha;
    foc_handle->Uabc[1] = (-foc_handle->Ubeta + sqrtf(3.0f) * foc_handle->Ualpha) / 2.0f;
    foc_handle->Uabc[2] = (-foc_handle->Ubeta - sqrtf(3.0f) * foc_handle->Ualpha) / 2.0f;
}

/**
 * @brief Park Transform
 *
 * @param foc_handle pointer to foc component handler
 */
void park(struct foc* foc_handle)
{
    float cosVal,sinVal;

    cosVal = cosf(foc_handle->thetaELEC);
    sinVal = sinf(foc_handle->thetaELEC);

    foc_handle->Id =  foc_handle->Ialpha * cosVal + foc_handle->Ibeta * sinVal;
    foc_handle->Iq = -foc_handle->Ialpha * sinVal + foc_handle->Ibeta * cosVal;
}

/**
 * @brief Inverse-Park transform
 *
 * @param foc_handle pointer to foc component handler
 */
void ipark(struct foc* foc_handle)
{
    float cosVal,sinVal;

    cosVal = cosf(foc_handle->thetaELEC);
    sinVal = sinf(foc_handle->thetaELEC);
    
    foc_handle->Ualpha = foc_handle->Ud * cosVal - foc_handle->Uq * sinVal;
    foc_handle->Ubeta  = foc_handle->Ud * sinVal + foc_handle->Uq * cosVal;
}

/**
 * @brief SVPWM OverModulation Process, T1+T2 should less than 1.0f.
 *
 * @param T1 Basic vector time T1
 * @param T2 Basic vector time T2
 */
static void svpwm_overmodulation1(float *T1, float *T2)
{
    if((*T1 + *T2) > 1.0f){
        *T1 = *T1 / (*T1 + *T2);
        *T2 = *T2 / (*T1 + *T2);
    }
}

/**
 * @brief Traditional SVPWM algorithm
 *
 * @param foc_handle pointer to foc component handler
 */
void svpwm(struct foc* foc_handle)
{
    /**
     *  N = 4C + 2B + A :
     *      Ubeta > 0 ? A = 1 : A = 0;
     *      sqrtf(3)/2 * Ualpha - 0.5 * Ubeta > 0 ? B = 1 : B = 0;
     *      -sqrtf(3)/2 * Ualpha - 0.5 * Ubeta > 0 ? C = 1 : C = 0;
     *  X Y Z :
     *      X = (sqrtf(3) * Ts) / Udc * Ubeta;
     *      Y = (sqrtf(3) * Ts) / Udc * (sqrtf(3)/2 * Ualpha + 0.5 * Ubeta);
     *      Z = (sqrtf(3) * Ts) / Udc * (-sqrtf(3)/2 * Ualpha + 0.5 * Ubeta);
     *  Ta Tb Tc :
     *      Ta = (Ts - T1 - T2) / 4;
     *      Tb = Ta + T1 / 2;
     *      Tc = Tb + T2 / 2;
     *  Relationship table :
     *      |    N   |   3  |   1  |   5  |   4  |   6  |   2  |
     *      | sector |  one | two  | three| four | five |  six |
     *      |   T1   |  -Z  |   Z  |   X  |  -X  |  -Y  |   Y  |
     *      |   T2   |   X  |   Y  |  -Y  |   Z  |  -Z  |  -X  |
     *      |   T0   |      T0/T7 = （Ts - T1 - T2) / 2
     *      |  Tcm1  |  Ta  |  Tb  |  Tc  |  Tc  |  Tb  |  Ta  |
     *      |  Tcm2  |  Tb  |  Ta  |  Ta  |  Tb  |  Tc  |  Tc  |
     *      |  Tcm3  |  Tc  |  Tc  |  Tb  |  Ta  |  Ta  |  Tb  |
     */
    /* auxiliary variables */
    uint16_t N;
    float X,Y,Z;
    float Ta,Tb,Tc;
    float T1,T2;
    float Tcm1,Tcm2,Tcm3;
    float Ualpha,Ubeta,Udc;

    /* set Udc Ualpha Ubeta value */
    Udc = foc_handle->Udc;
    Ualpha = foc_handle->Ualpha;
    Ubeta = foc_handle->Ubeta;

    /* calculate N */
    N = Ubeta > 0 ? 1 : 0;
    N += (sqrtf(3.0f)/2.0f * Ualpha - 0.5f * Ubeta > 0 ? 1 : 0) << 1;
    N += (sqrtf(3.0f)/-2.0f * Ualpha - 0.5f * Ubeta > 0 ? 1 : 0) << 2;

    /* calculate X Y Z , Ts is PWM period counter */
    X = (sqrtf(3.0f) * 1.0f) / Udc * Ubeta;
    Y = (sqrtf(3.0f) * 1.0f) / Udc * (sqrtf(3.0f)/2.0f * Ualpha + 0.5f * Ubeta);
    Z = (sqrtf(3.0f) * 1.0f) / Udc * (sqrtf(3.0f)/-2.0f * Ualpha + 0.5f * Ubeta);

    /* identify sectors for T1 T2 */
    switch (N){
        case 1:
            T1 = Z;
            T2 = Y;
            foc_handle->Sector = 2;
            break;
        case 2:
            T1 =  Y;
            T2 = -X;
            foc_handle->Sector = 6;
            break;
        case 3:
            T1 = -Z;
            T2 =  X;
            foc_handle->Sector = 1;
            break;
        case 4:
            T1 = -X;
            T2 =  Z;
            foc_handle->Sector = 4;
            break;
        case 5:
            T1 =  X;
            T2 = -Y;
            foc_handle->Sector = 3;
            break;
        case 6:
            T1 = -Y;
            T2 = -Z;
            foc_handle->Sector = 5;
            break;
        default:
            T1 = 0;
            T2 = 0;
            break;
    }

    /* overmodulation process */
    svpwm_overmodulation1(&T1,&T2);

    /* calculate Ta Tb Tc */
    Ta = (1 - T1 - T2) / 4.0f;
    Tb = Ta + T1 / 2.0f;
    Tc = Tb + T2 / 2.0f;

    /* calculate Tcm1 Tcm2 Tcm3 */
    switch (N){
        case 1:
            Tcm1 = Tb;
            Tcm2 = Ta;
            Tcm3 = Tc;
            break;
        case 2:
            Tcm1 = Ta;
            Tcm2 = Tc;
            Tcm3 = Tb;
            break;
        case 3:
            Tcm1 = Ta;
            Tcm2 = Tb;
            Tcm3 = Tc;
            break;
        case 4:
            Tcm1 = Tc;
            Tcm2 = Tb;
            Tcm3 = Ta;
            break;
        case 5:
            Tcm1 = Tc;
            Tcm2 = Ta;
            Tcm3 = Tb;
            break;
        case 6:
            Tcm1 = Tb;
            Tcm2 = Tc;
            Tcm3 = Ta;
            break;
        default:
            Tcm1 = 0;
            Tcm2 = 0;
            Tcm3 = 0;
            break;
    }

    /* save Tcm1 Tcm2 Tcm3 */
    foc_handle->Tcm1 = Tcm1;
    foc_handle->Tcm2 = Tcm2;
    foc_handle->Tcm3 = Tcm3;

    /* set pwm duty */
    foc_handle->svpwm_setDuty(foc_handle);

}

/**
 * @brief Fast SVPWM algorithm
 *
 * @param foc_handle pointer to foc component handler
 */
void fast_svpwm(struct foc* foc_handle)
{
    int svmMode = 0;

    float Vmax_pu = 0,Vmin_pu = 0,Vcom_pu;
    float oneOverDcBus_invV = (float)(1.0f / foc_handle->Udc);

    float Valpha_pu = foc_handle->Ualpha * oneOverDcBus_invV;
    float Vbeta_pu = foc_handle->Ubeta * oneOverDcBus_invV;

    float Va_tmp = (float)(0.5f) * Valpha_pu;
    float Vb_tmp = sqrtf(3.0) / 2.0f * Vbeta_pu;

    float Va_pu = Valpha_pu;

    //
    // -0.5*Valpha + sqrt(3)/2 * Vbeta
    //
    float Vb_pu = -Va_tmp + Vb_tmp;

    //
    // -0.5*Valpha - sqrt(3)/2 * Vbeta
    float Vc_pu = -Va_tmp - Vb_tmp;

    //
    // Find Vmax and Vmin
    //
    if(Va_pu > Vb_pu)
    {
        Vmax_pu = Va_pu;
        Vmin_pu = Vb_pu;
    }
    else
    {
        Vmax_pu = Vb_pu;
        Vmin_pu = Va_pu;
    }

    if(Vc_pu > Vmax_pu)
    {
        Vmax_pu = Vc_pu;
    }
    else if(Vc_pu < Vmin_pu)
    {
        Vmin_pu = Vc_pu;
    }

    // Compute Vcom = 0.5*(Vmax+Vmin)
    Vcom_pu = 0.5f * (Vmax_pu + Vmin_pu);

    if(svmMode == 0)
    {
        /* CSVPWM */
        // Subtract common-mode term to achieve SV modulation
        foc_handle->Tcm1 = -(Va_pu - Vcom_pu);
        foc_handle->Tcm2 = -(Vb_pu - Vcom_pu);
        foc_handle->Tcm3 = -(Vc_pu - Vcom_pu);
    }
    else if(svmMode == 1)
    {
        /* DPWMMIN(-120°DPWM) */
        foc_handle->Tcm1 = -((Va_pu - Vmin_pu) - 0.5f);
        foc_handle->Tcm2 = -((Vb_pu - Vmin_pu) - 0.5f);
        foc_handle->Tcm3 = -((Vc_pu - Vmin_pu) - 0.5f);
    }
    else if(svmMode == 2)
    {
        /* DPWMMAX(+120°DPWM) */
        foc_handle->Tcm1 = -((Va_pu - Vmax_pu) + 0.5f);
        foc_handle->Tcm2 = -((Vb_pu - Vmax_pu) + 0.5f);
        foc_handle->Tcm3 = -((Vc_pu - Vmax_pu) + 0.5f);
    }

    /* save Tcm1 Tcm2 Tcm3 */
    foc_handle->Tcm1 += 0.5f;
    foc_handle->Tcm2 += 0.5f;
    foc_handle->Tcm3 += 0.5f;

    /* set pwm duty */
    foc_handle->svpwm_setDuty(foc_handle);

}

/**
 * @brief Saturation function to limit the output.
 *
 * @param val The input val
 * @param Max The output Maximum
 * @param Min The output Minimum
 * @return float The final output value
 */
static inline float saturation(float val, float Max, float Min)
{
    return ( val > Max ) ? Max : ( val < Min ) ? Min : val;
}

/**
 * @brief Id PI algorithm
 *
 * @param foc_handle pointer to foc component handler
 * @return float The Current Id loop output val
 */
void Id_PI(struct foc* foc_handle)
{
    float Ui,Up;

    /* calculate Id error */
    foc_handle->Id_error = foc_handle->Id_ref - foc_handle->Id;

    /* calculate Kp * error */
    Up = foc_handle->Id_Kp * foc_handle->Id_error;

    /* calculate new Integral error , Intergral Error + Kp * Ki * Integral error */
    Ui = foc_handle->IdErrorAll + foc_handle->Id_Ki * Up;

    /* Anti-Integrating saturation */
    Ui = saturation(Ui, foc_handle->outMax, foc_handle->outMin);

    /* save Integral error */
    foc_handle->IdErrorAll = Ui;

    /* calculate out */
    foc_handle->Ud = saturation((Up + Ui), foc_handle->outMax, foc_handle->outMin);
}

/**
 * @brief Iq PI algorithm
 *
 * @param foc_handle pointer to foc component handler
 * @return float The Current Iq loop output val
 */
void Iq_PI(struct foc* foc_handle)
{
    float Ui,Up;

    /* calculate Iq error */
    foc_handle->Iq_error = foc_handle->Iq_ref - foc_handle->Iq;

    /* calculate Kp * error */
    Up = foc_handle->Iq_Kp * foc_handle->Iq_error;

    /* calculate new Integral error , Intergral Error + Kp * Ki * Integral error */
    Ui = foc_handle->IqErrorAll + foc_handle->Iq_Ki * Up;

    /* Anti-Integrating saturation */
    Ui = saturation(Ui, foc_handle->outMax, foc_handle->outMin);

    /* save Integral error */
    foc_handle->IqErrorAll = Ui;

    /* calculate out */
    foc_handle->Uq = saturation((Up + Ui), foc_handle->outMax, foc_handle->outMin);
}

/**
 * @brief Speed PID algorithm
 *
 * @param foc_handle pointer to foc component handler
 * @return float The Speed loop output val
 */
void Speed_PID(struct foc* foc_handle)
{
    float Ui,Up;

    /* calculate Speed error */
    foc_handle->Speed_error = foc_handle->Speed_ref - fabsf(foc_handle->motorSpeed);

    /* calculate Kp * error */
    Up = foc_handle->Speed_Kp * foc_handle->Speed_error;

    /* calculate new Integral error , Intergral Error + Kp * Ki * Integral error */
    Ui = foc_handle->SpeedErrorAll + foc_handle->Speed_Ki * Up;

    /* Anti-Integrating saturation */
    Ui = saturation(Ui, foc_handle->outMax, foc_handle->outMin);

    /* save Integral error */
    foc_handle->SpeedErrorAll = Ui;

    /* calculate out */
    foc_handle->Iq_ref = saturation((Up + Ui), foc_handle->outMax, foc_handle->outMin);

}


/**
 * @brief Set PWM Duty
 *
 * @param Ua Set A phase PWM duty
 * @param Ub Set B phase PWM duty
 * @param Uc Set C phase PWM duty
 */
__attribute__((__weak__))
void svpwm_setDuty(struct foc* foc_handle)
{
    //TODO
    /* Like that :
    
    uint16_t period = foc_handle->pwm_period;
    pwm_setDuty(foc_handle->Tcm1 * period,
                foc_handle->Tcm2 * period,
                foc_handle->Tcm3 * period);

    */
}

/**
 * @brief Get the motor mechanical radian angle
 *
 * @param foc_handle
 */
__attribute__((__weak__))
void get_RadianAngle(struct foc* foc_handle)
{
    //TODO
    /* Like that ：

    foc_handle->thetaMECH = get_as5600Angle();
    foc_handle->thetaELEC = foc_handle->thetaMECH * foc_handle->motorPoles;

    */
}

/**
 * @brief Get the motor three phase current Iabc
 *
 * @param foc_handle pointer to foc component handler
 */
__attribute__((__weak__))
void get_current_Iabc(struct foc* foc_handle)
{
    //TODO
    /* Like that
    
    foc_handle->Iabc[0] = ADC_read_val(Ia);
    foc_handle->Iabc[1] = ADC_read_val(Ib);
    foc_handle->Iabc[2] = ADC_read_val(Ic);
    
    */
}

/**
 * @brief Get the motor three phase voltage Uabc
 *
 * @param foc_handle pointer to foc component handler
 */
__attribute__((__weak__))
void get_voltage_Uabc(struct foc* foc_handle)
{
    //TODO
    /* Like that
    
    foc_handle->Uabc[0] = ADC_read_val(Ua);
    foc_handle->Uabc[1] = ADC_read_val(Ub);
    foc_handle->Uabc[2] = ADC_read_val(Uc);
    
    */
}

/**
 * @brief Get the motor rotation Speed
 *
 * @param foc_handle pointer to foc component handler
 */
__attribute__((__weak__))
void get_motorSpeed(struct foc* foc_handle)
{
    //TODO
}

/**
 * @brief Initialize foc control handle
 *
 * @param foc_handle pointer to foc component handler
 */
void foc_init(struct foc* foc_handle)
{
    foc_handle->Udc = 12.0f;

    foc_handle->pwm_period = 6250;

    foc_handle->direction = DIR_CCW;

    foc_handle->motorFreq = 20;
    foc_handle->motorPoles = FOC_MOTOR_Poles;
    foc_handle->motorRs = FOC_MOTOR_Rs;
    foc_handle->motorLd = FOC_MOTOR_Ld;
    foc_handle->motorLq = FOC_MOTOR_Lq;

    foc_handle->Iq_ref = 0.0f;
    foc_handle->Id_ref = 0.0f;
    foc_handle->Speed_ref = 0.0f;

    foc_handle->Iq_Kp = FOC_Iq_Kp;
    foc_handle->Iq_Ki = FOC_Iq_Ki;

    foc_handle->Id_Kp = FOC_Id_Kp;
    foc_handle->Id_Ki = FOC_Id_Ki;

    foc_handle->Speed_Kp = FOC_Speed_Kp;
    foc_handle->Speed_Ki = FOC_Speed_Ki;

    foc_handle->IdErrorAll = 0.0f;
    foc_handle->IqErrorAll = 0.0f;
    foc_handle->SpeedErrorAll = 0.0f;

    foc_handle->outMax = 5.0f;
    foc_handle->outMin = -5.0f;

    foc_handle->Ud = 0.0f;
    foc_handle->Uq = 0.0f;

    foc_handle->Sector = 2;

    foc_handle->clarke = &clarke;
    foc_handle->iclarke = &iclarke;
    foc_handle->park   = &park;
    foc_handle->ipark  = &ipark;
    foc_handle->svpwm  = &svpwm;
    foc_handle->fast_svpwm  = &fast_svpwm;
    foc_handle->Id_PI  = &Id_PI;
    foc_handle->Iq_PI  = &Iq_PI;
    foc_handle->Speed_PID = &Speed_PID;
    foc_handle->svpwm_setDuty = &svpwm_setDuty;
    foc_handle->get_RadianAngle  = &get_RadianAngle;
    foc_handle->get_motorSpeed   = &get_motorSpeed;
    foc_handle->get_current_Iabc = &get_current_Iabc;
    foc_handle->get_voltage_Uabc = &get_voltage_Uabc;
}

/**
 * @brief Run motor foc control algorithm
 *
 * @param foc_handle pointer to foc component handler
 */
void foc_run(struct foc* foc_handle)
{
    /* temporary variable */

    /* get motor electrical angle */
    foc_handle->get_RadianAngle(foc_handle);

    /* get motor speed */
    foc_handle->get_motorSpeed(foc_handle);

    /* get motor current Iabc */
    foc_handle->get_current_Iabc(foc_handle);

    /* get motor voltage Uabc */
    foc_handle->get_voltage_Uabc(foc_handle);

    /* run clarke transform */
    foc_handle->clarke(foc_handle);

    /* run park transform */
    foc_handle->park(foc_handle);
    
    /* run Speed current loop PID regulation */
    foc_handle->Speed_PID(foc_handle);

    /* run current loop PID regulation */
    // foc_handle->Iq_PI(foc_handle);
    // foc_handle->Id_PI(foc_handle);

    /* run inverse-park transform */
    foc_handle->ipark(foc_handle);

    /* run svpwm */
    // foc_handle->svpwm(foc_handle);
    foc_handle->fast_svpwm(foc_handle);
}
