//#############################################################################
// $Copyright:
// Copyright (C) 2017-2023 Texas Instruments Incorporated - http://www.ti.com/
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//! \file   \libraries\observers\esmo\include\esmo.h
//! \brief  Contains the public interface to the 
//!         enhanced smo estimator
//!

#ifndef _ESMO_H_
#define _ESMO_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
//! \defgroup ESMO ESMO
//! @{
//
//*****************************************************************************


// the includes
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/src/float/CLAmath.h"
#else
#include <math.h>
#endif

#include "libraries/math/include/math.h"

//modules
#include "userParams.h"

// **************************************************************************
// the defines


// the typedefs

//! \brief Defines the ESMO controller object
//!
typedef struct _ESMO_Obj_
{
    float32_t scaleFreq_Hz;   //

    float32_t speed_sf;     // speed PU scale factor
    float32_t voltage_sf;   // voltage PU scale factor
    float32_t current_sf;   // current PU scale factor

    float32_t Ts;           // sampling period in sec
    float32_t base_wTs;     // base_wTs = BASE_FREQ * T ,for lib
    float32_t filterFc_sf;  // smo filter frequency coefficient

    float32_t Fdsmopos;     // motor dependent plant matrix
    float32_t Fqsmopos;     // motor dependent plant matrix
    float32_t Gdsmopos;     // motor dependent control gain
    float32_t Gqsmopos;     // motor dependent control gain
    float32_t Kslf;         // sliding control filter gain
    float32_t E0;           // estimated bemf threshold

    float32_t Kslide;       // sliding control gain
    float32_t KslideMax;    // sliding control gain, maximum value
    float32_t KslideMin;    // sliding control gain, minimum value

	float32_t Valpha;       // stationary d-axis phase voltage (pu)
	float32_t Vbeta;        // stationary q-axis phase voltage (pu)

    float32_t EstIalpha;    // estimated stationary alfa-axis stator current
    float32_t EstIbeta;     // estimated stationary beta-axis stator current

	float32_t Ealpha;       // stationary alfa-axis back EMF
	float32_t Ebeta;        // stationary beta-axis back EMF

	float32_t Zalpha;       // stationary alfa-axis sliding control
	float32_t Zbeta;        // stationary beta-axis sliding control

    float32_t Ed;
    float32_t Eq;
    float32_t Eq_mag;

    float32_t thetaOffset_rad;
    float32_t thetaElec_rad;

	float32_t thetaErr;
	float32_t thetaErrSF;
	float32_t theta;
	float32_t thetaPll;
	float32_t thetaDelta;
    float32_t offsetSF;     // Scale factor
    float32_t thetaEst;

    float32_t speedRef;
    float32_t speedEst;
    float32_t speedFlt;

	float32_t pll_Out;          // controller output
	float32_t pll_Umax;         // upper saturation limit
	float32_t pll_Umin;         // lower saturation limit
	float32_t pll_ui;           // integral term

	float32_t pll_Kp;           // proportional gain
	float32_t pll_KpMax;        // maximum proportional gain
	float32_t pll_KpMin;        // minimum proportional loop gain
	float32_t pll_KpSF;         // proportional gain coefficient
	float32_t pll_Ki;           // integral gain

	float32_t lpf_b0;           // Low Pass Filter Param b0
	float32_t lpf_a1;           // Low Pass Filter Param a1

	float32_t lpfFc_Hz;         // Low Pass Filter desired cut off frequency (Hz)
} ESMO_Obj;


//! \brief Defines the ESMO handle
//!
typedef struct _ESMO_Obj_ *ESMO_Handle;

// **************************************************************************
// the function prototypes
//! \brief     Initializes the ESMO controller
//! \param[in] pMemory   A pointer to the memory for the ESMO controller object
//! \param[in] numBytes  The number of bytes allocated for the ESMO controller object, bytes
//! \return The ESMO controller (ESMO) object handle
extern ESMO_Handle ESMO_init(void *pMemory, const size_t numBytes);


//! \brief     Reset the ESMO controller
//! \param[in] handle      The ESMO controller handle
extern void ESMO_resetParams(ESMO_Handle handle);


//! \brief     Set the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] pUserParams  The pointer to the user parameters
extern void ESMO_setParams(ESMO_Handle handle, const USER_Params *pUserParams);


//! \brief     Set PLL parameters for the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] pll_KpMax   The maximum Kp of PLL
//! \param[in] pll_KpMin   The minimum Kp of PLL
//! \param[in] pll_KpSF    The Kp calculation coefficient of PLL
extern void ESMO_setPLLParams(ESMO_Handle handle,
                              const float32_t pll_KpMax,
                              const float32_t pll_KpMin,
                              const float32_t pll_KpSF);


//! \brief     Set PLL parameters for the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] pll_Ki      The Ki of PLL
extern void ESMO_setPLLKi(ESMO_Handle handle, const float32_t pll_Ki);


//! \brief     Set kp of the pll for the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] pll_Kp    The Kp value
static inline void ESMO_setBEMFThreshold(ESMO_Handle handle, const float32_t bemfThreshold)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->E0 = bemfThreshold;

    return;
}

//! \brief     Set kp of the pll for the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] pll_Kp    The Kp calculation coefficient value
static inline void ESMO_setPLLKpSF(ESMO_Handle handle, const float32_t pll_KpSF)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_KpSF = pll_KpSF;

    return;
}

//! \brief     Set PLL parameters for the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] pll_KpMax   The maximum Kp of PLL
//! \param[in] pll_KpMin   The maximum Kp of PLL
extern void ESMO_setKslideParams(ESMO_Handle handle,
                              const float32_t KslideMax,
                              const float32_t KslideMin);


//! \brief     Sets Kslide for the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] Kslide    The Kslide value
static inline void ESMO_setKslide(ESMO_Handle handle, const float32_t Kslide)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->Kslide = Kslide;

    return;
}

//! \brief     Reset the ESMO controller
//! \param[in] handle      The ESMO controller handle
extern void  ESMO_updateFilterParams(ESMO_Handle handle);


//! \brief     Reset the ESMO controller
//! \param[in] handle      The ESMO controller handle
extern void ESMO_updatePLLParams    (ESMO_Handle handle);

//! \brief     Runs the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] Vdcbus      The dc bus voltage
//! \param[in] pVabc_pu    The pointer to the phase voltage value (PU)
//! \param[in] pIabVec     The pointer to the phase current value
extern void ESMO_full_run(ESMO_Handle handle,
                     float32_t Vdcbus, MATH_vec3 *pVabc_pu, MATH_vec2 *pIabVec);


//! \brief     Resets PLL integration for the ESMO controller
//! \param[in] handle    The ESMO controller handle
static inline void ESMO_resetPLL(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_ui = 0.0f;
    obj->pll_Out = 0.0f;

    return;
}

//! \brief     Sets angle offset coefficient for the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] offsetSF    The angle offset coefficient
static inline void ESMO_setOffsetCoef(ESMO_Handle handle, const float32_t offsetSF)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->offsetSF = offsetSF;

    return;
}

//! \brief     Sets speed filter cut off frequency for the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] filterFc_Hz    The slide filter frequency
static inline void ESMO_setBEMFKslfFreq(ESMO_Handle handle, const float32_t filterFc_sf)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->filterFc_sf = filterFc_sf;

    return;
}

//! \brief     Sets speed filter cut off frequency for the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] lpfFc_Hz  The cut off frequency of the speed filter
static inline void ESMO_setSpeedFilterFreq(ESMO_Handle handle, const float32_t lpfFc_Hz)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->lpfFc_Hz = lpfFc_Hz;

    return;
}

//! \brief     Runs the ESMO controller
//! \param[in] handle    The ESMO controller handle
//! \param[in] pll_Kp    The Kp value
static inline void ESMO_setPLLKp(ESMO_Handle handle, const float32_t pll_Kp)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->pll_Kp = pll_Kp;

    return;
}

//! \brief     Set Angle to the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] theta_rad   The estimation angle, rad
static inline void ESMO_setAnglePu(ESMO_Handle handle, const float32_t theta_rad)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->theta = theta_rad * MATH_ONE_OVER_TWO_PI;

    return;
}

//! \brief     Set Angle to the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] speed_Hz    The feedback speed value, Hz
static inline void ESMO_setPLLSpeedPu(ESMO_Handle handle, const float32_t speed_Hz)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->speedFlt = speed_Hz * obj->speed_sf;
    obj->pll_Out = speed_Hz * obj->speed_sf;

    return;
}

//! \brief     Set reference speed to the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] refValue    The reference speed value
static inline void ESMO_setSpeedRef(ESMO_Handle handle, const float32_t speedRef_Hz)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    obj->speedRef = speedRef_Hz * obj->speed_sf;

    return;
}

//! \brief     Gets the speed from the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \return    The angle from eSMO estimator (Hz)
static inline float32_t ESMO_getSpeed_Hz(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->speedEst * obj->scaleFreq_Hz);
}


//! \brief     Gets the PLL speed from the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \return    The speed from eSMO PLL (Hz)
static inline float32_t ESMO_getSpeedPLL_Hz(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->speedFlt * obj->scaleFreq_Hz);
}

//! \brief     Gets the PLL angle from the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \return    The angle from eSMO PLL
static inline float32_t ESMO_getAnglePLL(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->thetaEst);
}

//! \brief     Gets the angle from the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \return    The angle from eSMO estimator
static inline float32_t ESMO_getAngleElec(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    return(obj->thetaElec_rad);
}

//! \brief     Update Kslide for the ESMO controller
//! \param[in] handle      The ESMO controller handle
static inline void ESMO_updateKslide(ESMO_Handle handle)
{
    ESMO_Obj *obj = (ESMO_Obj *)handle;

    if(obj->Kslide < obj->KslideMax)
    {
        obj->Kslide += 0.000002f;
    }

    return;
}

//! \brief     Runs the ESMO controller
//! \param[in] handle      The ESMO controller handle
//! \param[in] Vdcbus      The dc bus voltage
//! \param[in] pVabc_pu    The pointer to the phase voltage value (PU)
//! \param[in] pIabVec     The pointer to the phase current value
#define ESMO_inline_run ESMO_run        // for compatible with old version

static inline void ESMO_run(ESMO_Handle handle,
                      float32_t Vdcbus, MATH_vec3 *pVabc_pu, MATH_vec2 *pIabVec)
{
	ESMO_Obj *obj = (ESMO_Obj *)handle;

    // Scale the incomming modulation functions with the DC bus voltage value
    // and calculate the 3 Phase voltages
    float32_t Vtemp   = Vdcbus * obj->voltage_sf;

    float32_t VphaseA = Vtemp *
            (pVabc_pu->value[0] * 2.0f - pVabc_pu->value[1] - pVabc_pu->value[2]);

    float32_t VphaseB = Vtemp *
            (pVabc_pu->value[1] * 2.0f - pVabc_pu->value[0] - pVabc_pu->value[2]);

    // Voltage transformation (a,b,c)  ->  (Alpha,Beta)
    obj->Valpha = VphaseA;
    obj->Vbeta = (VphaseA + VphaseB * 2.0f) * MATH_ONE_OVER_SQRT_THREE;

    // Sliding mode current observer
    float32_t ValphaError = obj->Valpha - obj->Ealpha - obj->Zalpha;
    float32_t VbetaError  = obj->Vbeta - obj->Ebeta - obj->Zbeta;

    obj->EstIalpha = obj->Gdsmopos * ValphaError + obj->Fdsmopos * obj->EstIalpha;
    obj->EstIbeta  = obj->Gqsmopos * VbetaError  + obj->Fqsmopos * obj->EstIbeta;

    //  Current errors
    float32_t IalphaError = obj->EstIalpha - pIabVec->value[0] * obj->current_sf;
    float32_t IbetaError  = obj->EstIbeta  - pIabVec->value[1] * obj->current_sf;

    // Sliding control calculator
    obj->Zalpha = __fsat(IalphaError, obj->E0, -obj->E0) * obj->Kslide;
    obj->Zbeta  = __fsat(IbetaError,  obj->E0, -obj->E0) * obj->Kslide;

    //  Sliding control filter -> back EMF calculator
    obj->Ealpha = obj->Ealpha + obj->Kslf * (obj->Zalpha - obj->Ealpha);
    obj->Ebeta  = obj->Ebeta  + obj->Kslf * (obj->Zbeta  - obj->Ebeta);

    // arc tangent of src radians
    float32_t thetaOffset = __atan2puf32((obj->speedRef * obj->offsetSF), obj->Kslf);
    obj->thetaPll  = obj->theta - thetaOffset;

    float32_t pllSine   = __sinpuf32(obj->thetaPll);
    float32_t pllCosine = __cospuf32(obj->thetaPll);

    obj->Ed = obj->Ealpha * pllCosine + obj->Ebeta  * pllSine;
    obj->Eq = obj->Ebeta  * pllCosine - obj->Ealpha * pllSine;
    obj->Eq_mag = sqrtf(obj->Ealpha * obj->Ealpha + obj->Ebeta * obj->Ebeta);

    //0.1591549431 = 1/6.28 (1/(2*PI())
    float32_t thetaErrSF = obj->thetaErrSF;

    if(obj->Eq >= 0.0f)
    {
        thetaErrSF = -obj->thetaErrSF;
    }

    obj->thetaErr = obj->Ed * thetaErrSF / obj->Eq_mag;

    // integral term
    obj->pll_ui = (obj->pll_Ki * obj->thetaErr) + obj->pll_ui;

    // control output
    obj->pll_Out = __fsat((obj->pll_Kp * obj->thetaErr + obj->pll_ui),
                          obj->pll_Umax, obj->pll_Umin);

    obj->speedEst = (obj->pll_Out + obj->speedEst) * 0.5f;

    // low pass filter for estimation speed
    obj->speedFlt = obj->lpf_b0 * obj->pll_Out + obj->lpf_a1 * obj->speedFlt;

    // speed integration to get the rotor angle
    obj->theta = obj->theta + obj->speedFlt * obj->thetaDelta;

    if(obj->theta > 1.0f)
    {
        obj->theta -= 1.0f;
    }
    else if(obj->theta < -1.0f)
    {
        obj->theta += 1.0f;
    }

    obj->thetaEst = obj->theta * MATH_TWO_PI;

    if(obj->thetaEst > MATH_PI)
    {
        obj->thetaEst -= MATH_TWO_PI;
    }
    else if(obj->thetaEst < (-MATH_PI))
    {
        obj->thetaEst += MATH_TWO_PI;
    }

	return;
} // end of ESMO_run() function


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //end of _ESMO_H_ definition

