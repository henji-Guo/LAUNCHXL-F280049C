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

#ifndef SSIPD_H
#define SSIPS_H

//! \file   libraries\observers\ssipd\include\ssipd.h
//! \brief  Contains the public interface to the
//!         speed calculation
//!


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
//! \defgroup SSIPD SSIPD
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


#define SSIPD_DEBUG     1

#define SSIPD_DETECT_NUM      24.0f
#define SSIPD_BUFF_NUM        (uint16_t)(SSIPD_DETECT_NUM + 2)

//*****************************************************************************
//
//! \brief Defines the SSIPD_Obj object
//
//*****************************************************************************
typedef struct _SSIPD_obj_
{
    float32_t  VdSet_V;         // Output: Injection voltage
    float32_t  VdInject_V;      // Output: Injection voltage
    float32_t  IsTemp_A;        //
    float32_t  IsPeak_A;        //
    float32_t  angleTemp_rad;    // Output: detection command angle
    float32_t  angleCmd_rad;    // Output: detection command angle
    float32_t  angleOut_rad;    // Output: detection output angle
    float32_t  angleInc_rad;    // Output: detection delta angle
    float32_t  angleMax_rad;    // Output: detection maximum angle

    uint16_t   pulseWidth;      //
    uint16_t   pulseCount;      //
    bool       flagDirection;   //
    bool       flagEnablePWM;   //
    bool       flagDoneStatus;  //
    bool       flagRunState;    //
} SSIPD_Obj;

//*****************************************************************************
//
//! \brief Defines the SSIPD handle
//
//*****************************************************************************
typedef struct _SSIPD_obj_ *SSIPD_Handle;

#ifdef SSIPD_DEBUG
extern float32_t IsPeakBuff[SSIPD_BUFF_NUM];
extern float32_t AngleBuff[SSIPD_BUFF_NUM];
extern uint16_t peakBuffcnt;
#endif // SSIPD_DEBUG

//*****************************************************************************
//
// Prototypes for the APIs
//
//*****************************************************************************
//! \brief     Initializes the SSIPD object
//! \param[in] *pMemory         Pointer in to the SSIPD object
//! \param[in] numBytes         Size of the object
//! \return    The SSIPD object handle
extern SSIPD_Handle SSIPD_init(void *pMemory, const size_t numBytes);

//! \brief     Sets the SSIPD parameters
//! \param[in] handle  The SSIPD handle
//! \param[in] value  The injection voltage
//! \param[in] value  The angle delta
//! \return    None
extern void SSIPD_setParams(SSIPD_Handle handle, const float32_t volSet_V,
                            const float32_t angleInc_rad, const uint16_t pulseWidth);


//! \brief     Gets the enable PWM flag
//! \param[in] handle      The SSIPD handle
//! \return    the flag enable PWM
static inline bool SSIPD_getFlagEnablePWM(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    return(obj->flagEnablePWM);
}

//! \brief     Gets the SSIPD state
//! \param[in] handle      The SSIPD handle
//! \return    SSIPD running state
static inline bool SSIPD_getRunState(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    return(obj->flagRunState);
}

//! \brief     Gets the SSIPD state
//! \param[in] handle      The SSIPD handle
//! \return    SSIPD state
static inline float32_t SSIPD_getDoneStatus(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    return(obj->flagDoneStatus);
}

//! \brief     Gets the SSIPD state
//! \param[in] handle      The SSIPD handle
//! \return    Angle output
static inline float32_t SSIPD_getAngleOut_rad(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    return(obj->angleOut_rad);
}

//! \brief     Gets the SSIPD state
//! \param[in] handle      The SSIPD handle
//! \return    Angle detection value
static inline float32_t SSIPD_getAngleCmd_rad(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    return(obj->angleCmd_rad);
}

//! \brief     Gets the SSIPD state
//! \param[in] handle      The SSIPD handle
//! \return    Injection voltage
static inline float32_t SSIPD_getVolInject_V(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    return(obj->VdInject_V);
}

//! \brief     Runs six-pulse initial position detection
//! \param[in] handle   The SSIPD handle
//! \param[in] pIab     The pointer to the input vector
//! \return    None
static inline void SSIPD_start(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    obj->flagRunState = true;
    obj->flagDirection = false;

    obj->pulseCount = 0;
    obj->angleCmd_rad = 0.0f;
    obj->angleTemp_rad = 0.0f;
    obj->angleOut_rad = 0.0f;

    obj->IsPeak_A = 0.0f;

#ifdef SSIPD_DEBUG
    peakBuffcnt = 0;
#endif // SSIPD_DEBUG

}

//! \brief     Runs six-pulse initial position detection
//! \param[in] handle   The SSIPD handle
//! \param[in] pIab     The pointer to the input vector
//! \return    None
static inline void SSIPD_reset(SSIPD_Handle handle)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    obj->flagDoneStatus = false;
    obj->flagEnablePWM = false;
    obj->flagRunState = false;
    obj->flagDirection = false;

    obj->pulseCount = 0;

    obj->angleCmd_rad = 0.0f;
    obj->angleTemp_rad = 0.0f;
    obj->angleOut_rad = 0.0f;

    obj->IsPeak_A = 0.0f;

#ifdef SSIPD_DEBUG
    peakBuffcnt = 0;
#endif // SSIPD_DEBUG
}

//! \brief     Runs six-pulse initial position detection
//! \param[in] handle   The SSIPD handle
//! \param[in] pIab     The pointer to the input vector
//! \return    None
static inline void SSIPD_inine_run(SSIPD_Handle handle, MATH_Vec2 *pIab)
{
    SSIPD_Obj *obj = (SSIPD_Obj *)handle;

    obj->pulseCount++;

    if(obj->pulseCount >= (obj->pulseWidth<<4))
    {
        obj->pulseCount = 0;

        if(obj->flagDirection == false)
        {
            obj->angleTemp_rad += obj->angleInc_rad;
            obj->angleCmd_rad = obj->angleTemp_rad;

            obj->flagDirection = true;
        }
        else
        {
            obj->angleCmd_rad = obj->angleTemp_rad + MATH_PI;

            obj->flagDirection = false;
        }

#ifdef SSIPD_DEBUG
        IsPeakBuff[peakBuffcnt] = obj->IsPeak_A;
        AngleBuff[peakBuffcnt] = obj->angleCmd_rad;
        peakBuffcnt++;
        if(peakBuffcnt >= SSIPD_BUFF_NUM)
        {
            peakBuffcnt = 0;
        }
#endif // SSIPD_DEBUG
    }
    else if(obj->pulseCount <= obj->pulseWidth)
    {
        obj->flagEnablePWM = true;

        obj->VdInject_V = obj->VdSet_V;

        obj->IsTemp_A = pIab->value[0] * pIab->value[0] + pIab->value[1] * pIab->value[1];

        if(obj->IsTemp_A > obj->IsPeak_A)
        {
            obj->IsPeak_A = obj->IsTemp_A;
            obj->angleOut_rad = obj->angleCmd_rad;
        }
    }
    else
    {
        obj->VdInject_V = 0.0f;
        obj->flagEnablePWM = false;
    }

    if(obj->angleTemp_rad > obj->angleMax_rad)
    {
        obj->VdInject_V = 0.0f;
        obj->IsTemp_A = 0.0f;

        obj->flagEnablePWM = false;
        obj->flagDoneStatus = true;
        obj->flagRunState = false;
    }

    return;
}   // end of SSIPD_run() function

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

#endif //end of SSIPD_H definition
