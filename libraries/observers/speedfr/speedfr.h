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

#ifndef SPEED_FR_H
#define SPEED_FR_H

//! \file   libraries\observers\speedfr\include\speedfr.h
//! \brief  Contains the public interface to the
//!         speed FRulation

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
//! \defgroup SPEED_FR SPEED_FR
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


typedef struct _SPDFR_obj_
{
    float32_t  theta;           // Input: reference set-point
    float32_t  thetaPrev;       // Input: feedback
    float32_t  K2;              // Parameter: proportional loop gain
    float32_t  K3;              // Parameter: integral gain
    float32_t  scaleFreq;       // Parameter:
    float32_t  speed_pu;        // Output: frequency
    float32_t  speed_Hz;        // Output: frequency
} SPDFR_Obj;

//! \brief Defines the SPDFR handle
//!
typedef struct _SPDFR_obj_ *SPDFR_Handle;

// ***************************************
// extern functions
// ***************************************
//! \brief     Set the SPDFR controller
//! \param[in] handle   The SPDFR controller handle
SPDFR_Handle SPDFR_init(void *pMemory, const size_t numBytes);

//! \brief     Set the SPDFR controller
//! \param[in] handle   The ESMO controller handle
void SPDFR_reset(SPDFR_Handle handle);

//! \brief     Set the SPDFR controller
//! \param[in] handle      The SPDFR controller handle
void SPDFR_setParams(SPDFR_Handle handle, const USER_Params *pUserParams);

//! \brief     Set the SPDFR controller
//! \param[in] handle      The SPDFR controller handle
static inline float32_t SPDFR_getSpeedHz(SPDFR_Handle handle)
{
    SPDFR_Obj *obj = (SPDFR_Obj *)handle;

    return(obj->speed_Hz);
}

//! \brief     Set the SPDFR controller
//! \param[in] handle   The SPDFR controller handle
static inline void SPDFR_run(SPDFR_Handle handle, float32_t theta)
{
    SPDFR_Obj *obj = (SPDFR_Obj *)handle;
    float32_t error;

    obj->theta  = theta * MATH_ONE_OVER_TWO_PI;

    // error cal
    error = obj->theta - obj->thetaPrev;

    if(error < -0.5f)
    {
        error += 1.0f;
    }
    else if(error > 0.5f)
    {
        error -= 1.0f;
    }

    // Update the electrical angle
    obj->thetaPrev = obj->theta;

    // Low-pass filter
    obj->speed_pu = (obj->K2 * obj->speed_pu) + (obj->K3 * error);

    // Saturate the output
    obj->speed_pu = __fsat(obj->speed_pu, 1.0f, -1.0f);

    // Change motor speed for pu to rpm value
    obj->speed_Hz = obj->scaleFreq * obj->speed_pu;

    return;
}

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

#endif //end of SPEED_FR_H definition
