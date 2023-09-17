//#############################################################################
//
// FILE:   filter_fo.h
//
// TITLE:  second-order notch filter
//         module filters out the input signal with a notch filter, thus
//         removing a particular frequency from the input signal.
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

#ifndef FILTER_NOTCH_H
#define FILTER_NOTCH_H

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

// the includes
#ifdef __TMS320C28XX_CLA__
#include "libraries/math/src/float/CLAmath.h"
#else
#include <math.h>
#endif

#include "libraries/math/include/math.h"

//*****************************************************************************
//
//! \defgroup FILTER_NOTCH FILTER_NOTCH
//!
//! @{
//
//*****************************************************************************

#ifdef __cplusplus
extern "C" {
#endif

//*****************************************************************************
//
//! \brief Defines the notch filter coefficient (FILTER_COEFF) object
//
//*****************************************************************************
typedef struct _FILTER_NOTCH_CoeffObj_
{
    float32_t b2;        //!< the numerator filter coefficient value for z^(-2)
    float32_t b1;        //!< the numerator filter coefficient value for z^(-1)
    float32_t b0;        //!< the numerator filter coefficient value for z^0
    float32_t a2;        //!< the denominator filter coefficient value for z^(-2)
    float32_t a1;        //!< the denominator filter coefficient value for z^(-1)
} FILTER_NOTCH_CoeffObj;

//*****************************************************************************
//
//! \brief Defines the notch filter coefficient (FILTER_NOTCH_Coeff) handle
//
//*****************************************************************************
typedef struct _FILTER_NOTCH_CoeffObj_ *FILTER_NOTCH_Coeff_Handle;


//*****************************************************************************
//
//! \brief Defines the notch filter (FILTER_NOTCH) object
//
//*****************************************************************************
typedef struct _FILTER_NOTCH_Obj_
{
    float32_t out1;       //!< the output value at time sample n=-1
    float32_t out2;       //!< the output value at time sample n=-2
    float32_t in;         //!< the input value
    float32_t in1;        //!< the input value at time sample n=-1
    float32_t in2;        //!< the input value at time sample n=-2
    float32_t out;        //!< the output value
} FILTER_NOTCH_Obj;

//*****************************************************************************
//
//! \brief Defines the notch filter (FILTER_NOTCH) handle
//
//*****************************************************************************
typedef struct _FILTER_NOTCH_Obj_ *FILTER_NOTCH_Handle;

//*****************************************************************************

//*****************************************************************************
//
//! \brief     Gets the notch filter input value at time sample n-1
//!
//! \param[in] handle  The notch filter handle
//!
//! \return    The input value at time sample n-1
//
//*****************************************************************************
static inline float32_t
FILTER_NOTCH_get_in1(FILTER_NOTCH_Handle handle)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    return(obj->in1);
} // end of FILTER_NOTCH_get_in1() function


//*****************************************************************************
//
//! \brief     Gets the notch filter input value at time sample n-2
//!
//! \param[in] handle  The notch filter handle
//!
//! \return    The input value at time sample n-2
//
//*****************************************************************************
static inline float32_t
FILTER_NOTCH_get_in2(FILTER_NOTCH_Handle handle)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    return(obj->in2);
} // end of FILTER_NOTCH_get_in2() function


//*****************************************************************************
//
//! \brief     Gets the nothc filter output value at time sample n
//!
//! \param[in] handle  The notch filter handle
//!
//! \return    The output value at time sample n
//
//*****************************************************************************
static inline float32_t
FILTER_NOTCH_get_out(FILTER_NOTCH_Handle handle)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    return(obj->out);
} // end of FILTER_NOTCH_get_out() function


//*****************************************************************************
//
//! \brief     Gets the nothc filter output value at time sample n-1
//!
//! \param[in] handle  The notch filter handle
//!
//! \return    The output value at time sample n-1
//
//*****************************************************************************
static inline float32_t
FILTER_NOTCH_get_out1(FILTER_NOTCH_Handle handle)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    return(obj->out1);
} // end of FILTER_NOTCH_get_out1() function


//*****************************************************************************
//
//! \brief     Gets the nothc filter output value at time sample n-2
//!
//! \param[in] handle  The notch filter handle
//!
//! \return    The output value at time sample n-2
//
//*****************************************************************************
static inline float32_t
FILTER_NOTCH_get_out2(FILTER_NOTCH_Handle handle)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    return(obj->out2);
} // end of FILTER_NOTCH_get_out2() function

//*****************************************************************************
//
//! \brief     Initializes the notch filter coefficient handle
//!
//! \param[in] pMemory   A pointer to the memory for the notch filter coefficient
//!                      object
//!
//! \param[in] numBytes  The number of bytes allocated for the notch
//!                      filter coefficient object, bytes
//!
//! \return    The notch filter (FILTER_COEFF) object handle
//
//*****************************************************************************
extern FILTER_NOTCH_Coeff_Handle
FILTER_COEFF_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Initializes the notch filter
//!
//! \param[in] pMemory   A pointer to the memory for the notch filter
//!                      object
//!
//! \param[in] numBytes  The number of bytes allocated for the notch
//!                      filter object, bytes
//!
//! \return    The notch filter (FILTER_NOTCH) object handle
//
//*****************************************************************************
extern FILTER_NOTCH_Handle
FILTER_NOTCH_init(void *pMemory, const size_t numBytes);

//*****************************************************************************
//
//! \brief     Sets the notch filter input value at time sample n
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] in      The input value at time sample n
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_NOTCH_set_in(FILTER_NOTCH_Handle handle, const float32_t in)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    obj->in = in;

    return;
} // end of FILTER_NOTCH_set_in() function


//*****************************************************************************
//
//! \brief     Sets the notch filter input value at time sample n-1
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] in1      The input value at time sample n-1
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_NOTCH_set_in1(FILTER_NOTCH_Handle handle, const float32_t in1)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    obj->in1 = in1;

    return;
} // end of FILTER_NOTCH_set_in1() function

//*****************************************************************************
//
//! \brief     Sets the notch filter input value at time sample n-2
//!
//! \param[in] handle  The filter handle
//!
//! \param[in] in2      The input value at time sample n-2
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_NOTCH_set_in2(FILTER_NOTCH_Handle handle, const float32_t in2)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    obj->in2 = in2;

    return;
} // end of FILTER_NOTCH_set_in2() function

//*****************************************************************************
//
//! \brief     Sets the notch filter output value at time sample n
//!
//! \param[in] handle  The notch filter handle
//!
//! \param[in] out      The output value at time sample n
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_NOTCH_set_out(FILTER_NOTCH_Handle handle, const float32_t out)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    obj->out = out;

    return;
} // end of FILTER_NOTCH_set_out() function

//*****************************************************************************
//
//! \brief     Sets the notch filter output value at time sample n-1
//!
//! \param[in] handle  The notch filter handle
//!
//! \param[in] out1    The output value at time sample n-1
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_NOTCH_set_out1(FILTER_NOTCH_Handle handle, const float32_t out1)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    obj->out1 = out1;

    return;
} // end of FILTER_NOTCH_set_out1() function


//*****************************************************************************
//
//! \brief     Sets the notch filter output value at time sample n-2
//!
//! \param[in] handle  The notch filter handle
//!
//! \param[in] out2    The output value at time sample n-2
//!
//! \return    None
//
//*****************************************************************************
static inline void
FILTER_NOTCH_set_out2(FILTER_NOTCH_Handle handle, const float32_t out2)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    obj->out2 = out2;

    return;
} // end of FILTER_NOTCH_set_out2() function

//*****************************************************************************
//
//! \brief     Runs a notch filter of the form
//!            y[n] = a1*y[n-1] + a2*y[n-2] + b0*x[n] + b1*x[n-1] + b2*x[n-2]
//!
//! \param[in] notchHandle  The filter notch handle
//!
//! \param[in] coeffHandle  The filter coefficient handle
//!
//! \return    The output value from the filter
//
//*****************************************************************************
static inline float32_t
FILTER_NOTCH_run(FILTER_NOTCH_Handle notchHandle, FILTER_NOTCH_Coeff_Handle coeffHandle)
{
    FILTER_NOTCH_Obj *objFilter = (FILTER_NOTCH_Obj *)notchHandle;
    FILTER_NOTCH_CoeffObj *objCoeff  = (FILTER_NOTCH_CoeffObj *)coeffHandle;

    //
    // Compute the output
    //
    objFilter->out = objCoeff->a1 * objFilter->out1 +
                     objCoeff->a2 * objFilter->out2 +
                     objCoeff->b0 * objFilter->in   +
                     objCoeff->b1 * objFilter->in1  +
                     objCoeff->b2 * objFilter->in2;

    //
    // Store values for next time
    //
    objFilter->out2 = objFilter->out1;
    objFilter->out1 = objFilter->out;

    objFilter->in2 = objFilter->in1;
    objFilter->in1 = objFilter->in;

    return(objFilter->out);
} // end of FILTER_NOTCH_run() function


//*****************************************************************************
//
//! \brief     Resets the  notch filter
//!
//! \param[in] handle  The notch filter handle
//!
//! \return    None
//
//*****************************************************************************
extern void FILTER_NOTCH_reset(FILTER_NOTCH_Handle handle);

//*****************************************************************************
//
//! \brief     Updates the notch filter numerator coefficients
//!
//! \param[in] handle   The notch filter coefficient handle
//!
//! \param[in] Ts       The notch filter sampling period, sec
//!
//! \param[in] freqGrid The Nominal AC grid frequency for notch filter, Hz
//!
//! \param[in] c2       The notch filter parameter
//!
//! \param[in] c1       The notch filter parameter
//!
//! \return    None
//
//*****************************************************************************
extern void
FILTER_COEFF_update(FILTER_NOTCH_Coeff_Handle handle, const float32_t Ts,
              const float32_t freqGrid, const float32_t c2, const float32_t c1);

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

#endif // FILTER_NOTCH_H
