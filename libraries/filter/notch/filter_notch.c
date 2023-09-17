//#############################################################################
//
// FILE:   filter_notch.c
//
// TITLE:  second-order notch filter
//
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

#include "filter_notch.h"


//*****************************************************************************
//
// FILTER_NOTCH_init
//
//*****************************************************************************
FILTER_NOTCH_Handle
FILTER_NOTCH_init(void *pMemory, const size_t numBytes)
{
    FILTER_NOTCH_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(FILTER_NOTCH_Obj))
    {
        return((FILTER_NOTCH_Handle)NULL);
    }

    handle = (FILTER_NOTCH_Handle)pMemory;

    return(handle);
} // end of FILTER_NOTCH_init() function


//*****************************************************************************
//
// FILTER_NOTCH_init
//
//*****************************************************************************
FILTER_NOTCH_Coeff_Handle
FILTER_COEFF_init(void *pMemory, const size_t numBytes)
{
    FILTER_NOTCH_Coeff_Handle handle;

    if((int16_t)numBytes < (int16_t)sizeof(FILTER_NOTCH_CoeffObj))
    {
        return((FILTER_NOTCH_Coeff_Handle)NULL);
    }

    handle = (FILTER_NOTCH_Coeff_Handle)pMemory;

    return(handle);
} // end of FILTER_COEFF_init() function

//*****************************************************************************
//
// FILTER_COEFF_update
//
//*****************************************************************************
void
FILTER_NOTCH_reset(FILTER_NOTCH_Handle handle)
{
    FILTER_NOTCH_Obj *obj = (FILTER_NOTCH_Obj *)handle;

    // reset the Notch filer
    obj->in = 0.0f;
    obj->in1 = 0.0f;
    obj->in2 = 0.0f;
    obj->out = 0.0f;
    obj->out1 = 0.0f;
    obj->out2 = 0.0f;

    return;
} // end of FILTER_COEFF_update() function

//*****************************************************************************
//
// FILTER_COEFF_update
//
//*****************************************************************************
void
FILTER_COEFF_update(FILTER_NOTCH_Coeff_Handle handle, const float32_t Ts,
              const float32_t freqGrid, const float32_t c2, const float32_t c1)
{
    FILTER_NOTCH_CoeffObj *obj = (FILTER_NOTCH_CoeffObj *)handle;

    // Note c2<<c1 for the notch to work
    float32_t wn = 2.0f * MATH_TWO_PI * freqGrid;

    float32_t x = 2.0f * c2 * wn * Ts;
    float32_t y = 2.0f * c1 * wn * Ts;
    float32_t z = wn * Ts * wn * Ts;

    obj->b0 = (1.0f);
    obj->b1 = (x - 2.0f);
    obj->b2 = (z - x + 1.0f);
    obj->a1 = -(y - 2.0f);
    obj->a2 = -(z - y + 1.0f);

    return;
} // end of FILTER_COEFF_update() function

// end of file
