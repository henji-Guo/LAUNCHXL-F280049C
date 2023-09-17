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

//******************************************************************************
//
// FILE:   \libraries\control\vib_comp\source\vib_comp.c
//
// TITLE:  define the Vibration Compensation (VIB_COMP) module routines
//
//******************************************************************************


// **************************************************************************
// the includes

#include "vib_comp.h"

// **************************************************************************
// the defines


// **************************************************************************
// the globals


// **************************************************************************
// the functions

VIB_COMP_Handle VIB_COMP_init(void *pMemory, const size_t numBytes)
{
    VIB_COMP_Handle vib_compHandle;


    if(numBytes < sizeof(VIB_COMP_Obj))
    {
        return((VIB_COMP_Handle)NULL);
    }

    // assign the handle
    vib_compHandle = (VIB_COMP_Handle)pMemory;

    return(vib_compHandle);
} // end of VIB_COMP_init() function

void VIB_COMP_reset(VIB_COMP_Handle handle)
{
    VIB_COMP_Obj *obj = (VIB_COMP_Obj *)handle;
    int16_t cnt;

    obj->flagEnableFF = false;

    obj->index = 0;

    for(cnt = 0; cnt < (int16_t)VIB_COMP_BUF_SIZE; cnt++)
    {
        obj->FF_table[cnt] = 0.0f;
    }

    return;
} // end of VIB_COMP_reset() function


void VIB_COMPA_setParams(VIB_COMP_Handle handle,
                        const float32_t alpha, const float32_t gain,
                        const int16_t indexDelta, const uint16_t numPolePairs)
{
    VIB_COMP_Obj *obj = (VIB_COMP_Obj *)handle;

    obj->alpha = alpha * gain;
    obj->beta = (1.0f - alpha) * gain;

    obj->anglePairsMax = ((float32_t)numPolePairs) * MATH_TWO_PI;
    obj->angleMechInvSf = 1.0f / ((float32_t)numPolePairs);
    obj->indexSf = (float32_t)VIB_COMP_BUF_SIZE / MATH_TWO_PI;

    obj->angleElecPrev_rad = 0.0f;
    obj->angleMechPoles_rad = 0.0f;

    obj->indexDelta = indexDelta;

    return;
} // end of VIB_COMPA_setParams() function

void VIB_COMPT_setParams(VIB_COMP_Handle handle, const uint16_t numPolePairs)
{
    VIB_COMP_Obj *obj = (VIB_COMP_Obj *)handle;

    obj->anglePairsMax = ((float32_t)numPolePairs) * MATH_TWO_PI;
    obj->angleMechInvSf = 1.0f / ((float32_t)numPolePairs);

    obj->angleElecPrev_rad = 0.0f;
    obj->angleMechPoles_rad = 0.0f;

    return;
} // end of VIB_COMPT_setParams() function

// end of file
