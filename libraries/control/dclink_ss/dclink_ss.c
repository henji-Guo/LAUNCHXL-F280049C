//#############################################################################
//
// FILE:   dclink_ss.c
//
// TITLE:  C28x DC-Link Single-Shunt Current Reconstruction library
//         (floating point)
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

#include "dclink_ss.h"

// **************************************************************************
// the defines

// **************************************************************************
// the globals


// **************************************************************************
// the functions

//*****************************************************************************
//
// DCLINK_SS_init
//
//*****************************************************************************
DCLINK_SS_Handle
DCLINK_SS_init(void *pMemory,const size_t numBytes)
{
    DCLINK_SS_Handle dclinkHandle;

    if((int16_t)numBytes < (int16_t)sizeof(DCLINK_SS_Obj))
    {
        return((DCLINK_SS_Handle)NULL);
    }

    //
    // Assign the handle
    //
    dclinkHandle = (DCLINK_SS_Handle)pMemory;

    return(dclinkHandle);
} // end of DCLINK_SS_init() function

//*****************************************************************************
//
// DCLINK_SS_setInitialConditions
//
//*****************************************************************************
void
DCLINK_SS_setInitialConditions(DCLINK_SS_Handle handle,
                               const uint16_t pwmPeriod,
                               const float32_t SSTOffThrVs_pu)
{
    DCLINK_SS_Obj *obj = (DCLINK_SS_Obj *)handle;

    obj->minAvDuration = 90;
    obj->sampleDelay = 50;
    obj->sector_1 = 0;
    obj->sector = 0;

    obj->I_A.value[0] = 0.0;
    obj->I_A.value[1] = 0.0;
    obj->I_A.value[2] = 0.0;

    obj->SSTOffThrVs_pu = SSTOffThrVs_pu;

    obj->pwmPeriod = pwmPeriod;

    obj->vecArea = 0;
    obj->vecArea_1 = 0;

    obj->flag_SST = 0;
    obj->flag_SST_1 = 0;

    //disable full sampling by default
    obj->flagEnableFullSample = false;

    //enable sequence control by default
    obj->flagEnableSequenceControl = true;

    obj->flagRunInHighModulation = false;

    return;
} // end of DCLINK_SS_setInitialConditions() function
//
// end of file
//
