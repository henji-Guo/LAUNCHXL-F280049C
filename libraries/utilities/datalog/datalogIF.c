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

//! \file   libraries/utilities/datalog/source/datalog.c
//! \brief  Portable C fixed point code.  These functions define the
//!         data logging (DATALOG) module routines
//!

// **************************************************************************
// the includes
#include "datalogIF.h"

// **************************************************************************
// the defines

// **************************************************************************
// the globals
#if defined(DATALOGI4_EN) || defined(DATALOGF2_EN)
#if defined(DATALOGF2_EN)
DATALOG_Obj datalog;
DATALOG_Handle datalogHandle;       //!< the handle for the Datalog object
#pragma DATA_SECTION(datalog, "datalog_data");
#pragma DATA_SECTION(datalogHandle, "datalog_data");

float32_t datalogBuff1[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
float32_t datalogBuff2[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
#pragma DATA_SECTION(datalogBuff1, "datalog_data");
#pragma DATA_SECTION(datalogBuff2, "datalog_data");

#elif defined(DATALOGI4_EN)
DATALOG_Obj datalog;
DATALOG_Handle datalogHandle;                   //!< the handle for the Datalog object

int16_t datalogBuff1[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
int16_t datalogBuff2[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
int16_t datalogBuff3[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
int16_t datalogBuff4[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];

#pragma DATA_SECTION(datalogBuff1, "datalog_data");
#pragma DATA_SECTION(datalogBuff2, "datalog_data");
#pragma DATA_SECTION(datalogBuff3, "datalog_data");
#pragma DATA_SECTION(datalogBuff4, "datalog_data");
#endif  // DATALOGI4_EN || DATALOGF2_EN

// **************************************************************************
// the functions

DATALOG_Handle DATALOGIF_init(void *pMemory, const size_t numBytes)
{
    DATALOG_Handle handle;

    if(numBytes < sizeof(DATALOG_Obj))
    {
        return((DATALOG_Handle)NULL);
    }

    // assign the handle
    handle = (DATALOG_Handle)pMemory;

    DATALOG_Obj *obj = (DATALOG_Obj *)handle;
    uint16_t cnt;

    obj->cntr = 0;
    obj->size = DATA_LOG_BUFF_SIZE;

    obj->scaleCnt = 0;
    obj->scaleFactor = DATA_LOG_SCALE_FACTOR;

    for(cnt = 0; cnt < DATA_LOG_BUFF_NUM; cnt++)
    {
        obj->faultCount[cnt] = 0;

        obj->maxValue[cnt] = 2048 + 50;
        obj->minValue[cnt] = 2048 - 50;

        obj->flagEnable[cnt] = false;
    }

    obj->maxValue[0] = 50;
    obj->minValue[0] = 0;

    return(handle);
} // end of DATALOG_init() function

#endif  // DATALOGI4_EN || DATALOGF2_EN

// end of file
