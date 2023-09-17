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

#ifndef DATALOG_H
#define DATALOG_H

//! \file   libraries/utilities/datalog/include/datalog.h
//! \brief  Contains the public interface to the
//!         data logging (DATALOG) module routines
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
//! \defgroup DATALOGIF DATALOGIF
//! @{
//
//*****************************************************************************

// the includes
#include <math.h>

// system
#include "stdbool.h"  // needed for bool type, true/false
#if !defined(__TMS320C28XX_CLA__)
#include "string.h"   // needed for size_t typedef
#endif
#include "stdint.h"   // needed for C99 data types

// **************************************************************************
// the defines
#if defined(DATALOGI4_EN) || defined(DATALOGF2_EN)

#include "libraries/math/include/math.h"

#if defined(DATALOGF2_EN)
#define DATA_LOG_BUFF_SIZE      200     // Must be intergral times of (20)
#define DATA_SIZE_EXT           2       //

#define DATA_LOG_BUFF_NUM       2       // = 2 for F28002x or = 4 for other devices

#define DLOG_BURST_SIZE         20       // write 200 to the register for
                                         // a burst size of 20

#define DLOG_TRANSFER_SIZE      (1 + 2* DATA_LOG_BUFF_SIZE / DLOG_BURST_SIZE)
                                // 200->21, 300->32

#elif defined(DATALOGI4_EN)
#define DATA_LOG_BUFF_SIZE      200      //**20
#define DATA_SIZE_EXT           20       //

#define DATA_LOG_BUFF_NUM       4       // = 2 for F28002x or = 4 for other devices

#define DLOG_BURST_SIZE         20       // write 200 to the register for
                                         // a burst size of 20

#define DLOG_TRANSFER_SIZE      10       // [(MEM_BUFFER_SIZE/(BURST)]
#endif  // DATALOGI4_EN

#define DLOG_WRAP_SIZE          DLOG_TRANSFER_SIZE + 1

#define DATA_LOG_SCALE_FACTOR   10        // update every 10 times

// **************************************************************************
// the typedefs

//! \brief Defines the data logging (DATALOG) object
//!
typedef struct _DATALOG_OBJ_
{
#if defined(DATALOGF2_EN)
    volatile float32_t  *iptr[DATA_LOG_BUFF_NUM];   //!< Input: First input pointer
#elif defined(DATALOGI4_EN)
    volatile int16_t  *iptr[DATA_LOG_BUFF_NUM];     //!< Input: First input pointer
#endif  // DATALOGI4_EN

    uint16_t  cntr;                             //!< Variable:  Data log counter
    uint16_t  size;                             //!< Parameter: Maximum data buffer
    uint16_t  scaleCnt;                         //!< Variable:  datalog skip counter
    uint16_t  scaleFactor;                      //!< Variable:  datalog prescale
    uint16_t  faultCount[DATA_LOG_BUFF_NUM];    //!< error data count
    int16_t   maxValue[DATA_LOG_BUFF_NUM];      //!< error data threshold maximum value
    int16_t   minValue[DATA_LOG_BUFF_NUM];      //!< error data threshold minimum value
    bool      flagEnable[DATA_LOG_BUFF_NUM];    //!< enable
} DATALOG_Obj;

//! \brief Defines the DATALOG handle
//!
typedef struct _DATALOG_Obj_   *DATALOG_Handle;

// **************************************************************************
// the globals
#if defined(DATALOGF2_EN)
extern float32_t datalogBuff1[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
extern float32_t datalogBuff2[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
#elif defined(DATALOGI4_EN)
extern int16_t datalogBuff1[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
extern int16_t datalogBuff2[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
extern int16_t datalogBuff3[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
extern int16_t datalogBuff4[DATA_LOG_BUFF_SIZE + DATA_SIZE_EXT];
#endif  // DATALOGI4_EN

//! \brief Defines the DATALOG object
//!
extern DATALOG_Obj datalog;
extern DATALOG_Handle datalogHandle;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the data logger
//! \param[in] ptr  The pointer to memory
extern DATALOG_Handle DATALOGIF_init(void *pMemory, const size_t numBytes);

//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
static inline bool DATALOGIF_enable(DATALOG_Handle handle)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;
    bool enableFlag = false;

    obj->scaleCnt++;

    if(obj->scaleCnt >= obj->scaleFactor)
    {
        enableFlag = true;
        obj->scaleCnt = 0;
    }

    return(enableFlag);
}


//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
static inline void DATALOGIF_setPrescalar(DATALOG_Handle handle, uint16_t prescalar)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

    obj->scaleFactor = prescalar;

    return;
}

//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
static inline void DATALOGIF_update(DATALOG_Handle handle)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

    obj->scaleCnt++;

    if(obj->scaleCnt >= obj->scaleFactor)
    {
        if(obj->cntr >= obj->size)
        {
            obj->cntr = 0;
        }

#if defined(DATALOGF2_EN)
        datalogBuff1[obj->cntr] = (*obj->iptr[0]);
        datalogBuff2[obj->cntr] = (*obj->iptr[1]);
#elif defined(DATALOGI4_EN)
        datalogBuff1[obj->cntr] = (*obj->iptr[0]);
        datalogBuff2[obj->cntr] = (*obj->iptr[1]);
        datalogBuff3[obj->cntr] = (*obj->iptr[2]);
        datalogBuff4[obj->cntr] = (*obj->iptr[3]);
#endif  // DATALOGI4_EN

        obj->cntr++;
        obj->scaleCnt = 0;
    }

    return;
}

//! \brief     Updates the data logger
//! \param[in] ptr  The pointer to memory
static inline void DATALOGIF_updateWithDMA(DATALOG_Handle handle)
{
    DATALOG_Obj *obj = (DATALOG_Obj *)handle;

    uint16_t number = obj->size - 1;

#if defined(DATALOGF2_EN)
    datalogBuff1[obj->size] = (*obj->iptr[0]);
    datalogBuff1[number]    = (*obj->iptr[0]);
    datalogBuff2[obj->size] = (*obj->iptr[1]);
    datalogBuff2[number]    = (*obj->iptr[1]);
#elif defined(DATALOGI4_EN)
    datalogBuff1[obj->size] = (*obj->iptr[0]);

    if((datalogBuff1[number] > obj->maxValue[0]) ||
            (datalogBuff1[number] < obj->minValue[0]))
    {
        obj->faultCount[0]++;
    }

    datalogBuff2[obj->size] = (*obj->iptr[1]);

    if((datalogBuff2[number] > obj->maxValue[1]) ||
            (datalogBuff2[number] < obj->minValue[1]))
    {
        obj->faultCount[1]++;
    }

    datalogBuff3[obj->size] = (*obj->iptr[2]);

    if((datalogBuff3[number] > obj->maxValue[2]) ||
            (datalogBuff3[number] < obj->minValue[2]))
    {
        obj->faultCount[2]++;
    }

    datalogBuff4[obj->size] = (*obj->iptr[3]);

    if((datalogBuff4[number] > obj->maxValue[3]) ||
            (datalogBuff4[number] < obj->minValue[3]))
    {
        obj->faultCount[3]++;
    }
#endif  // DATALOGI4_EN

    return;
}
#endif // DATALOGI4_EN || DATALOGF2_EN

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

#endif // end of DATALOG_H definition

