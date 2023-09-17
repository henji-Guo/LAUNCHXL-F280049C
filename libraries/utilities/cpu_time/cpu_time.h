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

#ifndef CPU_TIME_H
#define CPU_TIME_H

//! \file   libraries/utilities/cpu_time/include/cpu_time.h
//! \brief  Contains the public interface to the 
//!         CPU_time (CPU_time) module routines
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
//! \defgroup CPU_TIME CPU_TIME
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

// system
#include <stdbool.h>  // needed for bool type, true/false
#if !defined(__TMS320C28XX_CLA__)
#include <string.h>   // needed for size_t typedef
#endif
#include <stdint.h>   // needed for C99 data types


// **************************************************************************
// the defines
#define MOD_INDEX_MAX      4

#define TIME_MOD_CHK0      0
#define TIME_MOD_CHK1      1
#define TIME_MOD_CHK2      2
#define TIME_MOD_CHK3      3

// **************************************************************************
// the typedefs

//! \brief Defines the CPU usage (CPU_TIME) object
//!
typedef struct _CPU_TIME_Obj_
{
  float32_t cpuWidthRatio;
  uint32_t  timerCntNow;
  uint32_t  timerCntPrev[MOD_INDEX_MAX];

  uint16_t  ctrlPeriodCnt;
  uint16_t  deltaNow[MOD_INDEX_MAX];    // the latest delta count value, cnts
  uint16_t  deltaMin[MOD_INDEX_MAX];    // the minimum delta counts, cnts
  uint16_t  deltaMax[MOD_INDEX_MAX];    // the maximum delta counts, cnts

  bool      flag_resetStatus;           // a flag to reset all measured data
} CPU_TIME_Obj;


//! \brief Defines the CPU_TIME handle
//!
typedef struct _CPU_TIME_Obj_ *CPU_TIME_Handle;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes

//! \brief     Initializes the CPU TIME (CPU_TIME) object
//! \param[in] pMemory    A pointer to the base address of the object
//! \param[in] numBytes   The object size, bytes
//! \return    The handle to the object
extern CPU_TIME_Handle CPU_TIME_init(void *pMemory,const size_t numBytes);


//! \brief     Resets timer
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
extern void CPU_TIME_reset(CPU_TIME_Handle handle);

//! \brief     Sets the state of the reset stats flag
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
static inline void CPU_TIME_setResetFlag(CPU_TIME_Handle handle)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->flag_resetStatus = true;

  return;
} // end of CPU_TIME_setResetFlag() function

//! \brief     Runs the CPU TIME module
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
//! \param[in] ctrlPeriod  The control interrupt period
static inline void
CPU_TIME_setCtrlPeriod(CPU_TIME_Handle handle, uint16_t ctrlPeriod)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->ctrlPeriodCnt = ctrlPeriod * 2;

  return;
} // end of CPU_TIME_setCtrlPeriod() function


//! \brief     Updates the current and previous count values
//! \param[in] handle    The CPU TIME (CPU_TIME) handle
static inline void
CPU_TIME_calcCPUWidthRatio(CPU_TIME_Handle handle)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  float32_t ctrlPeriod = (float32_t)obj->ctrlPeriodCnt;

  obj->cpuWidthRatio = ((float32_t)obj->deltaMax[0]) / ctrlPeriod;

  return;
} // end of CPU_TIME_calcCPUWidthRatio() function


//! \brief     Updates the current and previous count values
//! \param[in] handle    The CPU TIME (CPU_TIME) handle
//! \param[in] timerCnt  The current count value
//! \param[in] modIndex  The current index
static inline void
CPU_TIME_updateCnts(CPU_TIME_Handle handle, const uint32_t cnt, uint16_t modIndex)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->timerCntPrev[modIndex] = cnt;

  if(obj->flag_resetStatus == true)
  {
	  obj->flag_resetStatus = false;

      uint16_t cnt;

      for(cnt = 0; cnt < MOD_INDEX_MAX; cnt++)
      {
          obj->deltaMin[cnt] = 0xFFFF;
          obj->deltaMax[cnt] = 0;
      }
  }

  return;
} // end of CPU_TIME_updateCnts() function


//! \brief     Runs the CPU TIME module
//! \param[in] handle  The CPU TIME (CPU_TIME) handle
//! \param[in] modIndex  The current index
static inline void
CPU_TIME_run(CPU_TIME_Handle handle, uint32_t count, uint16_t modIndex)
{
  CPU_TIME_Obj *obj = (CPU_TIME_Obj *)handle;

  obj->timerCntNow = count;

  if(modIndex >= MOD_INDEX_MAX)
  {
      return;
  }

  obj->deltaNow[modIndex] =
                    (uint16_t)(obj->timerCntPrev[modIndex] - obj->timerCntNow);

  if(obj->deltaNow[modIndex] > obj->deltaMax[modIndex])
      obj->deltaMax[modIndex] = obj->deltaNow[modIndex];

  if(obj->deltaNow[modIndex] < obj->deltaMin[modIndex])
      obj->deltaMin[modIndex] = obj->deltaNow[modIndex];

  return;
} // end of CPU_TIME_run() function

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

#endif // end of CPU_TIME_H definition


