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

//! \file   \libraries\dacs\dac128s085\include\dac128s085.h
//! \brief  Contains public interface to various functions related
//!         to the dacs128s085 object
//!

#ifndef DAC128S085_H
#define DAC128S085_H

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
//! \defgroup DAC128S DAC128S
//! @{
//
//*****************************************************************************

// the includes
#include <math.h>

//#include "libraries/math/include/math.h"

// drivers
#include "device.h"

//#include "libraries/math/include/math.h"

// **************************************************************************
// modules

// **************************************************************************
// solutions

// the defines

//! \brief Defines the address mask
//!
#define DAC128S_ADDR_MASK                   (0x7000)

//! \brief Defines the data mask
//!
#define DAC128S_DATA_MASK                   (0x0FFF)

//! \brief Defines the R/W mask
//!
#define DAC128S_MODE_MASK                   (0xF000)

//! \brief Defines the maximum DAC channel
//!
#define DAC128S_CH_NUM_MAXIMUM              (4U)    // One~Eight channels

//! \brief Defines the enabled DAC channel
//!
#define DAC128S_CH_NUM_ENABLE               (4U)    // One~Eight channels

#if DAC128S_CH_NUM_ENABLE > DAC128S_CH_NUM_MAXIMUM
#error The enabled channels must be less than or equal to the maximum channel
#endif

// **************************************************************************
// the typedefs

//------------------------------------------------------------------------------
//! \brief Enumeration for the R/W modes
//!
typedef enum
{
    DAC128S_MODE_WRM   = (8 << 12),  //!< Write
    DAC128S_MODE_WTM   = (9 << 12),  //!< Write data to channel
    DAC128S_MODE_WRD   = (0 << 12)   //!< write data mode
} DAC128S_writeMode_e;

//! \brief Enumeration for the register addresses
//!
typedef enum
{
    DAC128S_CH_A  = (0 << 12),   //!< Channel A
    DAC128S_CH_B  = (1 << 12),   //!< Channel B
    DAC128S_CH_C  = (2 << 12),   //!< Channel C
    DAC128S_CH_D  = (3 << 12),   //!< Channel D
    DAC128S_CH_E  = (4 << 12),   //!< Channel E
    DAC128S_CH_F  = (5 << 12),   //!< Channel F
    DAC128S_CH_G  = (6 << 12),   //!< Channel G
    DAC128S_CH_H  = (7 << 12),   //!< Channel H
} DAC128S_Channel_e;


//! \brief Object for the DAC128S registers and commands
//!
typedef struct _DAC128S_Obj_
{
    volatile float32_t *ptrData[DAC128S_CH_NUM_MAXIMUM];    //!< Input: First input pointer

    float32_t  gain[DAC128S_CH_NUM_MAXIMUM];       //!< the DAC data
    uint32_t   spiHandle;                          //!< handle for the serial peripheral interface

    int16_t    offset[DAC128S_CH_NUM_MAXIMUM];     //!< the DAC data
    int16_t    dacData[DAC128S_CH_NUM_ENABLE];
    uint16_t   enableChNum;
}DAC128S_Obj;


//! \brief Defines the DAC128S handle
//!
typedef struct _DAC128S_Obj_ *DAC128S_Handle;

//! \brief Defines the DAC128S Word type
//!
typedef  uint16_t    DAC_Word_t;

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes

//! \brief     Initializes the DAC128S object
//! \param[in] pMemory   A pointer to the memory for the DAC128S object
//! \param[in] numBytes  The number of bytes allocated for the DAC128S
//!                      object, bytes
//! \return    The DAC128S object handle
extern DAC128S_Handle DAC128S_init(void *pMemory);

//! \brief     Builds the control word
//! \param[in] ctrlMode  The control mode
//! \param[in] regName   The register name
//! \param[in] data      The data
//! \return    The control word
static inline DAC_Word_t DAC128S_buildCtrlWord(
        const DAC128S_writeMode_e writeMode, const uint16_t channel, const uint16_t data)
{
    DAC_Word_t ctrlWord = writeMode | (channel<<12) | (data & DAC128S_DATA_MASK);

    return(ctrlWord);
} // end of DAC128S_buildCtrlWord() function


//! \brief     Enables the DAC128S
//! \param[in] handle     The DAC128S handle
extern void DAC128S_enable(DAC128S_Handle handle);

//! \brief     setup the SPI for the DAC128S
//! \param[in] handle     The DAC128S handle
extern void DAC128S_setupSPI(DAC128S_Handle handle);


//! \brief     setup the SPI for the DAC128S
//! \param[in] handle     The DAC128S handle
//! \param[in] bitRate    The baud rate for SPI
void DAC128S_setupSPIBR(DAC128S_Handle handle, uint32_t bitRate);


//! \brief     Write command to the DAC128S SPI registers
//! \param[in] handle     The DAC128S handle
extern void DAC128S_writeCommand(DAC128S_Handle handle);


//! \brief     Write data to the DAC128S SPI registers
//! \param[in] handle     The DAC128S handle
extern void DAC128S_writeData(DAC128S_Handle handle);

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
#endif // extern "C"

#endif // end of DAC128S085_H definition
