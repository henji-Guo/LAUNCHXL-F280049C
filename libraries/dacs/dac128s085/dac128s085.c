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

//! \file   \libraries\dacs\dac128s085\source\dac128s085.c
//! \brief  Contains the various functions related to the dacs128s085 object
//!

// **************************************************************************
// the includes
#include "dac128s085.h"

// **************************************************************************
// drivers



// **************************************************************************
// modules

// **************************************************************************
// platforms

// **************************************************************************
// the defines

// **************************************************************************
// the globals

// **************************************************************************
// the function prototypes


DAC128S_Handle DAC128S_init(void *pMemory)
{
    DAC128S_Handle handle;
    DAC128S_Obj *obj;

    // assign the handle
    handle = (DAC128S_Handle)pMemory;
    obj = (DAC128S_Obj *)handle;

#if defined(DAC128S_SPIA)
    // assign the SPI handle
    obj->spiHandle = SPIA_BASE;
#elif defined(DAC128S_SPIB)
    // assign the SPI handle
    obj->spiHandle = SPIB_BASE;
#else
    // assign the SPI handle
    obj->spiHandle = SPIA_BASE;
#endif

    obj->enableChNum = DAC128S_CH_NUM_ENABLE;

    return(handle);
} // end of DAC128S_init() function


void DAC128S_setupSPI(DAC128S_Handle handle)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;

    // Must put SPI into reset before configuring it
    SPI_disableModule(obj->spiHandle);

#if defined(DAC_FASTUPDATE)
    // SPI configuration. Use a 500KHz/1MHz/2MHz SPICLK and 16-bit word size, 25/30MHz LSPCLK
    SPI_setConfig(obj->spiHandle, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER, 5000000, 16);       // 5.0MHz
#else   // !DAC_FASTUPDATE
    // SPI configuration. Use a 500KHz/1MHz/2MHz SPICLK and 16-bit word size, 25/30MHz LSPCLK
    SPI_setConfig(obj->spiHandle, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER, 2000000, 16);       // 2.0MHz
#endif  // !DAC_FASTUPDATE

    SPI_disableLoopback(obj->spiHandle);

    SPI_setEmulationMode(obj->spiHandle, SPI_EMULATION_FREE_RUN);

    SPI_enableFIFO(obj->spiHandle);

#if (DAC128S_CH_NUM_ENABLE == 1)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX1, SPI_FIFO_RX1);
#elif (DAC128S_CH_NUM_ENABLE == 2)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX2, SPI_FIFO_RX2);
#elif (DAC128S_CH_NUM_ENABLE == 3)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX3, SPI_FIFO_RX3);
#elif (DAC128S_CH_NUM_ENABLE == 4)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX4, SPI_FIFO_RX4);
#elif (DAC128S_CH_NUM_ENABLE == 5)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX5, SPI_FIFO_RX5);
#elif (DAC128S_CH_NUM_ENABLE == 6)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX6, SPI_FIFO_RX6);
#elif (DAC128S_CH_NUM_ENABLE == 7)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX7, SPI_FIFO_RX7);
#elif (DAC128S_CH_NUM_ENABLE == 8)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX8, SPI_FIFO_RX8);
#else
#error A wrong enabled channels!!
#endif

    SPI_setTxFifoTransmitDelay(obj->spiHandle, 0x02);   // Low delay

    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    // Configuration complete. Enable the module.
    SPI_enableModule(obj->spiHandle);

    return;
}  // end of DAC128S_setupSPI() function


void DAC128S_setupSPIBR(DAC128S_Handle handle, uint32_t bitRate)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;

    // Must put SPI into reset before configuring it
    SPI_disableModule(obj->spiHandle);

    // SPI configuration. Use a 500KHz/1MHz/2MHz SPICLK and 16-bit word size, 25/30MHz LSPCLK
    SPI_setConfig(obj->spiHandle, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_CONTROLLER, bitRate, 16);       // 500kMHz~2.0MHz

    SPI_disableLoopback(obj->spiHandle);

    SPI_setEmulationMode(obj->spiHandle, SPI_EMULATION_FREE_RUN);

    SPI_enableFIFO(obj->spiHandle);

#if (DAC128S_CH_NUM_ENABLE == 1)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX1, SPI_FIFO_RX1);
#elif (DAC128S_CH_NUM_ENABLE == 2)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX2, SPI_FIFO_RX2);
#elif (DAC128S_CH_NUM_ENABLE == 3)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX3, SPI_FIFO_RX3);
#elif (DAC128S_CH_NUM_ENABLE == 4)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX4, SPI_FIFO_RX4);
#elif (DAC128S_CH_NUM_ENABLE == 5)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX5, SPI_FIFO_RX5);
#elif (DAC128S_CH_NUM_ENABLE == 6)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX6, SPI_FIFO_RX6);
#elif (DAC128S_CH_NUM_ENABLE == 7)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX7, SPI_FIFO_RX7);
#elif (DAC128S_CH_NUM_ENABLE == 8)
    SPI_setFIFOInterruptLevel(obj->spiHandle, SPI_FIFO_TX8, SPI_FIFO_RX8);
#else
#error A wrong enabled channels!!
#endif

    SPI_setTxFifoTransmitDelay(obj->spiHandle, 0x02);

    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    // Configuration complete. Enable the module.
    SPI_enableModule(obj->spiHandle);

    return;
}  // end of DAC128S_setupSPIBR() function

void DAC128S_writeCommand(DAC128S_Handle handle)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;
    uint16_t ctrlWord;

    // reset the Rx & Tx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_resetTxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);
    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    // build the control word
    ctrlWord = (uint16_t)DAC128S_buildCtrlWord(DAC128S_MODE_WTM, 0, 0);

    // write the command
    SPI_writeDataNonBlocking(obj->spiHandle, ctrlWord);

    // Delay 20us
    DEVICE_DELAY_US(20U);

    return;
}  // end of DAC128S_writeData() function


void DAC128S_writeData(DAC128S_Handle handle)
{
    DAC128S_Obj *obj = (DAC128S_Obj *)handle;
    uint16_t cnt;
    uint16_t ctrlWord;

    float32_t dacData;

    // reset the Rx & Tx fifo pointer to zero
    SPI_resetRxFIFO(obj->spiHandle);
    SPI_resetTxFIFO(obj->spiHandle);
    SPI_enableFIFO(obj->spiHandle);
    SPI_clearInterruptStatus(obj->spiHandle, SPI_INT_TXFF);

    for(cnt = 0; cnt < obj->enableChNum; cnt++)
    {
        dacData = (*obj->ptrData[cnt]);
        obj->dacData[cnt] = (int16_t)(dacData * obj->gain[cnt]) + obj->offset[cnt];

        // build the control word
        ctrlWord = (uint16_t)DAC128S_buildCtrlWord(DAC128S_MODE_WRD, cnt, obj->dacData[cnt]);

        // write the command
        SPI_writeDataNonBlocking(obj->spiHandle, ctrlWord);
    }

    return;
}  // end of DAC128S_writeData() function


// end of file
