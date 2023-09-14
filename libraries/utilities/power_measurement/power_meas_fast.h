//#############################################################################
//
// FILE:     power_meas_fast.h
//
// TITLE:    Sine Analyzer with Power Measurement Module
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

#ifndef POWER_MEAS_FAST_H
#define POWER_MEAS_FAST_H

#ifdef __cplusplus

extern "C" 
{
#endif

//*****************************************************************************
//
//! \defgroup POWER_MEAS_FAST POWER_MEAS_FAST
//! @{
//
//*****************************************************************************

//
// Included Files
//
#include <stdint.h>
#ifndef __TMS320C28XX_CLA__
#include <math.h>
#else
#include <CLAmath.h>
#endif

//! \brief Defines the POWER_MEAS_SINE_ANALYZER structure
//!
//! \details The POWER_MEAS_SINE_ANALYZER can be used to analyze the
//!          input sine wave and calculates several parameters like
//!          RMS, Average and Frequency
//!
typedef volatile struct {
    float32_t v;           //!< Input: Voltage Sine Signal
    float32_t i;           //!< Input Current Signal

    float32_t sampleFreq;  //!< Input: Signal Sampling Freq
    float32_t threshold;   //!< Input: Voltage level corresponding to zero i/p
    float32_t vRms;        //!< Output: RMS Value
    float32_t iRms;        //!< Output: RMS Value of current
    float32_t acFreq;      //!< Output: Signal Freq

    float32_t vSqrSum;     //!< Internal : running sum for vacc square calculation over one sine cycle
    float32_t iSqrSum;     //!< Internal : running sum for Iacc_rms calculation over one sine cycle

    int32_t  nSamples;      //!< Internal: No of samples in one cycle of the sine wave
    int32_t  nSamplesMin;   //!< Internal: Lowerbound for no of samples in one sine wave cycle
    int32_t  nSamplesMax;   //!< Internal: Upperbound for no of samples in one sine wave cycle

    int16_t  prevSign;      //!< Internal: Flag to detect ZCD
    int16_t  currSign;      //!< Internal: Flag to detect ZCD
    int16_t  jitterCount;   //!< Internal: used to store jitter information due to noise on input
    int16_t  zcd;           //!< Output: Zero Cross detected
} POWER_MEAS_FAST;

//! \brief Resets internal data to zero
//! \param *v The POWER_MEAS_SINE_ANALYZER structure pointer
//! \return None
//!
static inline void POWER_MEAS_FAST_reset(POWER_MEAS_FAST *v)
{
    v->vRms = 0;
    v->iRms = 0;
    v->acFreq = 0;

    v->vSqrSum = 0;
    v->iSqrSum = 0;

    v->nSamples = 0;
    v->nSamplesMin = 0;
    v->nSamplesMax = 0;

    v->prevSign = 0;
    v->currSign = 0;
    v->jitterCount = 0;
    v->zcd = 0;
}

//! \brief Configures the power measurment module
//! \param *v The POWER_MEAS_SINE_ANALYZER structure pointer
//! \param isrFrequency Frequency at which SPLL module is run
//! \param threshold Threshold value to avoid zero crossing issues
//! \param gridMaxFreq Max grid frequency
//! \param gridMinFreq Min grid frequency
//! \return None
//!
static inline void POWER_MEAS_FAST_config(POWER_MEAS_FAST *v,
                                           float32_t isrFrequency,
                                           float32_t threshold,
                                           float32_t gridMaxFreq,
                                           float32_t gridMinFreq)
{
    v->sampleFreq = (float32_t)(isrFrequency);
    v->threshold = (float32_t)(threshold);

    v->nSamplesMax = (int32_t)(isrFrequency / gridMinFreq);
    v->nSamplesMin = (int32_t)(isrFrequency / gridMaxFreq);
}

//! \brief Perform calculations using the POWER_MEAS_SINE_ANALYZER module
//! \param *v The POWER_MEAS_SINE_ANALYZER structure pointer
//! \return None
static inline void POWER_MEAS_FAST_run(POWER_MEAS_FAST *v)
{
    v->currSign = ( v->v > v->threshold) ? 1 : 0;
    v->vSqrSum = v->vSqrSum + (v->v * v->v);
    v->iSqrSum = v->iSqrSum + (v->i * v->i);

    v->nSamples++;
    v->zcd = 0;

    if((v->prevSign != v->currSign) && (v->currSign == 1))
    {
        // check if the nSamples are in the ball park of a real frequency
        // that can be on the grid, this is done by comparing the nSamples
        // with the max value and min value it can be for the 
        // AC Grid Connection these Max and Min are initialized by the 
        // user in the code
        if(v->nSamplesMin < v->nSamples )
        {
            float32_t inverse_nSamples = (1.0f) / ((float32_t)v->nSamples);
            float32_t sqrt_inverse_nSamples = sqrtf(inverse_nSamples);

            v->vRms = sqrtf(v->vSqrSum) * sqrt_inverse_nSamples;
            v->iRms = sqrtf(v->iSqrSum) * sqrt_inverse_nSamples;
            v->acFreq = v->sampleFreq * inverse_nSamples;

            v->vSqrSum = 0;
            v->iSqrSum = 0;

            v->jitterCount = 0;
            v->nSamples = 0;
            v->zcd = 1;
        }
        else
        {
            //
            // otherwise it may be jitter ignore this reading
            // but count the number of jitters you are getting
            // but do not count to infinity as then when the grid comes back
            // it will take too much time to wind down the jitter count
            //
            if(v->jitterCount < 30)
            {
                v->jitterCount++;
            }

            v->nSamples = 0;
        }
    }

    if((v->nSamples > v->nSamplesMax) || (v->jitterCount > 20))
    {
        // most certainly the AC voltage is not present
        v->vRms = 0;
        v->iRms = 0;
        v->acFreq = 0;

        v->vSqrSum = 0;
        v->iSqrSum = 0;

        v->nSamples = 0;
        v->jitterCount = 0;
        v->zcd = 0;
    }

    v->prevSign = v->currSign;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

#ifdef __cplusplus
}
#endif                                  // extern "C"

#endif // end of POWER_MEAS_FAST_H definition

//
// End of File
//

