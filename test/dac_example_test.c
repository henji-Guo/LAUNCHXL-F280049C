/*
 * dac_example_test.c
 *
 *  Created on: 2023年7月1日
 *      Author: GHJ
 *
 *  DAC test: the pin DACB_OUP is same with ADCA_IN1,
 *  and this analog pin do not need pin mux operation.
 *  So can use ADCA directly gather DACB values to check.
 */
#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"

__attribute__((__unused__))
static uint16_t dac_to_adc_value;
// interrupt routine
static __interrupt void adca_isr_handler(void)
{
    // Get ADC result
    dac_to_adc_value = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    // clear ADCA_INT1 flag
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    // clear PIE ADC ACK
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

static void adca_init(void){
    //disable
    ADC_disableConverter(ADCA_BASE);
    // set Prescaler , max ADC_CLK 50MHz = 100MHz/2
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);
    // set ADC SOC
    // select ADCA , use SW trigger , use pin ADCINA1 , use SOC0 , sampling window 8 sysclk
    // note that sampling window do not low than 75ns
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN1, 8);
    // set soc priority
    ADC_setSOCPriority(ADCA_BASE, ADC_PRI_ALL_ROUND_ROBIN);
    //enable
    ADC_enableConverter(ADCA_BASE);
    // note that at least need wait 1ms , make sure that adc can works
    DEVICE_DELAY_US(1000);
    // set interrupt
    // Occurs at the end of the conversion
    ADC_setInterruptPulseMode(ADCA_BASE,ADC_PULSE_END_OF_CONV);
    // when SOC0 EOC trigger ADC_INT1
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    // enable ADCA_INT1
    ADC_enableInterrupt(ADCA_BASE,ADC_INT_NUMBER1);
    // register ADCA1_INT
    Interrupt_register(INT_ADCA1, &adca_isr_handler);
    // enable interrupt
    Interrupt_enable(INT_ADCA1);

    // start sample
    ADC_forceSOC(ADCA_BASE,ADC_SOC_NUMBER0);
}


void dac_test(void)
{
    // disable DACB
    DAC_disableOutput(DACB_BASE);
    //set analog voltage reference
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);
    // if DAC VREF 3.3V , Gain 1.65*2
    DAC_setGainMode(DACB_BASE, DAC_GAIN_TWO);
    // set sync load trigger source
    DAC_setLoadMode(DACB_BASE, DAC_LOAD_SYSCLK);
    // enable DAC
    DAC_enableOutput(DACB_BASE);
    // wait anlog system stable
    DEVICE_DELAY_US(1000);

    adca_init();

    uint64_t i;
    for ( i = 0; i < UINT64_MAX; i++) {
        // set output value
        DAC_setShadowValue(DACB_BASE, i%4096);
        DEVICE_DELAY_US(10);
        // restart sample
        ADC_forceSOC(ADCA_BASE,ADC_SOC_NUMBER0);
        DEVICE_DELAY_US(1000);
    }
}


