/*
 * adc_example_test.c
 *
 *  Created on: 2023年7月1日
 *      Author: GHJ
 */
#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"

uint16_t adca_result = 0;
static __interrupt void adca_isr(void);

// adc software trigger sample
void adc_sw_test(void)
{
    //Note that ADC PIN do not need pin mux init

    // not necessary
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCA);
    //disable
    ADC_disableConverter(ADCA_BASE);
    // set analog voltage reference
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);
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
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, 0);
    // enable ADCA_INT1
    ADC_enableInterrupt(ADCA_BASE,ADC_INT_NUMBER1);
    // register ADCA1_INT
//    PieVectTable.ADCA1_INT = &adca_isr;
    Interrupt_register(INT_ADCA1, &adca_isr);
    // enable interrupt
    Interrupt_enable(INT_ADCA1);

    // start sample
    ADC_forceSOC(ADCA_BASE,ADC_SOC_NUMBER0);
}

// interrupt routine
__interrupt void adca_isr(void)
{
    // Get ADC result
    adca_result = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    // clear ADCA_INT1 flag
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    // clear PIE ADC ACK
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    // restart sample
    ADC_forceSOC(ADCA_BASE,ADC_SOC_NUMBER0);
}
