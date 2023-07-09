/*
 * pga_example_test.c
 *
 *  Created on: 2023年7月2日
 *      Author: GHJ
 */
#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"

static struct {
    uint16_t dacVal;
    uint16_t adcVal;
    PGA_GainValue pgaGain;
    uint16_t pgaVal;
}pga_test_value={0x10,0,PGA_GAIN_3,0};

static void analog_vref_init(void)
{
    // reset
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_DACB);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCC);
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_PGA4);
    // set voltage reference internal 3.3v
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL,ADC_REFERENCE_3_3V);
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL,ADC_REFERENCE_3_3V);
    // set DACB gain 3.3V=1.65V*2
    DAC_setReferenceVoltage(DACB_BASE,DAC_REF_ADC_VREFHI);
    DAC_setGainMode(DACB_BASE, DAC_GAIN_TWO);
    DEVICE_DELAY_US(1000);
}

static void dacb_init(void){
    // disable DACB
    DAC_disableOutput(DACB_BASE);
    // set DAC load trigger source
    DAC_setLoadMode(DACB_BASE,DAC_LOAD_SYSCLK);
    // set DAC output
    DAC_setShadowValue(DACB_BASE,0x10);
    // enable DACB
    DAC_enableOutput(DACB_BASE);
    DEVICE_DELAY_US(500);
}

__interrupt static void adcc1_isr_handler(void)
{
    // read ADCCIN3
    pga_test_value.adcVal = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
    // set DAC value
    DAC_setShadowValue(DACB_BASE,pga_test_value.dacVal);
    // clear interrupt flag
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
    // clear ack
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
    // force soc1 to work
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER1);

}

__interrupt static void adcc3_isr_handler(void)
{
    // read ADCCIN9
    pga_test_value.pgaVal = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
    // clear interrupt flag
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER3);
    // clear ACK
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP10);
    // set PGA4 Gain
    PGA_setGain(PGA4_BASE, pga_test_value.pgaGain);
    // force soc0 to work
    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);

}
static void adcc_init(void)
{
    // disable ADCC
    ADC_disableConverter(ADCC_BASE);
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);
    // ADCC,SOC0 ,Software trigger,ADCCIN3,sample 8 sysclk
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN3, 8);
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN9, 8);
    // EOC Interrupt
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);
    // no ADC interrupt trigger soc
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);
    // set soc interrupt ADCC_INT1
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, 0);
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER3, 1);
    // enable ADCC_INT1
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER3);
    // PIE table register
    Interrupt_register(INT_ADCC1, &adcc1_isr_handler);
    Interrupt_register(INT_ADCC3, &adcc3_isr_handler);
    Interrupt_enable(INT_ADCC1);
    Interrupt_enable(INT_ADCC3);
    // enable
    ADC_enableConverter(ADCC_BASE);
    DEVICE_DELAY_US(500);
}

static void pga4_init(void)
{
    // disable
    PGA_disable(PGA4_BASE);
    // set PGA gain x3
    PGA_setGain(PGA4_BASE, PGA_GAIN_3);
    // note that the PGA_OUT pin is an internal signal
    // PGA4_OUT = ADCB_IN11 or ADCC_IN9

    // enable PGA4
    PGA_enable(PGA4_BASE);
    DEVICE_DELAY_US(500);
}

void pga_test(void)
{
    analog_vref_init();
    dacb_init();
    adcc_init();
    pga4_init();

    ADC_forceSOC(ADCC_BASE, ADC_SOC_NUMBER0);
}
