/*
 * pwm_example_test.c
 *
 *  Created on: 2023年7月5日
 *      Author: GHJ
 */
#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"

static void epwm3_gpio_mux_init(void)
{
    // GPIO4 => EPWM3A
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    // GPIO5 => EPWM3B
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
}

void pwm_base_test(void)
{
    // Must disable the clock to the ePWM modules to have all ePWM modules synchronized
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /*
     * EPWM3 Channel A 50KHz
     */
    // EPWM3 gpio mux
    epwm3_gpio_mux_init();
    // reset EPWM3
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM3);
    // stop EPWM3
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);

    // set EPWM3_CLK prescaler and highSpeedPrescaler    PWM_CLK = SYSCLK / (prescaler * highSpeedPrescaler)
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_1,EPWM_HSCLOCK_DIVIDER_1); //100Mhz
    // set EPWM3 cycle 50KHz , period counter = 100MHz/50KHz = 2000
    EPWM_setTimeBasePeriod(EPWM3_BASE,1999);
    // clear EPWM3 initial value
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    // set EPWM3 Period Load Mode
    EPWM_setPeriodLoadMode(EPWM3_BASE, EPWM_PERIOD_SHADOW_LOAD);
    // set EPWM SYNC
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
    // set EPWM Phase Shift
    EPWM_setPhaseShift(EPWM3_BASE, 0);
    // disable Phase Shift Load
    EPWM_disablePhaseShiftLoad(EPWM3_BASE);

    // EPWM3 CMP_A : Load when counter equals zero or period
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 1799);
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,EPWM_COUNTER_COMPARE_B, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_B, 99);

    // EPWM3 Action Qualifier
    EPWM_setActionQualifierShadowLoadMode(EPWM3_BASE, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_disableActionQualifierShadowLoadMode(EPWM3_BASE, EPWM_ACTION_QUALIFIER_A);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);

    EPWM_setEmulationMode(EPWM3_BASE, EPWM_EMULATION_STOP_AFTER_NEXT_TB);                         // Ignore emulation suspend

    // enable EPWM3 Time Base
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP);

    //---------------------------------------------------------------------
    //--- Enable the clocks to the ePWM module.
    //--- Note: this should be done after all ePWM modules are configured
    //--- to ensure synchronization between the ePWM modules.
    //---------------------------------------------------------------------
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // TBCLK to ePWM modules enabled

}
