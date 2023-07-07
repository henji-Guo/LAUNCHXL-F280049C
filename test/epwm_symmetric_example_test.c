/*
 * epwm_symmetric_example_test.c
 *
 *  Created on: 2023年7月5日
 *      Author: GHJ
 */

#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"

static struct {
    uint16_t debug_on;
    uint16_t test_period;
    uint16_t test_cmpa;
}epwm_symmetric_test_val={0,2500,1250};

static void epwm3_gpio_mux_init(void)
{
    // GPIO4 => EPWM3A
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(4, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(4, GPIO_CORE_CPU1);
    // GPIO5 => EPWM3B
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(5, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(5, GPIO_CORE_CPU1);
}

/*
 * use EPWM3_A and B to generate symmetric wave
 */
void epwm_symmetric_test(void)
{
    // Must disable the clock to the ePWM modules to have all ePWM modules synchronized
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /*
     * EPWM3 Channel AB: 10KHz, 50% duty, AB complementary
     */
    // EPWM3 gpio mux
    epwm3_gpio_mux_init();
    // reset EPWM3
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM3);
    // stop EPWM3
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);

    // set EPWM3_CLK prescaler and highSpeedPrescaler    PWM_CLK = SYSCLK / (prescaler * highSpeedPrescaler)
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_2,EPWM_HSCLOCK_DIVIDER_1); //50Mhz
    // set EPWM3 cycle 50KHz , period counter = 50MHz/10KHz = 5000 , 5000/2 in up and down mode
    EPWM_setTimeBasePeriod(EPWM3_BASE,2500);
    // clear EPWM3 initial value
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    // set EPWM3 Period Load Mode
    EPWM_setPeriodLoadMode(EPWM3_BASE, EPWM_PERIOD_SHADOW_LOAD);
    // set EPWM3 Period load Event
//    EPWM_selectPeriodLoadEvent(EPWM3_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
    // set EPWM SYNC
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
    // set EPWM Phase Shift
    EPWM_setPhaseShift(EPWM3_BASE, 0);
    // disable Phase Shift Load
    EPWM_disablePhaseShiftLoad(EPWM3_BASE);

    // EPWM3 CMP_A & CMP_B: Load when counter equals zero or period
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    // EPWM3 %50 duty
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 1250);

    // EPWM3 Action Qualifier, EPWM3AB complementary
    // EPWM3_A : zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x
    EPWM_setActionQualifierShadowLoadMode(EPWM3_BASE, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
    // EPWM3_B : zero ⬆, period x, CMPA up ⬇, CMPA down ⬆, CMPB x, T1/T2 x
    EPWM_setActionQualifierShadowLoadMode(EPWM3_BASE, EPWM_ACTION_QUALIFIER_B, EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_B, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

    // set EPWM3 Emulation
    EPWM_setEmulationMode(EPWM3_BASE, EPWM_EMULATION_STOP_AFTER_NEXT_TB);

    // enable EPWM3 Time Base up and down mode
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //---------------------------------------------------------------------
    //--- Enable the clocks to the ePWM module.
    //--- Note: this should be done after all ePWM modules are configured
    //--- to ensure synchronization between the ePWM modules.
    //---------------------------------------------------------------------
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // TBCLK to ePWM modules enabled

}

/*
 * use EPWM3_A generate symmetric wave by using Dead Band
 */
void epwm_symmetric_deadband_test(void)
{
    // Must disable the clock to the ePWM modules to have all ePWM modules synchronized
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    /*
     * EPWM3 Channel AB: 10KHz, 50% duty, AB complementary using dead band module
     */
    // EPWM3 gpio mux
    epwm3_gpio_mux_init();
    // reset EPWM3
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM3);
    // stop EPWM3
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);

    // set EPWM3_CLK prescaler and highSpeedPrescaler    PWM_CLK = SYSCLK / (prescaler * highSpeedPrescaler)
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_2,EPWM_HSCLOCK_DIVIDER_1); //50Mhz
    // set EPWM3 cycle 50KHz , period counter = 50MHz/10KHz = 5000 , 5000/2 in up and down mode
    EPWM_setTimeBasePeriod(EPWM3_BASE,2500);
    // clear EPWM3 initial value
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0U);
    // set EPWM3 Period Load Mode
    EPWM_setPeriodLoadMode(EPWM3_BASE, EPWM_PERIOD_SHADOW_LOAD);
    // set EPWM3 Period load Event
//    EPWM_selectPeriodLoadEvent(EPWM3_BASE, EPWM_SHADOW_LOAD_MODE_COUNTER_ZERO);
    // set EPWM SYNC
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_ON_COUNTER_ZERO);
    // set EPWM Phase Shift
    EPWM_setPhaseShift(EPWM3_BASE, 0);
    // disable Phase Shift Load
    EPWM_disablePhaseShiftLoad(EPWM3_BASE);

    // EPWM3 CMP_A & CMP_B: Load when counter equals zero or period
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE,EPWM_COUNTER_COMPARE_A, EPWM_COMP_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 1250);

    // EPWM3 Action Qualifier, EPWM3AB complementary
    // EPWM3_A : zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x
    EPWM_setActionQualifierShadowLoadMode(EPWM3_BASE, EPWM_ACTION_QUALIFIER_A, EPWM_AQ_LOAD_ON_CNTR_ZERO_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM3_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);
    // Dead band
    // DB counter cycle use TB_CLK
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);
    // enable rising and failing mode
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    // set failing wave inverse
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED, EPWM_DB_POLARITY_ACTIVE_LOW);
    // set rising delay 50 * TB_CLK 50MHz = 1us
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, 50);
    // set failing delay 50 * TB_CLK 50MHz = 1us
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, 50);

    // set EPWM3 Emulation
    EPWM_setEmulationMode(EPWM3_BASE, EPWM_EMULATION_STOP_AFTER_NEXT_TB);

    // enable EPWM3 Time Base up and down mode
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    //---------------------------------------------------------------------
    //--- Enable the clocks to the ePWM module.
    //--- Note: this should be done after all ePWM modules are configured
    //--- to ensure synchronization between the ePWM modules.
    //---------------------------------------------------------------------
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // TBCLK to ePWM modules enabled

    // test_loop moditfy test_period test_cmpa check wave
    while(1){
        if(epwm_symmetric_test_val.debug_on){
            EPWM_setTimeBasePeriod(EPWM3_BASE,epwm_symmetric_test_val.test_period);
            EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A,epwm_symmetric_test_val.test_cmpa);
        }
        DEVICE_DELAY_US(500000);
    }
}
