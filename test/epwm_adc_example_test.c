/*
 * epwm_adc_example_test.c
 *
 *  Created on: 2023年7月7日
 *      Author: GHJ
 */
#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"

#define ADC_SAMPLE_WINDOWS  11  // 11*SYS_CLK ns

/**
 * Voltage rangle is 0 ~ 57.5285V
 * DR8320RS Resistance partial voltage（Resistance 82K series resistance 4.99K)
 * ADC sample voltage = 4.99K/(4.99K+82K) * Actual Voltage
 */
#define ADC_VOLTAGE_MAX     57.5285 // 4095*3.3*(4.99+82)/4.99/4095
#define ADC_VAL_TO_VOLTAGE(val)     val*3.3f*(4.99f+82)/4.99f/4095

/**
 * Current rangle is -21.4208 ~ +21.4351
 * Isensor = Vsignal / 0.007Ω
 * VPGA_IN = 27.4K/(27.4K+2.49K)*(Vsignal-1.65V)+1.65V
 * VPGA_IN = ADC_result_val / 4096 / PGA_GAIN * 3.3V
 * Vsignal = （VPGA_IN - 1.65V）* (27.4K+2.49K) / 27.4K +1.65V
 */
#define ADC_CURRENT_MAX     42.8558 // -21.4208 ~ +21.4351
#define ADC_VAL_TO_CURRENT(val,offset)     ((val*3.3f/4095/12-1.65f)*(27.4f+2.49f)/27.4f+1.65f)/0.007f-offset;

struct {
    uint16_t debug_on;
    uint16_t calibration_on;
    uint16_t currentVal[3];     // Iabc
    uint16_t voltageVal[4];     // Uabc and Udc
    uint16_t Ta;
    uint16_t Tb;
    uint16_t Tc;
    float Iabc[3];
    float Uabc[3];
    float Ioffset[3];
    float Udc;
    float Ualpha;
    float Ubeta;
    float Ialpha;
    float Ibeta;
    float thetaElEC;
    void (*pwm_enable)();
    void (*pwm_disable)();
    void(*pwm_duty)(uint16_t Ua, uint16_t Ub, uint16_t Uc);
    void(*gpio_mux_init)();
}LAUNCHXL_F280049C_HANDLER;

/* Function Declaration */
static void drv8320rs_gpio_mux(void);
static void drv8320rs_enable(void);
static void drv8320rs_disable(void);
static void epwm_gpio_mux(void);
static void epwm_6_init(void);
static void epwm_5_init(void);
static void epwm_3_init(void);
static void pwm_enable(void);
static void pwm_disable(void);
static void analog_subsys_init(void);
static void adc_a_init(void);
static void adc_b_init(void);
static void adc_c_init(void);
static void dac_b_init(void);
static void pga_5_init(void);
static void pga_3_init(void);
static void pga_1_init(void);
static void ADCB1_ISR(void);
static void pwm_setDuty(uint16_t Ua, uint16_t Ub, uint16_t Uc);

static void drv8320rs_gpio_mux(void)
{
    /* J1_3 GPIO13 control Drv8320RS: 1 enable , 0 disable */
    GPIO_setPinConfig(GPIO_13_GPIO13);
    GPIO_setPadConfig(13,GPIO_PIN_TYPE_STD);
    GPIO_setDirectionMode(13,GPIO_DIR_MODE_OUT);
    GPIO_setQualificationMode(13,GPIO_QUAL_SYNC);
    GPIO_setMasterCore(13,GPIO_CORE_CPU1);
    GPIO_writePin(13,0);
}

static void drv8320rs_enable(void)
{
    GPIO_writePin(13,1);
}

static void drv8320rs_disable(void)
{
    GPIO_writePin(13,0);
}

static void epwm_gpio_mux(void)
{

    /*
     *  EPWM6AB GPIO MUX:
     *      GPIO10 --> EPWM6A
     *      GPIO11 --> EPWM6B
     *      Controller By CPU
     *      Synchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_10_EPWM6_A);
    GPIO_setPadConfig(10, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(10, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(10, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_11_EPWM6_B);
    GPIO_setPadConfig(11, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(11, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(11, GPIO_CORE_CPU1);

    /*
     *  EPWM5AB GPIO MUX:
     *      GPIO8 --> EPWM5A
     *      GPIO9 --> EPWM5B
     *      Controller By CPU
     *      Synchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_8_EPWM5_A);
    GPIO_setPadConfig(8, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(8, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(8, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_9_EPWM5_B);
    GPIO_setPadConfig(9, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(9, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(9, GPIO_CORE_CPU1);

    /*
     *  EPWM3AB GPIO MUX:
     *      GPIO4 --> EPWM3A
     *      GPIO5 --> EPWM3B
     *      Controller By CPU
     *      Synchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_4_EPWM3_A);
    GPIO_setPadConfig(4, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(4, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(4, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_5_EPWM3_B);
    GPIO_setPadConfig(5, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(5, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(5, GPIO_CORE_CPU1);
}

static void epwm_6_init(void)
{
    /*
     *  EPWM6AB:
     *      frequency 10KHz
     *      active high complementary
     *      dead-band 1us
     *      trigger ADC SOCx event
     */

//    /* clock disable */
//    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);
//    /* enable clock*/
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM6);

    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM6);
    /* TBCLK:100MHz/(EPWM_ClockDivider*EPWM_HSClockDivider) ; TBPRD:TBCLK/(2*PWM_CLK)=>5000) in up and down mode */
    EPWM_setClockPrescaler(EPWM6_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM6_BASE, 5000U);
    EPWM_setTimeBaseCounter(EPWM6_BASE, 0);

    /* TB counter mode: EPWM_COUNTER_MODE_UP_DOWN */
    EPWM_setTimeBaseCounterMode(EPWM6_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    /* No Phase Shift */
    EPWM_disablePhaseShiftLoad(EPWM6_BASE);
    EPWM_setPhaseShift(EPWM6_BASE, 0);

    /* SYNC OUT: EPWM_SYNC_OUT_PULSE_DISABLED*/
    EPWM_setSyncOutPulseMode(EPWM6_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);

    /* Action Qualifier: zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x */
    EPWM_setCounterCompareShadowLoadMode(EPWM6_BASE, EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, 2500U);

    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM6_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

    /* Dead-Band: DB_CLK=TB_CLK=100MHz */
    EPWM_setDeadBandCounterClock(EPWM6_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    /* set failing polarity inverse */
    EPWM_setDeadBandDelayPolarity(EPWM6_BASE, EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    /* enable rising and failing with 1us(100*DB_CLK) dead-band*/
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM6_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(EPWM6_BASE, 100U);
    EPWM_setFallingEdgeDelayCount(EPWM6_BASE, 100U);

    /* enable ADC trigger source: EPWM_SOC_A  EPWM_SOC_TBCTR_PERIOD */
    EPWM_enableADCTriggerEventCountInit(EPWM6_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM6_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM6_BASE, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(EPWM6_BASE, EPWM_SOC_A);

    /* set Trip Zone stop EPWM6AB output */
    EPWM_setTripZoneAction(EPWM6_BASE,EPWM_TZ_ACTION_EVENT_TZA,EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM6_BASE,EPWM_TZ_ACTION_EVENT_TZB,EPWM_TZ_ACTION_LOW);
}

static void epwm_5_init(void)
{
    /*
     *  EPWM5AB:
     *      frequency 10KHz
     *      active high complementary
     *      dead-band 1us
     *      trigger ADC SOCx event
     */

//    /* clock disable */
//    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);
//    /* enable clock*/
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM5);

    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM5);
    /* TBCLK:100MHz/(EPWM_ClockDivider*EPWM_HSClockDivider) ; TBPRD:TBCLK/(2*PWM_CLK)=>5000) in up and down mode */
    EPWM_setClockPrescaler(EPWM5_BASE, EPWM_CLOCK_DIVIDER_1,EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM5_BASE, 5000U);
    EPWM_setTimeBaseCounter(EPWM5_BASE, 0);

    /* TB counter mode: EPWM_COUNTER_MODE_UP_DOWN */
    EPWM_setTimeBaseCounterMode(EPWM5_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    /* No Phase Shift */
    EPWM_disablePhaseShiftLoad(EPWM5_BASE);
    EPWM_setPhaseShift(EPWM5_BASE, 0);

    /* SYNC OUT: EPWM_SYNC_OUT_PULSE_DISABLED*/
    EPWM_setSyncOutPulseMode(EPWM5_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);

    /* Action Qualifier: zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x */
    EPWM_setCounterCompareShadowLoadMode(EPWM5_BASE, EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, 2500U);

    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM5_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

    /* Dead-Band: DB_CLK=TB_CLK=100MHz */
    EPWM_setDeadBandCounterClock(EPWM5_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    /* set failing polarity inverse */
    EPWM_setDeadBandDelayPolarity(EPWM5_BASE, EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    /* enable rising and failing with 1us(100*DB_CLK) dead-band*/
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM5_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(EPWM5_BASE, 100U);
    EPWM_setFallingEdgeDelayCount(EPWM5_BASE, 100U);

    /* enable ADC trigger source: EPWM_SOC_A  EPWM_SOC_TBCTR_PERIOD */
    EPWM_enableADCTrigger(EPWM5_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM5_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);

    /* set Trip Zone stop EPWM5AB output */
    EPWM_setTripZoneAction(EPWM5_BASE,EPWM_TZ_ACTION_EVENT_TZA,EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM5_BASE,EPWM_TZ_ACTION_EVENT_TZB,EPWM_TZ_ACTION_LOW);
}

static void epwm_3_init(void)
{
    /*
     *  EPWM3AB:
     *      frequency 10KHz
     *      active high complementary
     *      dead-band 1us
     *      trigger ADC SOCx event
     */
//    /* clock disable */
//    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
//    /* enable clock*/
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM3);
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM3);
    /* TBCLK:100MHz/(EPWM_ClockDivider*EPWM_HSClockDivider) ; TBPRD:TBCLK/(2*PWM_CLK)=>5000) in up and down mode */
    EPWM_setClockPrescaler(EPWM3_BASE, EPWM_CLOCK_DIVIDER_1,EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM3_BASE, 5000U);
    EPWM_setTimeBaseCounter(EPWM3_BASE, 0);

    /* TB counter mode: EPWM_COUNTER_MODE_UP_DOWN */
    EPWM_setTimeBaseCounterMode(EPWM3_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    /* No Phase Shift */
    EPWM_disablePhaseShiftLoad(EPWM3_BASE);
    EPWM_setPhaseShift(EPWM3_BASE, 0);

    /* SYNC OUT: EPWM_SYNC_OUT_PULSE_DISABLED*/
    EPWM_setSyncOutPulseMode(EPWM3_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);

    /* Action Qualifier: zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x */
    EPWM_setCounterCompareShadowLoadMode(EPWM3_BASE, EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, 2500U);

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

    /* Dead-Band: DB_CLK=TB_CLK=100MHz */
    EPWM_setDeadBandCounterClock(EPWM3_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    /* set failing polarity inverse */
    EPWM_setDeadBandDelayPolarity(EPWM3_BASE, EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    /* enable rising and failing with 1us(100*DB_CLK) dead-band*/
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM3_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(EPWM3_BASE, 100U);
    EPWM_setFallingEdgeDelayCount(EPWM3_BASE, 100U);

    /* enable ADC trigger source: EPWM_SOC_A  EPWM_SOC_TBCTR_PERIOD */
    EPWM_enableADCTrigger(EPWM3_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM3_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);

    /* set Trip Zone stop EPWM3AB output */
    EPWM_setTripZoneAction(EPWM3_BASE,EPWM_TZ_ACTION_EVENT_TZA,EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM3_BASE,EPWM_TZ_ACTION_EVENT_TZB,EPWM_TZ_ACTION_LOW);
}

static void analog_subsys_init(void)
{
    /**
     * analog subsystem init:
     *      using internal voltage reference 3.3V(2*1.65)
     */
    SysCtl_delay(50);
    ASysCtl_setAnalogReferenceInternal(ASYSCTL_VREFHIA | ASYSCTL_VREFHIB | ASYSCTL_VREFHIC);
    ASysCtl_setAnalogReference1P65(ASYSCTL_VREFHIA | ASYSCTL_VREFHIB | ASYSCTL_VREFHIC);
    SysCtl_delay(50);
}

static void adc_a_init(void)
{
    /**
     *  ADCA：ADC_CLK 50MHz, 3.3V reference voltage, Acquisition window duration at least 75ns (8*SYS_CLK)
     *      name |      input pin     | soc number | trigger source | interrupt number |
     * J3_29 Ia  | PGA5_OUT(ADCINA14) |    SOC 0   |   EPWM6_SOCA   |
     * J3_23 Ua  |       ADCINA5      |    SOC 1   |   EPWM6_SOCA   |
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCA);

    /* disable */
    ADC_disableConverter(ADCA_BASE);

    /* set ADC voltage reference internal 3.3V*/
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    /* set prescaler ADC_CLK = 100MHz/2 =50MHz */
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_2_0);

    /* set the timing of the end-of-conversion pulse */
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    /* set soc number priority */
    ADC_setSOCPriority(ADCA_BASE,ADC_PRI_ALL_HIPRI);

    /* ADCINA14 SOC0, no interrupt trigger */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN14, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINA5 SOC1, no interrupt trigger */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN5, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    /* enable */
    ADC_enableConverter(ADCA_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void adc_b_init(void)
{
    /**
     *  ADCB：ADC_CLK 50MHz, 3.3V reference voltage, Acquisition window duration at least 75ns (8*SYS_CLK)
     *      name |      input pin     | soc number | trigger source | interrupt number |
     * J3_27 Ic  |  PGA1_OUT(ADCINB7) |    SOC 0   |   EPWM6_SOCA   |
     * J3_25 Ub  |       ADCINB0      |    SOC 1   |   EPWM6_SOCA   |
     * J3_26 Udc |       ADCINB1      |    SOC 2   |   EPWM6_SOCA   |   ADC_INT_NUMBER1
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCC);

    /* disable */
    ADC_disableConverter(ADCB_BASE);

    /* set ADC voltage reference internal 3.3V*/
    ADC_setVREF(ADCB_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    /* set prescaler ADC_CLK = 100MHz/2 =50MHz */
    ADC_setPrescaler(ADCB_BASE, ADC_CLK_DIV_2_0);

    /* set the timing of the end-of-conversion pulse */
    ADC_setInterruptPulseMode(ADCB_BASE, ADC_PULSE_END_OF_CONV);

    /* set soc number priority */
    ADC_setSOCPriority(ADCB_BASE,ADC_PRI_ALL_HIPRI);

    /* ADCINB7 SOC0, no interrupt trigger */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN7, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINB0 SOC1, no interrupt trigger */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN0, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINB1 SOC2, no interrupt trigger */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER2, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN1, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    /* set ADCB_SOC2 EOC using ADCB_INT1 */
    ADC_setInterruptSource(ADCB_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER2);

    /* enable interrupt */
    ADC_enableInterrupt(ADCB_BASE, ADC_INT_NUMBER1);

    /* PieVectorTable register */
    Interrupt_register(INT_ADCB1,&ADCB1_ISR);

    /* PIE enable */
    Interrupt_enable(INT_ADCB1);

    /* enable */
    ADC_enableConverter(ADCB_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void adc_c_init(void)
{
    /**
     *  ADCC：ADC_CLK 50MHz, 3.3V reference voltage, Acquisition window duration at least 75ns (8*SYS_CLK)
     *      name |      input pin     | soc number | trigger source | interrupt number |
     * J3_28 Ib  |  PGA3_OUT(ADCINC7) |    SOC 0   |   EPWM6_SOCA   |
     * J3_24 Uc  |       ADCINC2      |    SOC 1   |   EPWM6_SOCA   |
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_ADCC);

    /* disable */
    ADC_disableConverter(ADCC_BASE);

    /* set ADC voltage reference internal 3.3V*/
    ADC_setVREF(ADCC_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    /* set prescaler ADC_CLK = 100MHz/2 =50MHz */
    ADC_setPrescaler(ADCC_BASE, ADC_CLK_DIV_2_0);

    /* set the timing of the end-of-conversion pulse */
    ADC_setInterruptPulseMode(ADCC_BASE, ADC_PULSE_END_OF_CONV);

    /* set soc number priority */
    ADC_setSOCPriority(ADCC_BASE,ADC_PRI_ALL_HIPRI);

    /* ADCINC7 SOC0, no interrupt trigger */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN7, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER0, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINC2 SOC1, no interrupt trigger */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM6_SOCA, ADC_CH_ADCIN2, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER1, ADC_INT_SOC_TRIGGER_NONE);

    /* enable */
    ADC_enableConverter(ADCC_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void dac_b_init(void)
{
    /**
     *  DACB:
     *      using internal reference voltage 3.3V
     *      Supplied with 1.65V bias voltage
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_DACB);

    /* disable */
    DAC_disableOutput(DACB_BASE);

    /* set 3.3V internal reference voltage */
    DAC_setReferenceVoltage(DACB_BASE, DAC_REF_ADC_VREFHI);

    /* 3.3V(1.65*2) internal VREF need set gain x2 */
    DAC_setGainMode(DACB_BASE, DAC_GAIN_TWO);

    /* set DACB load mode SYSCLK */
    DAC_setLoadMode(DACB_BASE, DAC_LOAD_SYSCLK);

    /* set DACB 1.65V(0.5*4096) output */
    DAC_setShadowValue(DACB_BASE, 2048U);

    /* enable */
    DAC_enableOutput(DACB_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void pga_5_init(void)
{
    /**
     *  PGA5:
     *      set Gain x12
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_PGA5);

    /* disable */
    PGA_disable(PGA5_BASE);

    /* set Gain x12 */
    PGA_setGain(PGA5_BASE, PGA_GAIN_12);

    /* disable filter */
    PGA_setFilterResistor(PGA5_BASE, PGA_LOW_PASS_FILTER_DISABLED);

    /* enable */
    PGA_enable(PGA5_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void pga_3_init(void)
{
    /**
     *  PGA3:
     *      set Gain x12
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_PGA3);

    /* disable */
    PGA_disable(PGA3_BASE);

    /* set Gain x12 */
    PGA_setGain(PGA3_BASE, PGA_GAIN_12);

    /* disable filter */
    PGA_setFilterResistor(PGA3_BASE, PGA_LOW_PASS_FILTER_DISABLED);

    /* enable */
    PGA_enable(PGA3_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void pga_1_init(void)
{
    /**
     *  PGA1:
     *      set Gain x12
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_PGA1);

    /* disable */
    PGA_disable(PGA1_BASE);

    /* set Gain x12 */
    PGA_setGain(PGA1_BASE, PGA_GAIN_12);

    /* disable filter */
    PGA_setFilterResistor(PGA1_BASE, PGA_LOW_PASS_FILTER_DISABLED);

    /* enable */
    PGA_enable(PGA1_BASE);

    /* delay 5ms power up*/
    DEVICE_DELAY_US(5000);
}

static void pwm_setDuty(uint16_t Ua, uint16_t Ub, uint16_t Uc)
{
    EPWM_setCounterCompareValue(EPWM6_BASE, EPWM_COUNTER_COMPARE_A, Ua);
    EPWM_setCounterCompareValue(EPWM5_BASE, EPWM_COUNTER_COMPARE_A, Ub);
    EPWM_setCounterCompareValue(EPWM3_BASE, EPWM_COUNTER_COMPARE_A, Uc);
}

static void pwm_disable(void)
{
    EPWM_forceTripZoneEvent(EPWM6_BASE,EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWM5_BASE,EPWM_TZ_FORCE_EVENT_OST);
    EPWM_forceTripZoneEvent(EPWM3_BASE,EPWM_TZ_FORCE_EVENT_OST);
}

static void pwm_enable(void)
{
    EPWM_clearTripZoneFlag(EPWM6_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWM5_BASE, EPWM_TZ_FLAG_OST);
    EPWM_clearTripZoneFlag(EPWM3_BASE, EPWM_TZ_FLAG_OST);
}

void epwm_adc_test(void)
{
    /* init LAUNCHXL_F280049C_HANDLER value */
    LAUNCHXL_F280049C_HANDLER.debug_on = 0;
    LAUNCHXL_F280049C_HANDLER.calibration_on = 0;
    LAUNCHXL_F280049C_HANDLER.pwm_duty = &pwm_setDuty;
    LAUNCHXL_F280049C_HANDLER.pwm_enable = &pwm_enable;
    LAUNCHXL_F280049C_HANDLER.pwm_disable = &pwm_disable;
 
    /* GPIO MUX */
    drv8320rs_gpio_mux();
    // drv8320rs_enable();

    /* ANALOG SUBSYS Init*/
    analog_subsys_init();

    /* ADC Init */
    adc_a_init();
    adc_b_init();
    adc_c_init();

    /* PGA Init */
    pga_5_init();
    pga_3_init();
    pga_1_init();

    /* DAC Init */
    dac_b_init();

    /* EPWM Init */
    /* Must disable the clock to the ePWM modules to have all ePWM modules synchronized */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    /* EPWM6/5/3 GPIO MUX Init */
    epwm_gpio_mux();
    /* EPWM6/5/3 Init */
    epwm_6_init();
    epwm_5_init();
    epwm_3_init();
    /* TBCLK to ePWM modules enabled */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    while(1){
        if(LAUNCHXL_F280049C_HANDLER.debug_on){
            drv8320rs_disable();
            pwm_disable();
        }

        if (LAUNCHXL_F280049C_HANDLER.calibration_on){
            LAUNCHXL_F280049C_HANDLER.pwm_disable();
            SysCtl_delay(10);
            EPWM_forceADCTrigger(EPWM6_BASE, EPWM_SOC_A);
            SysCtl_delay(10);
            LAUNCHXL_F280049C_HANDLER.Ioffset[0] = (float)ADC_VAL_TO_CURRENT(LAUNCHXL_F280049C_HANDLER.currentVal[0],0);
            LAUNCHXL_F280049C_HANDLER.Ioffset[1] = (float)ADC_VAL_TO_CURRENT(LAUNCHXL_F280049C_HANDLER.currentVal[1],0);
            LAUNCHXL_F280049C_HANDLER.Ioffset[2] = (float)ADC_VAL_TO_CURRENT(LAUNCHXL_F280049C_HANDLER.currentVal[2],0);
            LAUNCHXL_F280049C_HANDLER.calibration_on = 0;
            EPWM_clearADCTriggerFlag(EPWM6_BASE,EPWM_SOC_A);
            LAUNCHXL_F280049C_HANDLER.pwm_enable();
        }
        
        DEVICE_DELAY_US(500000);
    }
}

__interrupt static void ADCB1_ISR(void)
{
    /* read ADC result value */
    
    /* Ia ADCA SOC0 , Ib ADCC SOC0 , Ic ADCB SOC0 */
    LAUNCHXL_F280049C_HANDLER.currentVal[0] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    LAUNCHXL_F280049C_HANDLER.currentVal[1] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
    LAUNCHXL_F280049C_HANDLER.currentVal[2] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    
    /* Ua ADCA SOC1 , Ub ADCB SOC1 , Uc ADCC SOC1 , Udc ADCC SOC2 */
    LAUNCHXL_F280049C_HANDLER.voltageVal[0] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
    LAUNCHXL_F280049C_HANDLER.voltageVal[1] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    LAUNCHXL_F280049C_HANDLER.voltageVal[2] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
    LAUNCHXL_F280049C_HANDLER.voltageVal[3] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);

    /* calculate current and voltage */
    LAUNCHXL_F280049C_HANDLER.Iabc[0] = (float)ADC_VAL_TO_CURRENT(LAUNCHXL_F280049C_HANDLER.currentVal[0],LAUNCHXL_F280049C_HANDLER.Ioffset[0]);
    LAUNCHXL_F280049C_HANDLER.Iabc[1] = (float)ADC_VAL_TO_CURRENT(LAUNCHXL_F280049C_HANDLER.currentVal[1],LAUNCHXL_F280049C_HANDLER.Ioffset[0]);
    LAUNCHXL_F280049C_HANDLER.Iabc[2] = (float)ADC_VAL_TO_CURRENT(LAUNCHXL_F280049C_HANDLER.currentVal[2],LAUNCHXL_F280049C_HANDLER.Ioffset[0]);

    LAUNCHXL_F280049C_HANDLER.Udc = (float)ADC_VAL_TO_VOLTAGE(LAUNCHXL_F280049C_HANDLER.voltageVal[3]);
    LAUNCHXL_F280049C_HANDLER.Uabc[0] = (float)ADC_VAL_TO_VOLTAGE(LAUNCHXL_F280049C_HANDLER.voltageVal[0]);
    LAUNCHXL_F280049C_HANDLER.Uabc[1] = (float)ADC_VAL_TO_VOLTAGE(LAUNCHXL_F280049C_HANDLER.voltageVal[1]);
    LAUNCHXL_F280049C_HANDLER.Uabc[2] = (float)ADC_VAL_TO_VOLTAGE(LAUNCHXL_F280049C_HANDLER.voltageVal[2]);

    /* clear EPWM SOCA flag */
    EPWM_clearADCTriggerFlag(EPWM6_BASE,EPWM_SOC_A);
    /* clear ADCB INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
    /* clear PIE FLAG */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}
