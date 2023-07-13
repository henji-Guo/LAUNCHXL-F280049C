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
#include <math.h>
#include "bsp_led.h"
#include <stdio.h>

#define ADC_SAMPLE_WINDOWS  11  // 11*SYS_CLK ns

/**
 * Voltage rangle is 0 ~ 57.5285V
 * DR8320RS Resistance partial voltage（Resistance 82K series resistance 4.99K)
 * ADC sample voltage = 4.99K/(4.99K+82K) * Actual Voltage
 */
#define ADC_VOLTAGE_MAX             57.5285 // 4095*3.3*(4.99+82)/4.99/4095
#define ADC_VAL_TO_VOLTAGE(val)     val*3.3f*(4.99f+82)/4.99f/4095

/**
 * Current rangle is -21.4208 ~ +21.4351
 * Isensor = Vsignal / 0.007Ω
 * VPGA_IN = 27.4K/(27.4K+2.49K)*(Vsignal-1.65V)+1.65V
 * VPGA_IN = ADC_result_val / 4096 / PGA_GAIN * 3.3V
 * Vsignal = （VPGA_IN - 1.65V）* (27.4K+2.49K) / 27.4K +1.65V
 */
#define ADC_CURRENT_MAX                     42.8558 // -21.4208 ~ +21.4351
#define ADC_VAL_TO_CURRENT(val,offset)     ((val*3.3f/4095/12-1.65f)*(27.4f+2.49f)/27.4f+1.65f)/0.007f-offset;

/* pwm period calculate */
#define EPWM_CLK            100000000U  // 100MHz
#define USER_CONTROL_FREQ   10000U   // 10Hz
#define EPWM_PERIOD         ( 0.5 * EPWM_CLK / USER_CONTROL_FREQ);

/**
 * % 直流母线电压
 * Udc = 320
 * % PWM开关频率
 * Tpwm = 1/10000
 * % 电机参数
 * % 磁极对数(Permanent magnet pole pairs)
 * Motor.Pn = 4 % ploe pairs
 * % 永磁体磁链(permanent magnet flux linkage)
 * Motor.Flux= 0.175 % Wb
 * % 相电阻(Phase resistance)
 * Motor.Rs = 2.875 % Ohm
 * % dq轴电感(d and q axis inductances)
 * Motor.Ld = 8.5e-3 % Henry
 * Motor.Lq = 8.5e-3 % Henry
 * % 电机转矩(Electromagnetic torque)
 * Motor.Te = 6 % N·m
 * % 转动惯量(Motor inertia)
 * Motor.J = 0.003 % Kg.m^2
 * % 沾粘系数(Friction constant)
 * Motor.B = 0.008 % Kg.m^2/s
 * % 反电动势常数(Back-EMF constant)
 * Motor.Ke = Motor.Flux * (sqrt(3)*1000*Motor.Pn*2*pi) / 60 % Vpk_LL/krpm
 * % 转矩常数(Torque constant)
 * Motor.Kt = Motor.Flux * Motor.Pn * 3/2 % N·m/A 
 */
/* Motor Parameter */
#define MOTOR_Rs        7.75f   // motor phase resistance (Ohm)
#define MOTOR_Ld        0.01f   // motor d axis inductances (Henry)
#define MOTOR_Lq        0.01f   // motor q axis inductances (Henry)
#define MOTOR_Flux      0.0f    // motor permanent magnet flux linkage (Wb)
#define MOTOR_Pn        7U      // motor Permanent magnet pole pairs (pole pairs)
#define MOTOR_Te        0.0f    // motor electromagnetic torque (N·m)
#define MOTOR_J         0.0f    // Motor inertia (Kg·m^2)
#define MOTOR_B         0.0f    // Motor friction constant (Kg·m^2/s)
#define MOTOR_Ke        MOTOR_Flux*(sqrtf(3)*1000U*MOTOR_Pn*2*M_PI)/60  // Motor Back-EMF constant (Vpk_LL/krpm)
#define MOTOR_Kt        MOTOR_Flux*MOTOR_Pn*3/2.0f  // Motor torque constant (N·m/A)

struct {
    uint16_t debug_on;
    uint16_t calibration_on;
    uint16_t drv8320rs_on;
    
    uint16_t currentVal[3];     // Iabc current ADC raw
    uint16_t voltageVal[4];     // voltage Uabc,Udc ADC raw
    
    uint16_t period;            // pwm period
    
    float Speedref;             // motor reference speed

    uint16_t motorPoles;        // motor poles
    float motorFreq;            // motor spin frequency
    float motorSpeed;           // motor spin speed
    float motorRs;              // motor phase resistance
    float motorVoltage;         // motor control voltage
    float VFOffset;             // V/F control voltage compensation

    float Iabc[3];              // real Iabc
    float Uabc[3];              // real Uabc
    float Ioffset[3];           // Iabc offset
    float Udc;                  // real Udc
    float Ualpha;               // Valpha
    float Ubeta;                // Vbeta
    float Ialpha;               // Ialpha
    float Ibeta;                // Ibeta
    
    float thetaElEC;            // electrical theta
    float thetaMECH;            // mechanical theta
    float theta_as5600;         // theta from encoder as5600
    float theta_as5600_old;     // last theta from encoder as5600

    uint16_t angle[4];           // raw angle from as5600
    
    void (*pwm_enable)();
    void (*pwm_disable)();
    void(*pwm_duty)(uint16_t Ua, uint16_t Ub, uint16_t Uc);
    void(*gpio_mux_init)();
}launchPad;

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
static void i2c_gpio_mux(void);
static void i2c_a_init(void);
static void timer_0_init(void);
static void timer_1_init(void);
static void ADCB1_ISR(void);
static void TIMER0_ISR(void);
static void TIMER1_ISR(void);
static void svpwm_overmodulation1(float *T1, float *T2);
static void svpwm(float Ualpha, float Ubeta, float Udc);
static void pwm_setDuty(uint16_t Ua, uint16_t Ub, uint16_t Uc);
static void get_motor_angle(void);

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
    EPWM_setTimeBasePeriod(EPWM6_BASE, launchPad.period);
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
    EPWM_setTimeBasePeriod(EPWM5_BASE, launchPad.period);
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
    EPWM_setTimeBasePeriod(EPWM3_BASE, launchPad.period);
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

static void i2c_gpio_mux(void)
{
    /*
     *  I2CA GPIO MUX:
     *      GPIO35 --> I2C SDA  (pull up)
     *      GPIO37 --> I2C SCL  (pull up)
     *      Controller By CPU
     *      Asynchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_35_I2CA_SDA);
    GPIO_setPadConfig(35, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(35, GPIO_QUAL_ASYNC);
    GPIO_setMasterCore(35, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_37_I2CA_SCL);
    GPIO_setPadConfig(37, GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(37, GPIO_QUAL_ASYNC);
    GPIO_setMasterCore(37, GPIO_CORE_CPU1);
}

static void i2c_a_init(void)
{
    /**
     *  I2CA:
     *      I2CA_CLK: 1MHz
     *      slaver address: 0x36
     *      transfer bit width: 8 bit
     */
    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_I2CA);

    /* disable */
    I2C_disableModule(I2CA_BASE);

    /* I2C_CLK 1MHz , Clock reference "TRM 24.1.5 Clock Generation" */
    /* configure I2C Module clock using I2CPSC register I2C Module clock should be 7-12Mhz */
    I2caRegs.I2CPSC.bit.IPSC = 4U;   // I2C_Module_CLK(Fmod) = 100MHz SYSCLK / (ISP = 9 + 1) = 10Mhz
    
    /* configure I2C Baud rate using I2CCLKL and I2CCLKH */
    /* I2C Baud rate = ((I2CCLKL + d)+(I2CCLKH + d))/Fmod = ((5+5)+(5+5))/10MHz = 1/1MHz */
    I2caRegs.I2CCLKL = 5U;
    I2caRegs.I2CCLKH = 5U;
    
    /* config master send mode */
    I2C_setConfig(I2CA_BASE, I2C_CONTROLLER_SEND_MODE);

    /* set slaver address 0x36 */
    I2C_setTargetAddress(I2CA_BASE, 0x36U);

    /* disable loop back mode */
    I2C_disableLoopback(I2CA_BASE);

    /* set bit width 8 bits */
    I2C_setBitCount(I2CA_BASE, I2C_BITCOUNT_8);

    /* set slaver address mode 7 bits address */
    I2C_setAddressMode(I2CA_BASE, I2C_ADDR_MODE_7BITS);

    /* enable FIFO */
    I2C_enableFIFO(I2CA_BASE);

    /* set emulation mode */
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_STOP_SCL_LOW);

    /* enable */
    I2C_enableModule(I2CA_BASE);
}

static void get_motor_angle(void)
{
        /* I2C Master TX/RX Flowchat */
    /* I2C Basic configuration */

    // disable I2C ISR=0
    I2caRegs.I2CMDR.bit.IRS = 0;

    // Clock reference "TRM 24.1.5 Clock Generation"
    // Configure I2C Module clock using I2CPSC register I2C Module clock should be 7-12Mhz
    I2caRegs.I2CPSC.bit.IPSC = 4;  // I2C_Module_CLK(Fmod) = 100MHz SYSCLK / (ISP=4 + 1) = 20Mhz

    // Configure I2C Baud rate using I2CCLKL and I2CCLKH
    // I2C Baud rate = (I2CCLKL + d)+(I2CCLKH + d)/Fmod
    I2caRegs.I2CCLKL = 5;
    I2caRegs.I2CCLKH = 5;

    // Configure I2C Own address using I2COAR (no need in master tx/rx mode)
    I2caRegs.I2COAR.bit.OAR = 0x00;

    // Configure I2C Slave address to talk to using I2CSAR
    I2caRegs.I2CSAR.bit.SAR = 0x36;
    /* FIFO mode (16-deep x 8-bit FIFO) */
    /* enable and configure TX/RX FIFO,configure TX/RX FIFO level,if need can enable FIFO interrupts*/
    I2caRegs.I2CFFTX.bit.I2CFFEN = 0x1;
    I2caRegs.I2CFFTX.bit.TXFFRST = 0x1;
    I2caRegs.I2CFFTX.bit.TXFFIL = 0x0;
//    I2caRegs.I2CFFTX.bit.TXFFIENA = 0x1;
    I2caRegs.I2CFFRX.bit.RXFFRST = 0x1;
    I2caRegs.I2CFFRX.bit.RXFFIL = 0x2;
//    I2caRegs.I2CFFRX.bit.RXFFIENA = 0x1;

    /* Application code decides to initiate i2c transaction */
    /* check I2C bus free I2CSTR.BB == 0 */
    while(I2caRegs.I2CSTR.bit.BB);

    /* Master Transmitter MST=1 or Master Receive MST=0 */
    /* Master Transmitter MST=1 */ 
    I2caRegs.I2CMDR.bit.MST = 1;

    /* check Repeat mode RM=1 or No-repeat mode RM=0 */
    /* note that repeat mode don't care I2CNT */
    I2caRegs.I2CMDR.bit.RM = 0;

    /* No-repeat mode */
    /* set I2CCNT number of bytes to be transmitted */
    I2caRegs.I2CCNT = 1;

    /* set I2CMDR */
    I2caRegs.I2CMDR.all = 0x6620;

    /* put data in fifo */
    for (int i = 0; i < 1; i++) {
        /* check TX FIFO is full */
        while(I2caRegs.I2CFFTX.bit.TXFFST == 16);
        I2caRegs.I2CDXR.bit.DATA = 0x0C;
    }

    /* check i2c tx status */
    while(I2caRegs.I2CSTR.bit.XSMT != 1 || I2caRegs.I2CSTR.bit.BYTESENT != 1);
    I2caRegs.I2CSTR.bit.BYTESENT = 1;

    /* check I2CSTR ARDY bit */
    while(I2caRegs.I2CSTR.bit.ARDY != 1);
    I2caRegs.I2CSTR.bit.ARDY = 1;

    /* enter recieve mode */
    /* Master Receive MST=0 */ 
    I2caRegs.I2CMDR.bit.MST = 0;

    /* check Repeat mode RM=1 or No-repeat mode RM=0 */
    /* note that repeat mode don't care I2CNT */
    I2caRegs.I2CMDR.bit.RM = 0;

    /* No-repeat mode */
    /* set I2CCNT number of bytes to be recieved */
    I2caRegs.I2CCNT = 2;

    /* set I2CMDR */
    I2caRegs.I2CMDR.all = 0x6420; // TRM misdescription

    while(I2caRegs.I2CFFRX.bit.RXFFINT != 1);

    /* put data in fifo */
    for (int i = 0; i < 2; i++) {
        // check TX FIFO is full
        if(I2caRegs.I2CFFRX.bit.RXFFST > 0){
            launchPad.angle[i] = I2caRegs.I2CDRR.all;
        }
    }

    /* check NACK */ 
    if(I2caRegs.I2CSTR.bit.NACKSNT != 1)
        I2caRegs.I2CMDR.bit.NACKMOD = 1;
    while(I2caRegs.I2CSTR.bit.NACKSNT != 1);
    I2caRegs.I2CSTR.bit.NACKSNT = 1;

    /* send stop */
    I2caRegs.I2CMDR.bit.STP = 1;

    /* calculate AS5600 raw value to angle(0,2PI) */
    launchPad.theta_as5600 = (launchPad.angle[0] << 8 | launchPad.angle[1]) / 4095.0f * 2 * M_PI;
    launchPad.motorSpeed = (launchPad.theta_as5600 - launchPad.theta_as5600_old)*30/(100e-6*M_PI);
    launchPad.theta_as5600_old = launchPad.theta_as5600;
}

static void timer_0_init(void)
{
    /**
     *  CPUTIMER0 :
     *      TIM0_CLK 1MHz SYS_CLK/(99+1)
     *      TIM0_Period 10KHz = 100us = 100 / 1MHz
     */
    /* stop */
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    /* */
    CPUTimer_selectClockSource(CPUTIMER0_BASE,CPUTIMER_CLOCK_SOURCE_SYS,CPUTIMER_CLOCK_PRESCALER_1);
    /* set emulation mode*/
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    /* set TIM0_CLK 1MHz */
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 99U);

    /* set TIM0 Period 100 (10KHz) */
    CPUTimer_setPeriod(CPUTIMER0_BASE, 100U);

    /* enable TIM0 interrupt */
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

    /* PieVectorTable register */
    Interrupt_register(INT_TIMER0,&TIMER0_ISR);

    /* PIE enable */
    Interrupt_enable(INT_TIMER0);

    /* reload TIM0 counter */
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);
}

static void timer_1_init(void)
{
    /**
     *  CPUTIMER1 :
     *      Using timer1 to calculate motor speed
     *      TIM0_CLK 1MHz SYS_CLK/(99+1)
     *      TIM0_Period 1KHz = 1000us = 1000 / 1MHz
     */
    /* stop */
    CPUTimer_stopTimer(CPUTIMER1_BASE);

    /* */
    CPUTimer_selectClockSource(CPUTIMER1_BASE,CPUTIMER_CLOCK_SOURCE_SYS,CPUTIMER_CLOCK_PRESCALER_1);
    /* set emulation mode*/
    CPUTimer_setEmulationMode(CPUTIMER1_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    /* set TIM0_CLK 1MHz */
    CPUTimer_setPreScaler(CPUTIMER1_BASE, 99U);

    /* set TIM0 Period 100 (10KHz) */
    CPUTimer_setPeriod(CPUTIMER1_BASE, 100U);

    /* enable TIM0 interrupt */
    CPUTimer_enableInterrupt(CPUTIMER1_BASE);

    /* PieVectorTable register */
    Interrupt_register(INT_TIMER1,&TIMER1_ISR);

    /* PIE enable */
    Interrupt_enable(INT_TIMER1);

    /* reload TIM0 counter */
    CPUTimer_reloadTimerCounter(CPUTIMER1_BASE);
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

static void svpwm_overmodulation1(float *T1, float *T2)
{
    if((*T1 + *T2) > 1.0f){
        *T1 = *T1 / (*T1 + *T2);
        *T2 = *T2 / (*T1 + *T2);
    }
}

static void svpwm(float Ualpha, float Ubeta, float Udc)
{
    /**
     *  N = 4C + 2B + A :
     *      Ubeta > 0 ? A = 1 : A = 0;
     *      sqrtf(3)/2 * Ualpha - 0.5 * Ubeta > 0 ? B = 1 : B = 0;
     *      -sqrtf(3)/2 * Ualpha - 0.5 * Ubeta > 0 ? C = 1 : C = 0;
     *  X Y Z :
     *      X = (sqrtf(3) * Ts) / Udc * Ubeta;
     *      Y = (sqrtf(3) * Ts) / Udc * (sqrtf(3)/2 * Ualpha + 0.5 * Ubeta);
     *      Z = (sqrtf(3) * Ts) / Udc * (-sqrtf(3)/2 * Ualpha + 0.5 * Ubeta);
     *  Ta Tb Tc :
     *      Ta = (Ts - T1 - T2) / 4;
     *      Tb = Ta + T1 / 2;
     *      Tc = Tb + T2 / 2;
     *  Relationship table :
     *      |    N   |   3  |   1  |   5  |   4  |   6  |   2  |
     *      | sector |   Ⅰ  |   Ⅱ  |   Ⅲ  |  Ⅳ  |  Ⅴ  |  Ⅵ  |
     *      |   T1   |  -Z  |   Z  |   X  |  -X  |  -Y  |   Y  |
     *      |   T2   |   X  |   Y  |  -Y  |   Z  |  -Z  |  -X  |
     *      |   T0   |      T0/T7 = （Ts - T1 - T2) / 2
     *      |  Tcm1  |  Ta  |  Tb  |  Tc  |  Tc  |  Tb  |  Ta  |
     *      |  Tcm2  |  Tb  |  Ta  |  Ta  |  Tb  |  Tc  |  Tc  |
     *      |  Tcm3  |  Tc  |  Tc  |  Tb  |  Ta  |  Ta  |  Tb  |
     */
    /* auxiliary variables */
    uint16_t N;
    float X,Y,Z;
    float Ta,Tb,Tc;
    float T1,T2;
    float Tcm1,Tcm2,Tcm3;

    /* calculate N */
    N = Ubeta > 0 ? 1 : 0;
    N += (sqrtf(3.0f)/2.0f * Ualpha - 0.5f * Ubeta > 0 ? 1 : 0) << 1;
    N += (sqrtf(3.0f)/-2.0f * Ualpha - 0.5f * Ubeta > 0 ? 1 : 0) << 2;

    /* calculate X Y Z , Ts is PWM period counter */
    X = (sqrtf(3.0f) * 1.0f) / Udc * Ubeta;
    Y = (sqrtf(3.0f) * 1.0f) / Udc * (sqrtf(3.0f)/2.0f * Ualpha + 0.5f * Ubeta);
    Z = (sqrtf(3.0f) * 1.0f) / Udc * (sqrtf(3.0f)/-2.0f * Ualpha + 0.5f * Ubeta);

    /* identify sectors for T1 T2 */
    switch (N){
        case 1:
            T1 = Z;
            T2 = Y;
            break;
        case 2:
            T1 =  Y;
            T2 = -X;
            break;
        case 3:
            T1 = -Z;
            T2 =  X;
            break;
        case 4:
            T1 = -X;
            T2 =  Z;
            break;
        case 5:
            T1 =  X;
            T2 = -Y;
            break;
        case 6:
            T1 = -Y;
            T2 = -Z;
            break;
        default:
            T1 = 0;
            T2 = 0;
            break;
    }

    /* overmodulation process */
    svpwm_overmodulation1(&T1,&T2);

    /* calculate Ta Tb Tc */
    Ta = (1 - T1 - T2) / 4.0f;
    Tb = Ta + T1 / 2.0f;
    Tc = Tb + T2 / 2.0f;

    /* calculate Tcm1 Tcm2 Tcm3 */
    switch (N){
        case 1:
            Tcm1 = Tb;
            Tcm2 = Ta;
            Tcm3 = Tc;
            break;
        case 2:
            Tcm1 = Ta;
            Tcm2 = Tc;
            Tcm3 = Tb;
            break;
        case 3:
            Tcm1 = Ta;
            Tcm2 = Tb;
            Tcm3 = Tc;
            break;
        case 4:
            Tcm1 = Tc;
            Tcm2 = Tb;
            Tcm3 = Ta;
            break;
        case 5:
            Tcm1 = Tc;
            Tcm2 = Ta;
            Tcm3 = Tb;
            break;
        case 6:
            Tcm1 = Tb;
            Tcm2 = Tc;
            Tcm3 = Ta;
            break;
        default:
            Tcm1 = 0;
            Tcm2 = 0;
            Tcm3 = 0;
            break;
    }

    /* set PWM duty */
    pwm_setDuty(Tcm1*launchPad.period,Tcm2*launchPad.period,Tcm3*launchPad.period);

}

void epwm_adc_test(void)
{
    /* init launchPad value */
    launchPad.debug_on = 0;
    launchPad.calibration_on = 0;
    launchPad.period = EPWM_PERIOD;
    launchPad.motorVoltage = 5;
    launchPad.Speedref = 1000;
    launchPad.motorFreq = 0;
    launchPad.VFOffset = 1.0;
    launchPad.Udc = 14.0;
    launchPad.motorRs = MOTOR_Rs;
    launchPad.motorPoles = MOTOR_Pn;
    launchPad.pwm_duty = &pwm_setDuty;
    launchPad.pwm_enable = &pwm_enable;
    launchPad.pwm_disable = &pwm_disable;

    while (!launchPad.drv8320rs_on);

    /* GPIO MUX */
    drv8320rs_gpio_mux();
    drv8320rs_enable();
    bsp_led5_init();

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

    /* TIMER Init */
    timer_0_init();
    timer_1_init();

    /* I2C Init */
    i2c_gpio_mux();
    i2c_a_init();

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

    /* enable */
    CPUTimer_startTimer(CPUTIMER0_BASE);

    while(1){
        // get_motor_angle();

        if(launchPad.debug_on){
            drv8320rs_disable();
            pwm_disable();
            
        }

        if (launchPad.calibration_on){
            launchPad.pwm_disable();
            SysCtl_delay(10);
            EPWM_forceADCTrigger(EPWM6_BASE, EPWM_SOC_A);
            SysCtl_delay(10);
            launchPad.Ioffset[0] = (float)ADC_VAL_TO_CURRENT(launchPad.currentVal[0],0);
            launchPad.Ioffset[1] = (float)ADC_VAL_TO_CURRENT(launchPad.currentVal[1],0);
            launchPad.Ioffset[2] = (float)ADC_VAL_TO_CURRENT(launchPad.currentVal[2],0);
            launchPad.calibration_on = 0;
            EPWM_clearADCTriggerFlag(EPWM6_BASE,EPWM_SOC_A);
            launchPad.pwm_enable();
        }
        
        DEVICE_DELAY_US(10000);
    }
}

__interrupt static void ADCB1_ISR(void)
{
    /* read ADC result value */
    
    /* Ia ADCA SOC0 , Ib ADCC SOC0 , Ic ADCB SOC0 */
    launchPad.currentVal[0] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    launchPad.currentVal[1] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER0);
    launchPad.currentVal[2] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER0);
    
    /* Ua ADCA SOC1 , Ub ADCB SOC1 , Uc ADCC SOC1 , Udc ADCC SOC2 */
    launchPad.voltageVal[0] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
    launchPad.voltageVal[1] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER1);
    launchPad.voltageVal[2] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER1);
    launchPad.voltageVal[3] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER2);

    /* calculate current and voltage */
    launchPad.Iabc[0] = (float)ADC_VAL_TO_CURRENT(launchPad.currentVal[0],launchPad.Ioffset[0]);
    launchPad.Iabc[1] = (float)ADC_VAL_TO_CURRENT(launchPad.currentVal[1],launchPad.Ioffset[0]);
    launchPad.Iabc[2] = (float)ADC_VAL_TO_CURRENT(launchPad.currentVal[2],launchPad.Ioffset[0]);

    launchPad.Udc = (float)ADC_VAL_TO_VOLTAGE(launchPad.voltageVal[3]);
    launchPad.Uabc[0] = (float)ADC_VAL_TO_VOLTAGE(launchPad.voltageVal[0]);
    launchPad.Uabc[1] = (float)ADC_VAL_TO_VOLTAGE(launchPad.voltageVal[1]);
    launchPad.Uabc[2] = (float)ADC_VAL_TO_VOLTAGE(launchPad.voltageVal[2]);

    /* clear EPWM SOCA flag */
    EPWM_clearADCTriggerFlag(EPWM6_BASE,EPWM_SOC_A);
    /* clear ADCB INT1 flag */
    ADC_clearInterruptStatus(ADCB_BASE, ADC_INT_NUMBER1);
    /* clear PIE FLAG */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt static void TIMER0_ISR(void)
{
    get_motor_angle();
    /* using Speedref calculate frequency */
    /* n = 60*fe/Pn */
    launchPad.motorFreq = (launchPad.Speedref * launchPad.motorPoles)/60.0f;

    /* Δθ = ω * Ts = 2πf * Ts （Delta electrical angle）*/
    launchPad.thetaElEC += 2 * M_PI * 100e-6f * launchPad.motorFreq;
    launchPad.thetaElEC = fmodf(launchPad.thetaElEC, 2*M_PI);

    /* using frequency calculate voltage */
    /* Uout = (Umax-Umin)/(FREQmax-FREQmin) x FREQ_reference + Uo */
    launchPad.motorVoltage = 12.0f/140 * launchPad.motorFreq + launchPad.VFOffset;

    /* set PWM */
    svpwm(launchPad.motorVoltage * cosf(launchPad.thetaElEC), launchPad.motorVoltage * sinf(launchPad.thetaElEC), launchPad.Udc);

    printf("angle:%f,%f,%f\n",launchPad.thetaElEC,launchPad.theta_as5600,launchPad.motorSpeed);

    /* clear TIM0 flag */
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    /* clear PIE flag */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt static void TIMER1_ISR(void)
{
    //TODO

    /* clear TIM0 flag */
    CPUTimer_clearOverflowFlag(CPUTIMER1_BASE);

}
