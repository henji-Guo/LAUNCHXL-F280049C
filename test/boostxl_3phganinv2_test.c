/****************** BOOSTXL-3PhGaNInv *******************
# J8  80      GPIO0       EPWM1A
# J8  79      GPIO1       EPWM1B
# J8  78      GPIO6       EPWM4A
# J8  77      GPIO7       EPWM4B
# J8  76      GPIO2       EPWM2A
# J8  75      GPIO3       EPWM2B
# 
# J8  74      GPIO26      Over Temperature (Input LOW LEVEL Effective)
# 
# J6  53      GPIO33      PWM_ENABLE  (Output LOW LEVEL Effective)
# 
# J5  46      ADCINA4     VREF (about 3.3V)
# 
# J7  63      ADCINA6     Udc             SOC6
# J7  64      ADCINB6     UA              SOC5
# J7  65      ADCINC14    UB              SOC7
# J7  66      ADCINC1     UC              SOC8
# J7  67      ADCINC3     Ia              SOC5
# J7  68      ADCINC5     Ib              SOC6
# J7  69      ADCINA3     Ic              SOC5
# J7  70      ADCINA0     DACA_OUT
# 
# The Udc that VBUS Sense :
#     Udc_sensor = 4.22K / (4.22K + 100K) * VBUS 
# The VBUS 80V (abs max) scaled to 3.3V.
# So that Udc_sensor can equal :
#     Udc_sensor = ADC_SAMPLE_VAL / 4095.0f * 80.0f
# 
# Note that the motor three phase(UA/B/C) is same with Udc calculation. 
# 
# R_sample = 0.005 Ohm.
# 
# the current sample chip is INA240A1PW that Gain 20 and support PWM rejection.
# 
# Precision in-line phase current sensing wih 5-mohm shunt,
# for theoretical ± 16.5A full scale range),and ± 10.5A peak
# nominal range for the BOOSTXL-3PhGaNInv.
# 
# the offset Voltage is half of VREF(3.3V).
# 
#     result = 20 * ( V+ - V-) + 1.65
# 
#     Isen = ADC_SAMPLE_VAL / 4095.0f * 33 - 16.5
# 
****************** BOOSTXL-3PhGaNInv *******************/


/* #################### Include File #################### */
#include "test.h"
#include "device.h"
#include "driverlib.h"
#include "f28004x_device.h"
#include <math.h>
#include <stdio.h>
#include "user_driver.h"
#include "foc.h"
#include "vofa.h"
#include "Queue.h"

/* #################### User Define #################### */
/* Over Temperature PIN */
#define OT_PIN      26
/* PWM_ENABLE PIN */
#define nEnable     33

/* ADC Sample-Hold time */
#define ADC_SAMPLE_WINDOWS  11  // 11*SYS_CLK ns

/* ADC value scaling */
#define ADC_VOLTAGE_SCALE       81.5f
#define ADC_CURRENT_SCALE       33.0f
#define ADC_CURRENT_OFFSET      16.5f

/* PWM period calculate */
#define EPWM_CLK            100000000U  // 100MHz
#define USER_CONTROL_FREQ   8000U   // 8KHz
#define EPWM_PERIOD         ( 0.5 * EPWM_CLK / USER_CONTROL_FREQ)

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
#define MOTOR_Rs        0.5f   // motor phase resistance (Ohm)
#define MOTOR_Ld        0.0126f   // motor d axis inductances (Henry)
#define MOTOR_Lq        0.0126f   // motor q axis inductances (Henry)
#define MOTOR_Flux      0.0055f    // motor permanent magnet flux linkage (Wb)
#define MOTOR_Pn        6U      // motor Permanent magnet pole pairs (pole pairs)
#define MOTOR_Te        0.0f    // motor electromagnetic torque (N·m)
#define MOTOR_J         0.0f    // Motor inertia (Kg·m^2)
#define MOTOR_B         0.0f    // Motor friction constant (Kg·m^2/s)
#define MOTOR_Ke        MOTOR_Flux*(sqrtf(3)*1000U*MOTOR_Pn*2*M_PI)/60.0f  // Motor Back-EMF constant (Vpk_LL/krpm)
#define MOTOR_Kt        MOTOR_Flux*MOTOR_Pn*3/2.0f  // Motor torque constant (N·m/A)

/**
 * LED :
 *      LED_ON  GPIO LOW
 *      LED_OFF GPIO HIGH
 *      GPIO25  DRV8320RS Board LED(Yellow)
 *      GPIO23  LanunchPad Board LED5(Green)
 *      GPIO34  LanunchPad Board LED4(Red)
 */
typedef enum {
    LED_RED     =   23,
    LED_YELLOW  =   25,
    LED_GREEN   =   34
}BOARD_LED;
typedef enum {
    LED_ON      =   0,
    LED_OFF     =   1,
    LED_TOGGLE  =   2
}LED_STATUS;

/* log frame print */
struct foc_frame sendMsg;

/* sweep sample */
#define SWEEP_SAMPLE_POINT  256
float sweep_sample[SWEEP_SAMPLE_POINT];

/* #################### Function Declaration #################### */
static void boostxl_3phganiv2_gpio_mux(void);
static void boostxl_3phganiv2_enable(void);
static void boostxl_3phganiv2_disable(void);

static void led_gpio_init(void);
static void led(BOARD_LED led,LED_STATUS status);

static void timer_0_init(void);

static void epwm_gpio_mux(void);
static void epwm_1_init(void);
static void epwm_4_init(void);
static void epwm_2_init(void);

static void analog_subsys_init(void);
static void adc_a_init(void);
static void adc_b_init(void);
static void adc_c_init(void);

static void i2c_gpio_mux(void);
static void i2c_a_init(void);
static float as5600_get_angle(void);

__interrupt static void XINT1_ISR(void);
__interrupt static void ADCC1_ISR(void);
__interrupt static void TIMER0_ISR(void);

/* #################### Function Definition #################### */
static void boostxl_3phganiv2_gpio_mux(void)
{
    /* GPIO33 PWM_ENABLE (LOW LEVEL Effective) */
    GPIO_setPinConfig(GPIO_33_GPIO33);
    GPIO_setQualificationMode(nEnable, GPIO_QUAL_SYNC);
    GPIO_setPadConfig(nEnable, GPIO_PIN_TYPE_PULLUP);
    GPIO_writePin(nEnable,1);
    GPIO_setDirectionMode(nEnable, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(nEnable, GPIO_CORE_CPU1);

    /* GPIO26 Over Temperature (LOW LEVEL Effective) */
    GPIO_setPinConfig(GPIO_26_GPIO26);
    GPIO_setQualificationMode(OT_PIN, GPIO_QUAL_SYNC);
    GPIO_setPadConfig(OT_PIN, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(OT_PIN, GPIO_DIR_MODE_IN);
    GPIO_setMasterCore(OT_PIN, GPIO_CORE_CPU1);

    SysCtl_delay(50);

    /* enable GPIO26 OT input interrupt */
    GPIO_setInterruptPin(OT_PIN,GPIO_INT_XINT1);
    GPIO_setInterruptType(GPIO_INT_XINT1,GPIO_INT_TYPE_FALLING_EDGE);
    GPIO_enableInterrupt(GPIO_INT_XINT1);

    /* register XINT1 interrupt function and enable in PIE */
    Interrupt_register(INT_XINT1,&XINT1_ISR);
    // Interrupt_enable(INT_XINT1);
}

static void boostxl_3phganiv2_enable(void)
{
    GPIO_writePin(nEnable,0);
}

static void boostxl_3phganiv2_disable(void)
{
    GPIO_writePin(nEnable,1);
}

static void led_gpio_init(void)
{
    /**
     * LED5 Red:
     *      GPIO23 output,pullup and control by CPU1
     */
    GPIO_setPinConfig(GPIO_23_GPIO23);
    GPIO_setQualificationMode(23, GPIO_QUAL_SYNC);
    GPIO_setPadConfig(23, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(23, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(23, GPIO_CORE_CPU1);
    GPIO_writePin(23,1);

    /**
     * LED4 Green:
     *      GPIO34 output,pullup and control by CPU1
     */
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setQualificationMode(34, GPIO_QUAL_SYNC);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(34, GPIO_CORE_CPU1);
    GPIO_writePin(34,1);

    /**
     * LED YELLOW:
     *      GPIO25 output,pullup and control by CPU1
     */
    GPIO_setPinConfig(GPIO_25_GPIO25);
    GPIO_setQualificationMode(25, GPIO_QUAL_SYNC);
    GPIO_setPadConfig(25, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(25, GPIO_DIR_MODE_OUT);
    GPIO_setMasterCore(25, GPIO_CORE_CPU1);
    GPIO_writePin(25,1);
}

static void led(BOARD_LED led,LED_STATUS status)
{
    switch (status) {
        case LED_ON: 
            GPIO_writePin(led,LED_ON);
            break;
        case LED_OFF: 
            GPIO_writePin(led,LED_OFF);
            break;
        case LED_TOGGLE: 
            GPIO_togglePin(led);
            break;
        default:
            break;
    }
}

static void timer_0_init(void)
{
    /**
     *  CPUTIMER0 :
     *      TIM0_CLK 1MHz SYS_CLK/(99+1)
     */
    /* stop */
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    /* */
    CPUTimer_selectClockSource(CPUTIMER0_BASE,CPUTIMER_CLOCK_SOURCE_SYS,CPUTIMER_CLOCK_PRESCALER_1);
    /* set emulation mode*/
    CPUTimer_setEmulationMode(CPUTIMER0_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);

    /* set TIM0_CLK 1MHz */
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 99U);

    /* set TIM0 Period 0XFFFF_FFFF */
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);

    /* enable TIM0 interrupt */
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

    /* PieVectorTable register */
    // Interrupt_register(INT_TIMER0,&TIMER0_ISR);

    /* PIE enable */
    // Interrupt_enable(INT_TIMER0);

    /* reload TIM0 counter */
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    /* start TIM0 */
    CPUTimer_startTimer(CPUTIMER0_BASE);

}

static void epwm_gpio_mux(void)
{

    /*
     *  EPWM1AB GPIO MUX:
     *      GPIO0 --> EPWM1A
     *      GPIO1 --> EPWM1B
     *      Controller By CPU
     *      Synchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_0_EPWM1_A);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(0, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(0, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_1_EPWM1_B);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(1, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(1, GPIO_CORE_CPU1);

    /*
     *  EPWM4AB GPIO MUX:
     *      GPIO6 --> EPWM4A
     *      GPIO7 --> EPWM4B
     *      Controller By CPU
     *      Synchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_6_EPWM4_A);
    GPIO_setPadConfig(6, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(6, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(6, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_7_EPWM4_B);
    GPIO_setPadConfig(7, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(7, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(7, GPIO_CORE_CPU1);

    /*
     *  EPWM2AB GPIO MUX:
     *      GPIO2 --> EPWM2A
     *      GPIO3 --> EPWM2B
     *      Controller By CPU
     *      Synchronization to SYSCLK
     */
    GPIO_setPinConfig(GPIO_2_EPWM2_A);
    GPIO_setPadConfig(2, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(2, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(2, GPIO_CORE_CPU1);

    GPIO_setPinConfig(GPIO_3_EPWM2_B);
    GPIO_setPadConfig(3, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(3, GPIO_QUAL_SYNC);
    GPIO_setControllerCore(3, GPIO_CORE_CPU1);
}

static void epwm_1_init(void)
{
    /*
     *  EPWM1AB:
     *      frequency USER_CONTROL_FREQ
     *      active high complementary
     *      dead-band 1us
     *      trigger ADC SOCx event
     */

//    /* clock disable */
//    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);
//    /* enable clock*/
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM1);

    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM1);
    /* TBCLK:100MHz/(EPWM_ClockDivider*EPWM_HSClockDivider) ; TBPRD:TBCLK/(2*PWM_CLK)) in up and down mode */
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM1_BASE, EPWM_PERIOD);
    EPWM_setTimeBaseCounter(EPWM1_BASE, 0);

    /* TB counter mode: EPWM_COUNTER_MODE_UP_DOWN */
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    /* No Phase Shift */
    EPWM_disablePhaseShiftLoad(EPWM1_BASE);
    EPWM_setPhaseShift(EPWM1_BASE, 0);

    /* SYNC OUT: EPWM_SYNC_OUT_PULSE_DISABLED*/
    EPWM_setSyncOutPulseMode(EPWM1_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);

    /* Action Qualifier: zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x */
    EPWM_setCounterCompareShadowLoadMode(EPWM1_BASE, EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, EPWM_PERIOD * 0.5);

    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

    /* Dead-Band: DB_CLK=TB_CLK=100MHz */
    EPWM_setDeadBandCounterClock(EPWM1_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    /* set failing polarity inverse */
    EPWM_setDeadBandDelayPolarity(EPWM1_BASE, EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    /* enable rising and failing with 1us(100*DB_CLK) dead-band*/
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM1_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(EPWM1_BASE, 100U);
    EPWM_setFallingEdgeDelayCount(EPWM1_BASE, 100U);

    /* enable ADC trigger source: EPWM_SOC_A  EPWM_SOC_TBCTR_PERIOD */
    EPWM_enableADCTriggerEventCountInit(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    /* set Trip Zone stop EPWM6AB output */
    EPWM_setTripZoneAction(EPWM1_BASE,EPWM_TZ_ACTION_EVENT_TZA,EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM1_BASE,EPWM_TZ_ACTION_EVENT_TZB,EPWM_TZ_ACTION_LOW);
}

static void epwm_4_init(void)
{
    /*
     *  EPWM4AB:
     *      frequency USER_CONTROL_FREQ
     *      active high complementary
     *      dead-band 1us
     *      trigger ADC SOCx event
     */

//    /* clock disable */
//    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);
//    /* enable clock*/
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM4);

    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM4);
    /* TBCLK:100MHz/(EPWM_ClockDivider*EPWM_HSClockDivider) ; TBPRD:TBCLK/(2*PWM_CLK)) in up and down mode */
    EPWM_setClockPrescaler(EPWM4_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM4_BASE, EPWM_PERIOD);
    EPWM_setTimeBaseCounter(EPWM4_BASE, 0);

    /* TB counter mode: EPWM_COUNTER_MODE_UP_DOWN */
    EPWM_setTimeBaseCounterMode(EPWM4_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    /* No Phase Shift */
    EPWM_disablePhaseShiftLoad(EPWM4_BASE);
    EPWM_setPhaseShift(EPWM4_BASE, 0);

    /* SYNC OUT: EPWM_SYNC_OUT_PULSE_DISABLED*/
    EPWM_setSyncOutPulseMode(EPWM4_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);

    /* Action Qualifier: zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x */
    EPWM_setCounterCompareShadowLoadMode(EPWM4_BASE, EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, EPWM_PERIOD * 0.5);

    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM4_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

    /* Dead-Band: DB_CLK=TB_CLK=100MHz */
    EPWM_setDeadBandCounterClock(EPWM4_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    /* set failing polarity inverse */
    EPWM_setDeadBandDelayPolarity(EPWM4_BASE, EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    /* enable rising and failing with 1us(100*DB_CLK) dead-band*/
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM4_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(EPWM4_BASE, 100U);
    EPWM_setFallingEdgeDelayCount(EPWM4_BASE, 100U);

    /* enable ADC trigger source: EPWM_SOC_A  EPWM_SOC_TBCTR_PERIOD */
    // EPWM_enableADCTriggerEventCountInit(EPWM4_BASE, EPWM_SOC_A);
    // EPWM_setADCTriggerSource(EPWM4_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    // EPWM_setADCTriggerEventPrescale(EPWM4_BASE, EPWM_SOC_A, 1);
    // EPWM_enableADCTrigger(EPWM4_BASE, EPWM_SOC_A);

    /* set Trip Zone stop EPWM6AB output */
    EPWM_setTripZoneAction(EPWM4_BASE,EPWM_TZ_ACTION_EVENT_TZA,EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM4_BASE,EPWM_TZ_ACTION_EVENT_TZB,EPWM_TZ_ACTION_LOW);
}

static void epwm_2_init(void)
{
    /*
     *  EPWM2AB:
     *      frequency USER_CONTROL_FREQ
     *      active high complementary
     *      dead-band 1us
     *      trigger ADC SOCx event
     */

//    /* clock disable */
//    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);
//    /* enable clock*/
//    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_EPWM2);

    /* reset */
    SysCtl_resetPeripheral(SYSCTL_PERIPH_RES_EPWM2);
    /* TBCLK:100MHz/(EPWM_ClockDivider*EPWM_HSClockDivider) ; TBPRD:TBCLK/(2*PWM_CLK)) in up and down mode */
    EPWM_setClockPrescaler(EPWM2_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_setTimeBasePeriod(EPWM2_BASE, EPWM_PERIOD);
    EPWM_setTimeBaseCounter(EPWM2_BASE, 0);

    /* TB counter mode: EPWM_COUNTER_MODE_UP_DOWN */
    EPWM_setTimeBaseCounterMode(EPWM2_BASE, EPWM_COUNTER_MODE_UP_DOWN);

    /* No Phase Shift */
    EPWM_disablePhaseShiftLoad(EPWM2_BASE);
    EPWM_setPhaseShift(EPWM2_BASE, 0);

    /* SYNC OUT: EPWM_SYNC_OUT_PULSE_DISABLED*/
    EPWM_setSyncOutPulseMode(EPWM2_BASE, EPWM_SYNC_OUT_PULSE_DISABLED);

    /* Action Qualifier: zero ⬇, period x, CMPA up ⬆, CMPA down ⬇, CMPB x, T1/T2 x */
    EPWM_setCounterCompareShadowLoadMode(EPWM2_BASE, EPWM_COUNTER_COMPARE_A,EPWM_COMP_LOAD_ON_CNTR_ZERO);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, EPWM_PERIOD * 0.5);

    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_ZERO);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_PERIOD);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);

    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPB);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T1_COUNT_DOWN);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_UP);
    EPWM_setActionQualifierAction(EPWM2_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_NO_CHANGE, EPWM_AQ_OUTPUT_ON_T2_COUNT_DOWN);

    /* Dead-Band: DB_CLK=TB_CLK=100MHz */
    EPWM_setDeadBandCounterClock(EPWM2_BASE, EPWM_DB_COUNTER_CLOCK_FULL_CYCLE);

    /* set failing polarity inverse */
    EPWM_setDeadBandDelayPolarity(EPWM2_BASE, EPWM_DB_FED,EPWM_DB_POLARITY_ACTIVE_LOW);

    /* enable rising and failing with 1us(100*DB_CLK) dead-band*/
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_RED, true);
    EPWM_setDeadBandDelayMode(EPWM2_BASE, EPWM_DB_FED, true);
    EPWM_setRisingEdgeDelayCount(EPWM2_BASE, 100U);
    EPWM_setFallingEdgeDelayCount(EPWM2_BASE, 100U);

    /* enable ADC trigger source: EPWM_SOC_A  EPWM_SOC_TBCTR_PERIOD */
    // EPWM_enableADCTriggerEventCountInit(EPWM2_BASE, EPWM_SOC_A);
    // EPWM_setADCTriggerSource(EPWM2_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_ZERO);
    // EPWM_setADCTriggerEventPrescale(EPWM2_BASE, EPWM_SOC_A, 1);
    // EPWM_enableADCTrigger(EPWM2_BASE, EPWM_SOC_A);

    /* set Trip Zone stop EPWM6AB output */
    EPWM_setTripZoneAction(EPWM2_BASE,EPWM_TZ_ACTION_EVENT_TZA,EPWM_TZ_ACTION_LOW);
    EPWM_setTripZoneAction(EPWM2_BASE,EPWM_TZ_ACTION_EVENT_TZB,EPWM_TZ_ACTION_LOW);
}

static void pwm_setDuty(uint16_t Ua, uint16_t Ub, uint16_t Uc)
{
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, Ua);
    EPWM_setCounterCompareValue(EPWM4_BASE, EPWM_COUNTER_COMPARE_A, Ub);
    EPWM_setCounterCompareValue(EPWM2_BASE, EPWM_COUNTER_COMPARE_A, Uc);
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
     * J7_63 Udc |       ADCINA6      |    SOC 6   |   EPWM1_SOCA   |
     * J7_69 Ic  |       ADCINA3      |    SOC 5   |   EPWM1_SOCA   |
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

    /* ADCINA6 SOC6, no interrupt trigger */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN6, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINA3 SOC5, no interrupt trigger */
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCA_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

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
     * J7_64 Ua  |       ADCINB6      |    SOC 5   |   EPWM1_SOCA   |
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

    /* ADCINB6  SOC5, no interrupt trigger */
    ADC_setupSOC(ADCB_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN6, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCB_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

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
     * J7_65 Ub  |       ADCINC14     |    SOC 7   |   EPWM6_SOCA   |
     * J7_66 Uc  |       ADCINC1      |    SOC 8   |   EPWM6_SOCA   |
     * J7_67 Ia  |       ADCINC3      |    SOC 5   |   EPWM6_SOCA   |
     * J7_68 Ib  |       ADCINC5      |    SOC 6   |   EPWM6_SOCA   |
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

    /* ADCINC3 SOC5, no interrupt trigger */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER5, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN3, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER5, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINC5 SOC6, no interrupt trigger */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER6, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN5, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER6, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINC14 SOC7, no interrupt trigger */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER7, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN14, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER7, ADC_INT_SOC_TRIGGER_NONE);

    /* ADCINC1 SOC8, no interrupt trigger */
    ADC_setupSOC(ADCC_BASE, ADC_SOC_NUMBER8, ADC_TRIGGER_EPWM1_SOCA, ADC_CH_ADCIN1, ADC_SAMPLE_WINDOWS);
    ADC_setInterruptSOCTrigger(ADCC_BASE, ADC_SOC_NUMBER8, ADC_INT_SOC_TRIGGER_NONE);

    /* set ADCC_SOC8 EOC using ADCC_INT1 */
    ADC_setInterruptSource(ADCC_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER8);

    /* enable interrupt */
    ADC_enableInterrupt(ADCC_BASE, ADC_INT_NUMBER1);

    /* PieVectorTable register */
    Interrupt_register(INT_ADCC1,&ADCC1_ISR);

    /* PIE enable */
    Interrupt_enable(INT_ADCC1);

    /* enable */
    ADC_enableConverter(ADCC_BASE);

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
    I2C_setEmulationMode(I2CA_BASE, I2C_EMULATION_FREE_RUN);

    /* enable */
    I2C_enableModule(I2CA_BASE);
}

static float as5600_get_angle(void)
{
    // note that this function need cost 54us

    /* temporary variable */
    uint16_t angle[2];

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
            angle[i] = I2caRegs.I2CDRR.all;
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
    return ( (angle[0] << 8 | angle[1]) / 4095.0f * 2.0f * M_PI );
}

static void sweep_init(void)
{
    int i;

    for (i = 0; i < SWEEP_SAMPLE_POINT; i++)
        sweep_sample[i] = sinf( (2 * M_PI / SWEEP_SAMPLE_POINT) * i );
}

uint16_t count,index,T=36;
float time;
int rs_flag;
struct JustFloat frame;
__interrupt 
static void ADCC1_ISR(void)
{
    // /* run foc */
    foc_run(&foc_motor1);

    // if(rs_flag){
    //     if (fabsf(foc_motor1.Iabc[0]) < 0.5f)
    //         i += 0.01;
    //     pwm_setDuty((1-i)*6250,6250,6250);
    //     foc_motor1.motorRs = foc_motor1.Uabc[0] / foc_motor1.Iabc[0] * 2.0f / 3.0f;

    // }else
    //     pwm_setDuty(0,0,0);

    // Ud = Rs*Id + Ld * dId/dt - omega*Lq*Iq
    // Uq = Rs*Iq + Lq * dIq/dt + omega*(Lq*Iq + Pusi )
    // foc_motor1.motorLq = (foc_motor1.motorRs * foc_motor1.Id - foc_motor1.Ud) / (foc_motor1.motorFreq * 2 * M_PI * foc_motor1.Iq);

    /* vofa print */
#if 1
    vofa.setData(&vofa,foc_motor1.Uabc[0], 0);
    vofa.setData(&vofa,foc_motor1.Uabc[1], 1);
    vofa.setData(&vofa,foc_motor1.Uabc[2], 2);
    vofa.setData(&vofa,foc_motor1.motorRs, 3);
    vofa.setData(&vofa,foc_motor1.motorLq, 4);
    vofa.setData(&vofa,foc_motor1.Iq, 5);
    vofa.setData(&vofa,foc_motor1.Id, 6);
    vofa.setData(&vofa,foc_motor1.Ud, 7);
#else
    /* sweep function */
    if (index == 256) {
        index = 0;
        if (T > 0)
            T = T - 2;
        else{
            pwm_setDuty(0,0,0);
            boostxl_3phganiv2_disable();
            ESTOP0;
        }
    }

    if(count++ % T != 0)
        return;
    foc_motor1.Iq_ref = sweep_sample[index];
    time += T/40.0f;

    vofa.setData(&vofa,foc_motor1.Iq_ref, 0);
    vofa.setData(&vofa,foc_motor1.Iq, 1);
    vofa.setData(&vofa,foc_motor1.Iq_error, 2);
    vofa.setData(&vofa,foc_motor1.Id_ref, 3);
    vofa.setData(&vofa,foc_motor1.Id, 4);
    vofa.setData(&vofa,foc_motor1.Id_error, 5);
    vofa.setData(&vofa,time, 6);
    vofa.setData(&vofa,sweep_sample[index++], 7);
    vofa.print(&vofa);
#endif
    vofa.print(&vofa);

    /* clear EPWM SOCA flag */
    EPWM_clearADCTriggerFlag(EPWM1_BASE,EPWM_SOC_A);
    /* clear ADCB INT1 flag */
    ADC_clearInterruptStatus(ADCC_BASE, ADC_INT_NUMBER1);
    /* clear PIE FLAG */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt
static void XINT1_ISR(void)
{
    if(GPIO_readPin(OT_PIN) == 0){
        boostxl_3phganiv2_disable();
        led(LED_RED,LED_ON);
        while(1);
    }
    /* clear PIE FLAG */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

__interrupt 
static void TIMER0_ISR(void)
{
    /* clear TIM0 flag */
    CPUTimer_clearOverflowFlag(CPUTIMER0_BASE);

    /* clear PIE flag */
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}


int debug_on;
void boostxl_3phganinv2_test(void)
{

    /* GPIO MUX */
    boostxl_3phganiv2_gpio_mux();
    led_gpio_init();

    /* ANALOG SUBSYS Init*/
    analog_subsys_init();

    /* ADC Init */
    adc_a_init();
    adc_b_init();
    adc_c_init();

    /* TIMER Init */
    timer_0_init();

    /* I2C Init */
    i2c_gpio_mux();
    i2c_a_init();

    /* FOC Init */
    foc_init(&foc_motor1);

    /* VOFA+ Init */
    vofa_init(&vofa);

    /* SWEEP Init */
    sweep_init();

    /* Queue Init */
    SequenceQueue_Init(&bufQueue,100);
    
    /* enable boostxl-3phganiv2 */
    boostxl_3phganiv2_enable();

    /* EPWM Init */
    /* Must disable the clock to the ePWM modules to have all ePWM modules synchronized */
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);
    /* EPWM1/4/2 GPIO MUX Init */
    epwm_gpio_mux();
    /* EPWM1/4/2 Init */
    epwm_1_init();
    epwm_4_init();
    epwm_2_init();
    /* TBCLK to ePWM modules enabled */
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    while (1){
        // DEVICE_DELAY_US(500000);
        if(debug_on){
            pwm_setDuty(0,0,0);
        }
       if(SequenceQueue_isEmpty(&bufQueue) != 0 && uart_isIDLE(&bsp_uart) == 0){
            sendMsg = SequenceQueue_Pop(&bufQueue);
            uart_int_transmit(&bsp_uart,sendMsg.data,36);
       }
    }
    
}

/**
 * @brief Set PWM Duty
 *
 * @param Ua Set A phase PWM duty
 * @param Ub Set B phase PWM duty
 * @param Uc Set C phase PWM duty
 */
void svpwm_setDuty(struct foc* foc_handle)
{
    uint16_t period = foc_handle->pwm_period;
    pwm_setDuty(foc_handle->Tcm1 * period,
                foc_handle->Tcm2 * period,
                foc_handle->Tcm3 * period);
}

/**
 * @brief Get the motor mechanical radian angle
 *
 * @param foc_handle
 */
void get_RadianAngle(struct foc* foc_handle)
{
    return;
#if 1
    foc_handle->thetaELEC += 2.0 * M_PI * 125e-6 * foc_handle->motorFreq;
    foc_handle->thetaELEC = fmodf(foc_handle->thetaELEC,2 * M_PI);
    foc_handle->thetaMECH = as5600_get_angle();
    foc_handle->nowClock = CPUTimer_getTimerCount(CPUTIMER0_BASE);
#else
    foc_handle->thetaMECH = as5600_get_angle();
    foc_handle->nowClock = CPUTimer_getTimerCount(CPUTIMER0_BASE);
    foc_handle->thetaELEC = foc_handle->thetaMECH * foc_handle->motorPoles;
    foc_handle->thetaELEC = fmodf(foc_handle->thetaELEC, 2 * M_PI);
#endif
}

/**
 * @brief Get the motor three phase current Iabc
 *
 * @param foc_handle pointer to foc component handler
 */
void get_current_Iabc(struct foc* foc_handle)
{
    /* Ia Ib Ic */
    foc_handle->Iabc[0] = -(ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER5) / 4095.0f * ADC_CURRENT_SCALE - ADC_CURRENT_OFFSET);
    foc_handle->Iabc[1] = -(ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER6) / 4095.0f * ADC_CURRENT_SCALE - ADC_CURRENT_OFFSET);
    foc_handle->Iabc[2] = -(ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER5) / 4095.0f * ADC_CURRENT_SCALE - ADC_CURRENT_OFFSET);
}

/**
 * @brief Get the motor three phase voltage Uabc
 *
 * @param foc_handle pointer to foc component handler
 */
void get_voltage_Uabc(struct foc* foc_handle)
{
    /* Ua Ub Uc Udc */
    foc_handle->Uabc[0] = ADC_readResult(ADCBRESULT_BASE, ADC_SOC_NUMBER5) / 4095.0f * ADC_VOLTAGE_SCALE;
    foc_handle->Uabc[1] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER7) / 4095.0f * ADC_VOLTAGE_SCALE;
    foc_handle->Uabc[2] = ADC_readResult(ADCCRESULT_BASE, ADC_SOC_NUMBER8) / 4095.0f * ADC_VOLTAGE_SCALE;
    foc_handle->Udc     = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER6) / 4095.0f * ADC_VOLTAGE_SCALE;
}

/**
 * @brief Get the motor rotation Speed
 *
 * @param foc_handle pointer to foc component handler
 */
void get_motorSpeed(struct foc* foc_handle)
{
    /* auxiliary variable */
    float auxTheta,tmpSpeed;
    uint32_t auxClock;

    /* Mechanical Angle diff */
    auxTheta = foc_handle->thetaMECH - foc_handle->thetaOLDMECH;

    /* timer diff */
    auxClock = -(foc_handle->nowClock - foc_handle->lastClock);

    /* Calculation circle number */
    if (fabsf(auxTheta) >= 0.8 * 2 * M_PI){
        foc_handle->nowTurns = (auxTheta > 0) ? foc_handle->lastTurns - 1 : foc_handle->lastTurns + 1;
        return;
    }

    float alpha = 0.01f/(0.01f + (float)(auxClock * 1e-6));
    /* calculate motor rotation speed RPM (rotation per minute) */
    tmpSpeed = ( (float)(foc_handle->nowTurns - foc_handle->lastTurns) * 2 * M_PI + auxTheta ) * 30.0f / ( auxClock * 1e-6 * M_PI );
    foc_handle->motorSpeed = alpha * foc_handle->motorSpeed + (1-alpha) * tmpSpeed;

    foc_handle->thetaOLDMECH = foc_handle->thetaMECH;
    foc_handle->lastTurns = foc_handle->nowTurns;
    foc_handle->lastClock = foc_handle->nowClock;
}

/**
 * @brief Vofa Print Data Message
 * 
 */
void vofa_print(struct vofa* vofa)
{
#if 0
    uint16_t *tx_buf = (uint16_t*)&vofa->frame;
    int i;
    for (i = 0; i < VOFA_CH_COUNT * 2; i++){
        while(SciaRegs.SCIFFTX.bit.TXFFST >= 14);
        SciaRegs.SCITXBUF.bit.TXDT = tx_buf[i];
        SciaRegs.SCITXBUF.bit.TXDT = tx_buf[i] >> 8;
    }
    while(SciaRegs.SCIFFTX.bit.TXFFST >= 12);
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[0];
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[1];
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[2];
    SciaRegs.SCITXBUF.bit.TXDT = (uint16_t)vofa->frame.tail[3];
#else
    struct foc_frame logFrame;
    uint32_t *tx_buf = (uint32_t*)&vofa->frame;
    int i;

    for (i = 0; i < VOFA_CH_COUNT ; i ++){
        logFrame.data[4*i] = tx_buf[i] & 0xFF;
        logFrame.data[4*i+1] = (tx_buf[i] & 0xFF00) >> 8;
        logFrame.data[4*i+2] = (tx_buf[i] & 0xFF0000) >> 16;
        logFrame.data[4*i+3] = (tx_buf[i] & 0xFF000000) >> 24;
    }
    logFrame.data[4*i] = (uint16_t)vofa->frame.tail[0];
    logFrame.data[4*i+1] = (uint16_t)vofa->frame.tail[1];
    logFrame.data[4*i+2] = (uint16_t)vofa->frame.tail[2];
    logFrame.data[4*i+3] = (uint16_t)vofa->frame.tail[3];
#endif

   if(SequenceQueue_isFull(&bufQueue) != 0);
        SequenceQueue_Push(&bufQueue,logFrame);
}

