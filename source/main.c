#include "device.h"
#include "board.h"
#include "user_driver.h"
#include "test.h"
#include "stdio.h"

/**
 * main.c
 */

int main(void)
{
    // Initialize device clock and peripherals
    Device_init();

    // Disable pin locks and enable internal pullups
    Device_initGPIO();

    // Initialize PIE and clear PIE registers. Disables CPU interrupts
    Interrupt_initModule();

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR)
    Interrupt_initVectorTable();

    // SysConfig settings
    Board_init();

    printf("******** APPLICATION STATR ********\r\n");

    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    EINT;
    ERTM;

    while(1){

        //led_test();
        sci_a_test();
        DEVICE_DELAY_US(500000);
    }

}

// To use printf
int fputc(int _c, FILE *_fp)
{
    SCI_writeCharBlockingNonFIFO(SCIA_BASE, _c);
    return 0;
}
