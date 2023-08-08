#include "device.h"
//#include "board.h"
#include "user_driver.h"
#include "test.h"
#include "stdio.h"
#include "string.h"


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

//    // SysConfig settings
//    Board_init();

    uart_init();

    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    EINT;
    ERTM;

    printf("******** APPLICATION STATR ********\r\n");

    DEVICE_DELAY_US(2000000);

   boostxl_3phganinv2_test();

    while (1)
    {
        //TODO

        DEVICE_DELAY_US(500000);
    }

}

// To use printf
int fputc(int _c, register FILE *_fp)
{
    SCI_writeCharBlockingFIFO(SCIA_BASE, _c);
    return _c;
}
int putc(int _c, register FILE *_fp)
{
    SCI_writeCharBlockingFIFO(SCIA_BASE, _c);
    return _c;
}
int putchar(int data)
{
    SCI_writeCharBlockingFIFO(SCIA_BASE, data);
    return data;
}
int fputs(const char *_ptr, register FILE *_fp)
{
    unsigned int i, len;
    len = strlen(_ptr);
    for (i = 0; i < len; i++)
    {
        SCI_writeCharBlockingFIFO(SCIA_BASE, _ptr[i]);
    }
    return len;
}
