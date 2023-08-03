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
    //
    // SCIA -> mySCI0 Pinmux
    //
    GPIO_setPinConfig(GPIO_28_SCIA_RX);
    GPIO_setPadConfig(28, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(28, GPIO_QUAL_ASYNC);

    GPIO_setPinConfig(GPIO_29_SCIA_TX);
    GPIO_setPadConfig(29, GPIO_PIN_TYPE_STD | GPIO_PIN_TYPE_PULLUP);
    GPIO_setQualificationMode(29, GPIO_QUAL_ASYNC);

    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF | SCI_INT_TXFF | SCI_INT_FE | SCI_INT_OE | SCI_INT_PE | SCI_INT_RXERR | SCI_INT_RXRDY_BRKDT | SCI_INT_TXRDY);
    SCI_clearOverflowStatus(SCIA_BASE);
    SCI_resetTxFIFO(SCIA_BASE);
    SCI_resetRxFIFO(SCIA_BASE);
    SCI_resetChannels(SCIA_BASE);
    SCI_setConfig(SCIA_BASE, DEVICE_LSPCLK_FREQ, 6500000, (SCI_CONFIG_WLEN_8|SCI_CONFIG_STOP_ONE|SCI_CONFIG_PAR_NONE));
    SCI_disableLoopback(SCIA_BASE);
    SCI_performSoftwareReset(SCIA_BASE);
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX0, SCI_FIFO_RX0);
    SCI_enableFIFO(SCIA_BASE);
    SCI_enableModule(SCIA_BASE);

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
