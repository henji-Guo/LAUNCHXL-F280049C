#include "main.h"

#define LOG_USE_VOFA_QUEUE

void main(void)
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

    // // SysConfig settings
    // Board_init();

    // Uart Init
    uart_init();

    // Vofa Init
    vofa_init(&vofa);

#ifdef LOG_USE_VOFA_QUEUE
    /* Queue Message */
    QueueElemType sendMsg;
    
    /* Queue Init */
    SequenceQueue_Init(&bufQueue, 50);
#endif

    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    EINT;
    ERTM;

    while (1)
    {
        // TODO

#ifdef LOG_USE_VOFA_QUEUE
        if (SequenceQueue_isEmpty(&bufQueue) != 0 && uart_isIDLE(&bsp_uart) == 0)
        {
            sendMsg = SequenceQueue_Pop(&bufQueue);
            uart_int_transmit(&bsp_uart, sendMsg.data, sizeof(sendMsg.data));
        }
#endif
    }
}
