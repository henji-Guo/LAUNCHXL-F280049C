/*
 * timer_example_test.c
 *
 *  Created on: 2023年6月26日
 *      Author: GHJ
 */
#include "test.h"
#include "bsp_led.h"
__interrupt void INT_myCPUTIMER0_ISR(void);


void timer_test()
{
    bsp_led5_init();

    // STOP TIMER0
    CpuTimer0Regs.TCR.bit.TSS = 1;
    // 1MHZ ; TIMER_CLK = CPU_CLK/(TDDRH:TDDR)
    CpuTimer0Regs.TPRH.bit.TDDRH = 0x0000;
    CpuTimer0Regs.TPR.bit.TDDR = 100;
    // 1s ; reload value 1000000(D)=0xF4240(H)
    CpuTimer0Regs.PRD.all = 0xF4240;
    // FREE & SOFT are related to simulation
    CpuTimer0Regs.TCR.bit.FREE = 0;
    CpuTimer0Regs.TCR.bit.SOFT = 0;
    // enable interrupt
    CpuTimer0Regs.TCR.bit.TIE = 1;
    // execute count reload
    CpuTimer0Regs.TCR.bit.TRB = 1;
    // configure vector isr function, this address is protected and need to use EALLOW & EDIS.
    EALLOW;
    PieVectTable.TIMER0_INT = &INT_myCPUTIMER0_ISR;
    EDIS;
    // enable PIE TIMER0 interrupt
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // enable CPU IER interrupt Group
    IER = 1;
    // final enable timer to work
    CpuTimer0Regs.TCR.bit.TSS = 0;
}

__interrupt void INT_myCPUTIMER0_ISR(void)
{
    // TIMER0 interrupt function
    // TODO
    bsp_led5_toggle();
    // clear PIE ACK (mean interrupt mark bit)
    PieCtrlRegs.PIEACK.bit.ACK1 = 1;
}
