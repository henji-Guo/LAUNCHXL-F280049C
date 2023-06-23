/*
 * led.c
 *
 *  Created on: 2023Äê6ÔÂ23ÈÕ
 *      Author: GHJ
 */
#include "led.h"

void led4_init(void)
{
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0b00;
    GpioCtrlRegs.GPAGMUX2.bit.GPIO23 = 0b00;
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0b0;
    GpioCtrlRegs.GPADIR.bit.GPIO23 = 0b1;
    GpioCtrlRegs.GPAINV.bit.GPIO23 = 0b0;
    GpioCtrlRegs.GPACSEL3.bit.GPIO23 = 0x0;
}

void led4_on(void)
{
    GpioDataRegs.GPASET.bit.GPIO23 = 0b1;
}

void led4_off(void)
{
    GpioDataRegs.GPACLEAR.bit.GPIO23 = 0b1;
}


void led5_init(void)
{
    GPIO_setPinConfig(GPIO_34_GPIO34);
    GPIO_setPadConfig(34, GPIO_PIN_TYPE_PULLUP);
    GPIO_setDirectionMode(34, GPIO_DIR_MODE_OUT);
}

void led5_on(void)
{
    GPIO_writePin(34, 1);
}

void led5_off(void)
{
    GPIO_writePin(34, 0);
}
