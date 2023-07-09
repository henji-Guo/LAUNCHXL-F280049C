/*
 * sci_example_test.c
 *
 *  Created on: 2023年6月23日
 *      Author: GHJ
 */

#include "test.h"
#include "device.h"
#include "string.h"

void sci_a_test(void)
{
    char *sci_tx_buffer = "Hello LaunchPad TMS320F280049C !!!\r\n";
    SCI_writeCharArray(SCIA_BASE,(uint16_t*)sci_tx_buffer, strlen(sci_tx_buffer));

}
