/*
 * iic_example_test.c
 *
 *  Created on: 2023Äê6ÔÂ23ÈÕ
 *      Author: GHJ
 */
#include "test.h"
#include "bsp_sw_iic.h"
#include "stdio.h"

#define as5600_address  0x36
#define as5600_read (as5600_address<<1|1)
#define as5600_write (as5600_address<<1)
#define raw_angle_address  0x0c
#define angle_address  0x0e
uint8_t angle[4];

void iic_test(void)
{

    bsp_iic_init();

    bsp_iic_start();
    bsp_iic_sendonebyte(as5600_write);
    if(bsp_iic_wait_ack() != ACK_OK)
        return;
    bsp_iic_sendonebyte(raw_angle_address);
    if(bsp_iic_wait_ack() != ACK_OK)
        return;
    bsp_iic_start();
    bsp_iic_sendonebyte(as5600_read);
    if(bsp_iic_wait_ack() != ACK_OK)
        return;
    angle[0] = bsp_iic_readonebyte();
    bsp_iic_ack();
    angle[1] = bsp_iic_readonebyte();
    bsp_iic_ack();
    angle[2] = bsp_iic_readonebyte();
    bsp_iic_ack();
    angle[3] = bsp_iic_readonebyte();
    bsp_iic_nack();
    bsp_iic_stop();

}
