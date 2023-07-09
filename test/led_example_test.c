/*
 * led_example_test.c
 *
 *  Created on: 2023年6月23日
 *      Author: GHJ
 */
#include "test.h"
#include "bsp_led.h"

void led_test(void)
{
    bsp_led4_init();
    bsp_led5_init();

    while(1){
        bsp_led4_on();
        bsp_led5_off();
        DEVICE_DELAY_US(800000);
        bsp_led4_off();
        bsp_led5_on();
        DEVICE_DELAY_US(800000);
    }
}
