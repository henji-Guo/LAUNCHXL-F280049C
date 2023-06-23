/*
 * led_example_test.c
 *
 *  Created on: 2023Äê6ÔÂ23ÈÕ
 *      Author: GHJ
 */
#include "test.h"
#include "led.h"

void led_test(void)
{
    led4_init();
    led5_init();

    while(1){
        led4_on();
        led5_off();
        DEVICE_DELAY_US(800000);
        led4_off();
        led5_on();
        DEVICE_DELAY_US(800000);
    }
}
