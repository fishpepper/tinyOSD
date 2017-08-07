/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/ or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http:// www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#include "timeout.h"
#include "delay.h"
#include "led.h"
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>

static volatile __IO uint32_t timeout_100us;
static volatile __IO uint32_t timeout2_100us;
static volatile __IO uint32_t timeout_100us_delay;

void timeout_init(void) {
    // configure 0.1ms sys tick:
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(rcc_ahb_frequency / 10000);
    systick_interrupt_enable();
    systick_counter_enable();

    // clear counter so it starts right away
    STK_CVR = 0;

    // set prio
    nvic_set_priority(NVIC_SYSTICK_IRQ, NVIC_PRIO_SYSTICK);

    timeout_100us = 0;
    timeout2_100us = 0;
    timeout_100us_delay = 0;
}

void timeout_set_100us(__IO uint32_t hus) {
    timeout_100us = hus;
}

void timeout2_set_100us(__IO uint32_t hus) {
    timeout2_100us = hus;
}

uint8_t timeout_timed_out(void) {
    return(timeout_100us == 0);
}

uint8_t timeout2_timed_out(void) {
    return(timeout2_100us == 0);
}

void timeout2_delay_100us(uint16_t us) {
    timeout2_set_100us(us);
    while (!timeout2_timed_out()) {}
}


// seperate ms delay function
void timeout_delay_ms(uint32_t timeout) {
    timeout_100us_delay = 10*timeout;

    while (timeout_100us_delay > 0) {
    }
}

void sys_tick_handler(void) {
    if (timeout_100us != 0) {
        timeout_100us--;
    }
    if (timeout2_100us != 0) {
        timeout2_100us--;
    }
    if (timeout_100us_delay != 0) {
        timeout_100us_delay--;
    }
}

uint32_t timeout_time_remaining(void) {
    return timeout_100us/ 10;
}
