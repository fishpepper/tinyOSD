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

#include "debug.h"
#include "delay.h"
#include "main.h"
#include "macros.h"
#include <stdint.h>
#include "config.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void debug_init_gpio(void);
static void debug_init_rcc(void);
static void debug_init_uart(void);

void debug_init(void) {
    debug_init_rcc();
    debug_init_gpio();
    debug_init_uart();

    debug_put_newline();
    //     ###################
    debug("###  tinyOSD    ###\n");
    debug(" (c) by fishpepper  \n\n");
    debug("debug: init done\n");
}

void debug_init_rcc(void) {
    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(DEBUG_GPIO));
    rcc_periph_clock_enable(DEBUG_UART_RCC);
}

void debug_init_gpio(void) {
    // set uart tx pin as output
    gpio_mode_setup(DEBUG_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, DEBUG_TX_PIN);
    gpio_set_output_options(DEBUG_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, DEBUG_TX_PIN);

    // setup USART TX pin as alternate function
    gpio_set_af(DEBUG_GPIO, DEBUG_GPIO_AF, DEBUG_TX_PIN);
}

void debug_init_uart(void) {
    // setup USART parameters
    usart_set_baudrate(DEBUG_UART, DEBUG_UART_BAUDRATE);
    usart_set_databits(DEBUG_UART, 8);
    usart_set_parity(DEBUG_UART, USART_PARITY_NONE);
    usart_set_stopbits(DEBUG_UART, USART_CR2_STOPBITS_1);
    usart_set_mode(DEBUG_UART, USART_MODE_TX);
    usart_set_flow_control(DEBUG_UART, USART_FLOWCONTROL_NONE);

    // finally enable the USART
    usart_enable(DEBUG_UART);
}


void debug_putc(uint8_t ch) {
    // add \r to newlines
    if (ch == '\n') debug_putc('\r');
    usart_send_blocking(DEBUG_UART, ch);
}

void debug_flush(void) {
}


void debug(char *data) {
    uint8_t c = (uint8_t)*data++;
    while (c) {
        debug_putc(c);
        c = *data++;
    }
}


// put hexadecimal number to debug out.
void debug_put_hex8(uint8_t val) {
    uint8_t lo = val&0x0F;
    uint8_t hi = val>>4;
    if (hi < 0x0A) {
        hi = '0' + hi;
    } else {
        hi = 'A' - 0x0A + hi;
    }

    if (lo < 0x0A) {
        lo = '0' + lo;
    } else {
        lo = 'A' - 0x0A + lo;
    }
    debug_putc(hi);
    debug_putc(lo);
}

// put 16bit hexadecimal number to debug out
void debug_put_hex16(uint16_t val) {
    debug_put_hex8(val>>8);
    debug_put_hex8(val & 0xFF);
}

// put 32bit hexadecimal number to debug out
void debug_put_hex32(uint32_t val) {
    debug_put_hex8(val>>24);
    debug_put_hex8(val>>16);
    debug_put_hex8(val>> 8);
    debug_put_hex8(val & 0xFF);
}

// output a signed 8-bit number to uart
void debug_put_int8(int8_t c) {
    uint8_t tmp;
    uint8_t mul;
    uint8_t l;
    uint8_t uint_s;

    if (c < 0) {
        debug_putc('-');
        uint_s = -c;
    } else {
        uint_s = c;
    }

    l = 0;
    for (mul = 100; mul > 0; mul = mul/ 10) {
        tmp = '0';
        while (uint_s >= mul) {
            uint_s -= mul;
            tmp++;
            l = 1;
        }
        if ((l == 0) && (tmp == '0') && (mul != 1)) {
            // dont print spacer
            // debug_putc(' ');
        } else {
            debug_putc(tmp);
        }
    }
}

// output an unsigned 8-bit number to uart
void debug_put_uint8(uint8_t c) {
    uint8_t tmp;
    uint8_t mul;
    uint8_t l;

    l = 0;
    for (mul = 100; mul >0 ; mul = mul/ 10) {
        tmp = '0';
        while (c >= mul) {
            c -= mul;
            tmp++;
            l = 1;
        }
        if ((l == 0) && (tmp == '0') && (mul != 1)) {
            // dont print spacer
            // debug_putc(' ');
        } else {
            debug_putc(tmp);
        }
    }
}

// output an unsigned 16-bit number to uart
void debug_put_uint16(uint16_t c) {
    uint8_t tmp;
    uint8_t l = 0;
    // loop unrolling is better(no int16 arithmetic)
    /*for (mul = 10000; mul>0; mul = mul/ 10) {
        uint16_t mul;

        l = 0;
                tmp = '0';
                while (c>=mul) {
                        c -= mul;
                        tmp++;
                        l = 1;
                }
                if ((l == 0) && (tmp == '0') && (mul!=1)) {
                        // debug_putc(' ');
                } else {
                        debug_putc(tmp);
                }
        }*/
    tmp = 0;
    while (c >= 10000L) {
        c -= 10000L;
        tmp++;
        l = 1;
    }
    if (tmp != 0) debug_putc('0' + tmp);

    tmp = 0;
    while (c >= 1000L) {
        c -= 1000L;
        tmp++;
        l = 1;
    }
    if (l || (tmp != 0)) debug_putc('0' + tmp);

    tmp = 0;
    while (c >= 100) {
        c -= 100;
        tmp++;
        l = 1;
    }
    if (l || (tmp != 0)) debug_putc('0' + tmp);

    tmp = 0;
    while (c >= 10) {
        c -= 10;
        tmp++;
        l = 1;
    }
    if (l || (tmp != 0)) debug_putc('0' + tmp);

    debug_putc('0' + (uint8_t)c);
}

void debug_put_fixed2(uint16_t c) {
    uint8_t tmp;
    uint8_t l = 0;
    tmp = 0;
    while (c >= 10000L) {
        c -= 10000L;
        tmp++;
        l = 1;
    }
    if (tmp != 0) debug_putc('0' + tmp);

    tmp = 0;
    while (c >= 1000L) {
        c -= 1000L;
        tmp++;
        l = 1;
    }
    if (l || (tmp != 0)) debug_putc('0' + tmp);

    tmp = 0;
    while (c >= 100) {
        c -= 100;
        tmp++;
        l = 1;
    }
    if (l || (tmp != 0)) debug_putc('0' + tmp);

    debug_putc('.');

    tmp = 0;
    while (c >= 10) {
        c -= 10;
        tmp++;
        l = 1;
    }
    if (l || (tmp != 0)) debug_putc('0' + tmp);

    debug_putc('0' + (uint8_t)c);
}

void debug_put_fixed1p3(uint16_t c) {
    uint8_t tmp;
    tmp = 0;
    while (c >= 1000L) {
        c -= 1000L;
        tmp++;
    }
    debug_putc('0' + tmp);

    debug_putc('.');

    tmp = 0;
    while (c >= 100) {
        c -= 100;
        tmp++;
    }
    debug_putc('0' + tmp);

    tmp = 0;
    while (c >= 10) {
        c -= 10;
        tmp++;
    }
    debug_putc('0' + tmp);

    debug_putc('0' + (uint8_t)c);
}

void debug_put_newline(void) {
    debug_putc('\n');
}

