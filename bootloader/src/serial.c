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

#include "serial.h"
#include "main.h"
#include "config.h"
#include "timeout.h"
#include "led.h"
#include "macros.h"
#include "bootloader.h"

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void serial_init_gpio(void);
static void serial_init_rcc(void);
static void serial_init_uart(void);

void serial_init(void) {
    serial_init_rcc();
    serial_init_gpio();
    serial_init_uart();
}

void serial_init_rcc(void) {
    // crc module clock enable
    rcc_periph_clock_enable(RCC_CRC);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(SERIAL_GPIO));
    rcc_periph_clock_enable(SERIAL_UART_RCC);
}

void serial_init_gpio(void) {
    // set rx and tx to AF
    gpio_mode_setup(SERIAL_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, SERIAL_TX_PIN);
    gpio_mode_setup(SERIAL_GPIO, GPIO_MODE_AF, GPIO_PUPD_NONE, SERIAL_RX_PIN);

    // both are AF1
    gpio_set_af(SERIAL_GPIO, SERIAL_GPIO_AF, SERIAL_TX_PIN);
    gpio_set_af(SERIAL_GPIO, SERIAL_GPIO_AF, SERIAL_RX_PIN);

    // set uart tx pin as output
    gpio_set_output_options(SERIAL_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, SERIAL_TX_PIN);
}

void serial_init_uart(void) {
    // setup USART parameters
    usart_set_baudrate(SERIAL_UART, SERIAL_UART_BAUDRATE);
    usart_set_databits(SERIAL_UART, 8);
    usart_set_parity(SERIAL_UART, USART_PARITY_NONE);
    usart_set_stopbits(SERIAL_UART, USART_CR2_STOPBITS_1);
    usart_set_mode(SERIAL_UART, USART_MODE_TX_RX);
    usart_set_flow_control(SERIAL_UART, USART_FLOWCONTROL_NONE);

    // disable overrun detection?
    #if SERIAL_OVERRUN_DETECTION_DISABLED
    USART_CR3(SERIAL_UART) |= USART_CR3_OVRDIS;
    #endif  // SERIAL_OVERRUN_DETECTION_DISABLED

    // finally enable the USART
    usart_enable(SERIAL_UART);
}

uint8_t serial_getc(void) {
    // visual indicator
    led_toggle();

#if !SERIAL_OVERRUN_DETECTION_DISABLED
    // any bytes lost (overrun)
    if ((USART_ISR(SERIAL_UART) & USART_ISR_ORE) != 0) {
        // overrun detected
        USART_ICR(SERIAL_UART) |= USART_ICR_ORECF;
    }
#endif  // SERIAL_OVERRUN_DETECTION_DISABLED

    // wait for incoming data with timeout
    timeout_set(BOOTLOADER_TIMEOUT_MS);
    while ((USART_ISR(SERIAL_UART) & USART_ISR_RXNE) == 0) {
        // no new data, check for timeout
        if (timeout_timed_out()) {
            // timeout reached, try to jump to app
            // this will fail if the flash checksum is invalid
            bootloader_jump_to_app();
            return 0;
        }
    }

    // ok, new data available, read and return it
    uint8_t rx = USART_RDR(SERIAL_UART);
    return rx;
}

void serial_putc(uint8_t c) {
    usart_send_blocking(SERIAL_UART, c);
}
