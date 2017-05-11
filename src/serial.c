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
#include "debug.h"
#include "main.h"
#include "config.h"
#include "video.h"
#include "led.h"

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

static void serial_init_gpio(void);
static void serial_init_rcc(void);
static void serial_init_uart(void);


enum serial_protocol_state_t {
    SERIAL_PROTOCOL_STATE_IDLE = 0,
    SERIAL_PROTOCOL_STATE_LEN,
    SERIAL_PROTOCOL_STATE_ADDRESS,
    SERIAL_PROTOCOL_STATE_DATA,
    SERIAL_PROTOCOL_STATE_CHECKSUM,
} serial_protocol_state;

#define SERIAL_PROTOCOL_HEADER 0x80


void serial_init(void) {
    debug_function_call();

    serial_init_rcc();
    serial_init_gpio();
    serial_init_uart();

    serial_protocol_state = SERIAL_PROTOCOL_STATE_IDLE;
}

void serial_init_rcc(void) {
    debug_function_call();

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(SERIAL_GPIO));
    rcc_periph_clock_enable(SERIAL_UART_RCC);
}

void serial_init_gpio(void) {
    debug_function_call();

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
    debug_function_call();

    // setup USART parameters
    usart_set_baudrate(SERIAL_UART, SERIAL_UART_BAUDRATE);
    usart_set_databits(SERIAL_UART, 8);
    usart_set_parity(SERIAL_UART, USART_PARITY_NONE);
    usart_set_stopbits(SERIAL_UART, USART_CR2_STOP_1_0BIT);
    usart_set_mode(SERIAL_UART, USART_MODE_TX_RX);
    usart_set_flow_control(SERIAL_UART, USART_FLOWCONTROL_NONE);

    // disable overrun detection?
    #if SERIAL_OVERRUN_DETECTION_DISABLED
    USART_CR3(SERIAL_UART) |= USART_CR3_OVRDIS;
    #endif

    // finally enable the USART
    usart_enable(SERIAL_UART);
}

static volatile uint8_t serial_protocol_frame_crc;
static volatile uint8_t serial_protocol_frame_len;
static volatile uint8_t serial_protocol_frame_address;
static volatile uint8_t serial_protocol_frame_data_count;
static volatile uint8_t serial_protocol_frame_data[256];

// process serial datastream
// this uses a very simple protocol:
// [HEADER] LEN ADDR <n DATA> [CSUM]
void serial_process(void) {
    // process incoming serial data, this is called
    // after rendering a video line, make sure
    // not to do big/time consuming stuff here

#if !SERIAL_OVERRUN_DETECTION_DISABLED
    // any bytes lost (overrun)
    if ((USART_ISR(SERIAL_UART) & USART_ISR_ORE) != 0) {
        // overrun detected
        video_uart_overrun++;
        USART_ICR(SERIAL_UART) |= USART_ICR_ORECF;
    }
#endif

    // incoming data?
    if ((USART_ISR(SERIAL_UART) & USART_ISR_RXNE) == 0) {
        // nope, no new data
        return;
    }

    // ok, new data available, read it
    uint8_t rx = USART_RDR(SERIAL_UART);

    // update crc
    serial_protocol_frame_crc ^= rx;

    switch (serial_protocol_state) {
        default:
        case(SERIAL_PROTOCOL_STATE_IDLE):
            // look for SOF
            if (rx == SERIAL_PROTOCOL_HEADER) {
                // new frame
                serial_protocol_state = SERIAL_PROTOCOL_STATE_LEN;
                serial_protocol_frame_crc = rx;
            }
            break;

        case(SERIAL_PROTOCOL_STATE_LEN):
            // fetch len
            serial_protocol_frame_len = rx;
            serial_protocol_state = SERIAL_PROTOCOL_STATE_ADDRESS;
            break;

        case(SERIAL_PROTOCOL_STATE_ADDRESS):
            // fetch address
            serial_protocol_frame_address = rx;
            serial_protocol_frame_data_count = 0;
            serial_protocol_state = SERIAL_PROTOCOL_STATE_DATA;
            break;

        case(SERIAL_PROTOCOL_STATE_DATA):
            // fetch n data bytes
            serial_protocol_frame_data[serial_protocol_frame_data_count++] = rx;
            serial_protocol_frame_len--;

            if (serial_protocol_frame_len == 0) {
                // finished, next byte is checksum
                serial_protocol_state = SERIAL_PROTOCOL_STATE_CHECKSUM;
            }
            break;

        case(SERIAL_PROTOCOL_STATE_CHECKSUM):
            if (serial_protocol_frame_crc == 0) {
                // valid data!
                switch(serial_protocol_frame_address) {
                    default:
                        break;
                    case(0x05):  // DMAH
                        video_char_buffer_write_ptr = (serial_protocol_frame_data[0] << 8) | (video_char_buffer_write_ptr & 0xFF);
                        break;
                    case(0x06):  // DMAL
                        video_char_buffer_write_ptr = (video_char_buffer_write_ptr & 0xFF00) | serial_protocol_frame_data[0];
                        break;
                    case(0x07):
                        if (video_char_buffer_write_ptr < VIDEO_CHAR_BUFFER_WIDTH * VIDEO_CHAR_BUFFER_HEIGHT) {
                            uint8_t *ptr = (&video_char_buffer[0][0]) + video_char_buffer_write_ptr;
                            *ptr = serial_protocol_frame_data[0];
                        }
                        break;
                }
            }

            // done!
            serial_protocol_state = SERIAL_PROTOCOL_STATE_IDLE;
            break;
    }
}
