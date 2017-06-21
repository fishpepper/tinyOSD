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
#include "macros.h"
#include "crc8.h"

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
    SERIAL_PROTOCOL_STATE_COMMAND,
    SERIAL_PROTOCOL_STATE_DATA,
    SERIAL_PROTOCOL_STATE_CHECKSUM,
} serial_protocol_state;

#define SERIAL_PROTOCOL_HEADER 0x80

#define SERIAL_PROTOCOL_FRAME_BUFFER_SIZE 64

void serial_init(void) {
    debug_function_call();

    serial_init_rcc();
    serial_init_gpio();
    serial_init_uart();

    serial_protocol_state = SERIAL_PROTOCOL_STATE_IDLE;
}

void serial_init_rcc(void) {
    debug_function_call();

    // crc module clock enable
    rcc_periph_clock_enable(RCC_CRC);

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
    usart_set_stopbits(SERIAL_UART, USART_CR2_STOPBITS_1);
    usart_set_mode(SERIAL_UART, USART_MODE_TX_RX);
    usart_set_flow_control(SERIAL_UART, USART_FLOWCONTROL_NONE);

    // disable overrun detection?
    #if SERIAL_OVERRUN_DETECTION_DISABLED
    USART_CR3(SERIAL_UART) |= USART_CR3_OVRDIS;
    #endif

    // finally enable the USART
    usart_enable(SERIAL_UART);
}

static volatile uint8_t  serial_protocol_frame_crc;
static volatile uint8_t  serial_protocol_frame_len;
static volatile uint8_t  serial_protocol_frame_command;
static volatile uint8_t  serial_protocol_frame_data_todo;
static volatile uint8_t  serial_protocol_frame_data[SERIAL_PROTOCOL_FRAME_BUFFER_SIZE];
static volatile uint8_t *serial_protocol_frame_data_ptr;

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
    CRC8_UPDATE(serial_protocol_frame_crc, rx);

    switch (serial_protocol_state) {
        default:
        case(SERIAL_PROTOCOL_STATE_IDLE):
            // look for SOF
            if (rx == SERIAL_PROTOCOL_HEADER) {
                // new frame
                 serial_protocol_state = SERIAL_PROTOCOL_STATE_LEN;
                 CRC8_INIT(serial_protocol_frame_crc, 0);
                 CRC8_UPDATE(serial_protocol_frame_crc, rx);
            }
            break;

        case(SERIAL_PROTOCOL_STATE_LEN):
            // fetch len
            serial_protocol_frame_len = rx;

            if (serial_protocol_frame_len == 0) {
                // invalid, ignore
                serial_protocol_state = SERIAL_PROTOCOL_STATE_IDLE;
                break;
            } else if (serial_protocol_frame_len >= SERIAL_PROTOCOL_FRAME_BUFFER_SIZE) {
                // invalid as well
                if (rx == SERIAL_PROTOCOL_HEADER) {
                    // this could be a new header
                    CRC8_INIT(serial_protocol_frame_crc, 0);
                    CRC8_UPDATE(serial_protocol_frame_crc, rx);
                    // stay in this state and abort
                    break;
                } else {
                    // abort
                    serial_protocol_state = SERIAL_PROTOCOL_STATE_IDLE;
                    break;
                }
            } else {
                // valid data
                serial_protocol_state = SERIAL_PROTOCOL_STATE_COMMAND;
            }
            break;

        case(SERIAL_PROTOCOL_STATE_COMMAND):
            // fetch data blob
            serial_protocol_frame_command = rx;

            // set data ptr
            serial_protocol_frame_data_todo = serial_protocol_frame_len;
            serial_protocol_frame_data_ptr  = &serial_protocol_frame_data[0];

            // update state
            serial_protocol_state = SERIAL_PROTOCOL_STATE_DATA;
            break;

        case(SERIAL_PROTOCOL_STATE_DATA):
            *serial_protocol_frame_data_ptr++ = rx;
            serial_protocol_frame_data_todo--;

            // len testing:
            if (!(serial_protocol_frame_data_todo)) {
                // finished with this buffer
                serial_protocol_state = SERIAL_PROTOCOL_STATE_CHECKSUM;
            }
            break;

        case(SERIAL_PROTOCOL_STATE_CHECKSUM):
            if (serial_protocol_frame_crc == 0) {
                // valid data!
                // [0]        = command
                // [1..(n-1)] = data
                switch(serial_protocol_frame_command) {
                    default:
                        break;

                    case(0x00):
                    case(0x01):
                    case(0x02):
                    case(0x03):
                        // thos cammands write to page 0, 1, 2, or 3
                        {
                            if (serial_protocol_frame_len >= 2) {
                                // at least 1 byte command + 1 byte addressoffset + 1 data byte is required
                                uint16_t p = (serial_protocol_frame_command << 8) | (serial_protocol_frame_data[0] & 0xFF);
                                uint8_t  len = serial_protocol_frame_len - 1;

                                // make sure not to exceed the buffer
                                if ((p + len) < VIDEO_CHAR_BUFFER_WIDTH * VIDEO_CHAR_BUFFER_HEIGHT){
                                    // safe to process
                                    uint8_t *data_buf_ptr = (uint8_t*)&serial_protocol_frame_data[1];
                                    uint8_t *char_buf_ptr = &video_char_buffer[0][0];

                                    // add adress offset:
                                    char_buf_ptr += p;

                                    // set all bytes
                                    while (len--) {
                                        *char_buf_ptr++ = *data_buf_ptr++;
                                    }
                                }
                            }
                        }
                        break;

                    case(0x07):
                        // set AETR stick pos
                        video_stick_data[0] = serial_protocol_frame_data[0];
                        video_stick_data[1] = serial_protocol_frame_data[1];
                        video_stick_data[2] = serial_protocol_frame_data[2];
                        video_stick_data[3] = serial_protocol_frame_data[3];
                        video_armed_state =  serial_protocol_frame_data[4];
                        video_uart_checksum_err = video_armed_state;
                        break;
                }
            }else{
                video_uart_checksum_err++;
                if (rx == SERIAL_PROTOCOL_HEADER) {
                    serial_protocol_state = SERIAL_PROTOCOL_STATE_LEN;
                    CRC8_INIT(serial_protocol_frame_crc, 0);
                    CRC8_UPDATE(serial_protocol_frame_crc, rx);
                    break;
                }
            }

            // done!
            serial_protocol_state = SERIAL_PROTOCOL_STATE_IDLE;
            break;
    }
}
