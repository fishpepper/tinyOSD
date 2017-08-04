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

static void serial_protocol_process_data(void);
static void serial_protocol_init(void);

#define PROTOCOL_STATE_IDLE       0x00
#define PROTOCOL_STATE_DEVICE_CMD 0x10
#define PROTOCOL_STATE_COMMAND    0x20
#define PROTOCOL_STATE_DATA_BYTE  0x30
#define PROTOCOL_STATE_DATA       0x40
#define PROTOCOL_STATE_CRC        0xF0

void serial_init(void) {
    debug_function_call();

    serial_init_rcc();
    serial_init_gpio();
    serial_init_uart();

    serial_protocol_init();
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

static volatile uint8_t  serial_protocol_crc;

static uint8_t serial_protocol_data[PROTOCOL_FRAME_MAX_LEN];
static uint8_t serial_protocol_data_index;
static uint8_t serial_protocol_data_count;
static uint8_t serial_protocol_state;

static void serial_protocol_init(void) {
    serial_protocol_state = PROTOCOL_STATE_IDLE;
}

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

    if (serial_protocol_data_index < PROTOCOL_FRAME_MAX_LEN){
        // store data
        serial_protocol_data[serial_protocol_data_index++] = rx;
    }

    // update crc
    CRC8_UPDATE(serial_protocol_crc, rx);

static uint8_t dbg=0;
    switch (serial_protocol_state) {
        default:
        case(PROTOCOL_STATE_IDLE):
            // look for SOF
            if (rx == PROTOCOL_HEADER) {
                // new frame
                serial_protocol_state = PROTOCOL_STATE_DEVICE_CMD;
                CRC8_INIT(serial_protocol_crc, 0);
                CRC8_UPDATE(serial_protocol_crc, rx);
//                serial_protocol_crc = CRC8_FROM_HEADER;
                serial_protocol_data_index = 0;
            }
            break;

        case(PROTOCOL_STATE_DEVICE_CMD):
            if (((rx >> 4) & 0xF) == PROTOCOL_DEVICE_OSD) {
                // valid device id
                serial_protocol_state = PROTOCOL_STATE_COMMAND + (rx & 0x0F);
            } else {
                // invalid device type -> abort
                if (rx == PROTOCOL_HEADER) {
                    serial_protocol_crc = CRC8_FROM_HEADER;
                    serial_protocol_data_index = 0;
                } else {
                    serial_protocol_state = PROTOCOL_STATE_IDLE;
                }
            }
            break;

        // 2 byte commands:
        case(PROTOCOL_STATE_COMMAND + PROTOCOL_CMD_SET_REGISTER):
            // [HEADER] [DEV+CMD] [REGISTER:8] [VALUE:8] [CRC:8]
            serial_protocol_data_count = 2-1;
            serial_protocol_state = PROTOCOL_STATE_DATA;
            break;

        // 3 byte commands:
        case(PROTOCOL_STATE_COMMAND + PROTOCOL_CMD_WRITE):
            // [HEADER] [DEV+CMD] [X:8] [Y:8] [VALUE:8] [CRC:8]
            serial_protocol_data_count = 3-1;
            serial_protocol_state = PROTOCOL_STATE_DATA;
            break;

        // 5 byte commands:
        case(PROTOCOL_STATE_COMMAND + PROTOCOL_CMD_FILL_REGION):
            // [HEADER] [DEV+CMD] [X:8] [Y:8] [WIDTH:8] [HEIGHT:8] [VALUE:8] [CRC:8]
            serial_protocol_data_count = 5-1;
            serial_protocol_state = PROTOCOL_STATE_DATA;
            break;

        // variable length commands
        case(PROTOCOL_STATE_COMMAND + PROTOCOL_CMD_WRITE_BUFFER_H):
        case(PROTOCOL_STATE_COMMAND + PROTOCOL_CMD_WRITE_BUFFER_V):
        case(PROTOCOL_STATE_COMMAND + PROTOCOL_CMD_SPECIAL):
            // [0x80] [DEV+CMD] [LEN] [LEN*DATA] [CRC:8] 
            if ((rx == 0) || (rx > (PROTOCOL_FRAME_MAX_LEN-2))) {
                // not allowed
                serial_protocol_state = PROTOCOL_STATE_IDLE;
            } else {
                serial_protocol_data_count = rx;
                serial_protocol_state = PROTOCOL_STATE_DATA;
            }
            break;

        // slurp in data
        case(PROTOCOL_STATE_DATA):
            serial_protocol_data_count--;
            if (!serial_protocol_data_count) {
                serial_protocol_state = PROTOCOL_STATE_CRC;
            }
            break;

        // finish data receiption
        case(PROTOCOL_STATE_CRC):
            // test crc
            if (serial_protocol_crc == 0) {
                // valid data!
                // [0]        = command
                // [1..(n-1)] = data
                serial_protocol_process_data();
                serial_protocol_state = PROTOCOL_STATE_IDLE;
            } else {
                // crc error! 
                video_uart_checksum_err++;
                if (rx == PROTOCOL_HEADER) {
                    serial_protocol_crc = CRC8_FROM_HEADER;
                    serial_protocol_data_index = 0;
                } else {
                    serial_protocol_state = PROTOCOL_STATE_IDLE;
                }
            }
            break;
    }
}

void serial_protocol_process_data(void) {
    uint8_t len = 0;
    // fetch pointer to data
    uint8_t *data = serial_protocol_data;

    // extract command
    uint8_t cmd = (*data++) & 0x0F;

    // variable or fixed length?
    if (cmd & 0x08) {
        // variable length command
        len = *data++;
    }

    switch(cmd) {
        default:
            // invalid, ignore
            break;
     
        case(PROTOCOL_CMD_SET_REGISTER):
        {
            // set register
            uint8_t reg = *data++;
            uint8_t value = *data;
            break;
        }

        case(PROTOCOL_CMD_FILL_REGION):
        {
            // fill screen region
            uint8_t x = *data++;
            uint8_t y = *data++;
            uint8_t width = *data++;
            uint8_t height = *data++;
            uint8_t value = *data;
            /*while (height--){
                if (y >= VIDEO_CHAR_BUFFER_HEIGHT) return;
                uint8_t todo = width;
                while (todo--) {
                    if (x >= VIDEO_CHAR_BUFFER_WIDTH) return;
                    video_char_buffer[y][x] = value;
                    x++;
                }
                y++;
            }*/
            break;
        }

        case(PROTOCOL_CMD_WRITE):
        {
            // set single character
            uint8_t x = *data++;
            uint8_t y = *data++;
            uint8_t value = *data;
            if (x >= VIDEO_CHAR_BUFFER_WIDTH) return;
            if (y >= VIDEO_CHAR_BUFFER_HEIGHT) return;
            video_char_buffer[y][x] = value;
            break;
        }

        case(PROTOCOL_CMD_WRITE_BUFFER_H):
        {
            // check for invalid length
            if (len <= 2) return;
            // write len bytes to framebuffer
            uint8_t x = *data++;
            uint8_t y = *data++;
            // subtract length 
            len -= 2;
            // check for valid x start
            if (x >= VIDEO_CHAR_BUFFER_WIDTH) return;
            // fill
            while(len){
                if (y >= VIDEO_CHAR_BUFFER_HEIGHT) return;
                video_char_buffer[y][x] = *data++;
                // prepare for next write position
                x++;
                // wrap to next line on x overflow
                if (x >= VIDEO_CHAR_BUFFER_WIDTH) y++;
                // keep track of bytes to write
                len--;
            }
            break;
        }

        case(PROTOCOL_CMD_WRITE_BUFFER_V):
        {
            // check for invalid length
            if (len <= 2) return;
            // write len bytes to framebuffer
            uint8_t x = *data++;
            uint8_t y = *data++;
            // subtract lenght
            if (len <= 2) return;
            len -= 2;
            // check for valid x start
            if (y >= VIDEO_CHAR_BUFFER_HEIGHT) return;
            // fill
            while(len){
                if (x >= VIDEO_CHAR_BUFFER_WIDTH) return;
                video_char_buffer[y][x] = *data++;
                // prepare for next write position
                y++;
                // wrap to next line on x overflow
                if (y >= VIDEO_CHAR_BUFFER_HEIGHT) x++;
                // keep track of bytes to write
                len--;
            }   
            break;
        }
    
        case(PROTOCOL_CMD_SPECIAL):
        {
            // special command
            uint8_t subcmd = *data++;
            if (subcmd == PROTOCOL_CMD_SPECIAL_SUBCMD_STICKSTATUS) {
                // stick input [A] [E] [T] [R] [FC_STATUS]
                video_stick_data[0] = *data++;
                video_stick_data[1] = *data++;
                video_stick_data[2] = *data++;
                video_stick_data[3] = *data++;
                video_armed_state =  *data++;
            }
            break;
         }
    }

}


/*
                switch(serial_protocol_frame_command) {
                    default:
                        break;

                    case(TINYOSD_COMMAND_SET_STATUS):
                        // SET STATUS 
                        break;

                    case(TINYOSD_COMMAND_FILL_SCREEN):
                        // fill screen with given value
                        for (uint8_t y=0; y<VIDEO_CHAR_BUFFER_HEIGHT; y++) {
                            for (uint8_t x=0; x<VIDEO_CHAR_BUFFER_WIDTH; x++) {
                                video_char_buffer[y][x] = serial_protocol_frame_data[0];
                            }
                        }
                        break;
                    
                    case(TINYOSD_COMMAND_WRITE_PAGE_0):
                    case(TINYOSD_COMMAND_WRITE_PAGE_1):
                    case(TINYOSD_COMMAND_WRITE_PAGE_2):
                    case(TINYOSD_COMMAND_WRITE_PAGE_3):
                        // thos cammands write to page 0, 1, 2, or 3
                        {
                            if (serial_protocol_frame_len >= 2) {
                                // at least 1 byte command + 1 byte addressoffset + 1 data byte is required
                                uint16_t p = ((serial_protocol_frame_command - TINYOSD_COMMAND_WRITE_PAGE_0) << 8) | (serial_protocol_frame_data[0] & 0xFF);
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

                    case(TINYOSD_COMMAND_WRITE_STICKDATA):
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
*/
