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
#include "rtc6705.h"
#include "video_io.h"

#include <stdint.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencmsis/core_cm3.h>

static uint16_t osd_register[OPENTCO_MAX_REGISTER];
static uint16_t vtx_register[OPENTCO_MAX_REGISTER];
const uint16_t *vtx_enabled_features = &osd_register[OPENTCO_OSD_REGISTER_STATUS];

static void serial_init_gpio(void);
static void serial_init_rcc(void);
static void serial_init_uart(void);
static void serial_init_interrupts(void);

static void serial_protocol_init(void);

static void serial_protocol_process_osd_data(void);
static void serial_protocol_write_osd_register(uint8_t reg, uint16_t value);
static void serial_protocol_read_osd_register(uint8_t reg);

static void serial_protocol_process_vtx_data(void);
static void serial_protocol_write_vtx_register(uint8_t reg, uint16_t value);
static void serial_protocol_read_vtx_register(uint8_t reg);
//static void serial_putc(char ch);

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
    serial_init_interrupts();

    serial_protocol_init();
}

static void serial_init_rcc(void) {
    debug_function_call();

    // crc module clock enable
    rcc_periph_clock_enable(RCC_CRC);

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(SERIAL_GPIO));
    rcc_periph_clock_enable(SERIAL_UART_RCC);
}

static void serial_init_gpio(void) {
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

static void serial_init_uart(void) {
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

static void serial_init_interrupts(void) {
    // enable irq
    nvic_enable_irq(SERIAL_UART_IRQN);
    nvic_set_priority(SERIAL_UART_IRQN, NVIC_PRIO_USART);
}

static volatile uint8_t  serial_protocol_crc;

static uint8_t serial_protocol_data[OPENTCO_MAX_FRAME_LENGTH];
static uint8_t serial_protocol_data_index;
static uint8_t serial_protocol_data_count;
static uint8_t serial_protocol_state;

static void serial_protocol_init(void) {
    // set supported features
    osd_register[OPENTCO_OSD_REGISTER_SUPPORTED_FEATURES] =
            OPENTCO_OSD_FEATURE_ENABLE |
            OPENTCO_OSD_FEATURE_INVERT |
            OPENTCO_OSD_FEATURE_BRIGHTNESS |
            OPENTCO_OSD_FEATURE_RENDER_LOGO |
            OPENTCO_OSD_FEATURE_RENDER_PILOTLOGO |
            OPENTCO_OSD_FEATURE_RENDER_STICKS |
            OPENTCO_OSD_FEATURE_RENDER_SPECTRUM |
            OPENTCO_OSD_FEATURE_RENDER_CROSSHAIR;

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

    led_toggle();

    // ok, new data available, read it
    uint8_t rx = USART_RDR(SERIAL_UART);

    if (serial_protocol_data_index < OPENTCO_MAX_FRAME_LENGTH){
        // store data
        serial_protocol_data[serial_protocol_data_index++] = rx;
    }

    // update crc
    CRC8_UPDATE(serial_protocol_crc, rx);

    switch (serial_protocol_state) {
        default:
        case(PROTOCOL_STATE_IDLE):
            // look for SOF
            if (rx == OPENTCO_PROTOCOL_HEADER) {
                // new frame
                serial_protocol_state = PROTOCOL_STATE_DEVICE_CMD;
                CRC8_INIT(serial_protocol_crc, 0);
                CRC8_UPDATE(serial_protocol_crc, rx);
//                serial_protocol_crc = CRC8_FROM_HEADER;
                serial_protocol_data_index = 0;
            }
            break;

        case(PROTOCOL_STATE_DEVICE_CMD):
            if ((((rx >> 4) & 0xF) == OPENTCO_DEVICE_OSD) ||
                (((rx >> 4) & 0xF) == OPENTCO_DEVICE_VTX) ) {
                // valid device id
                serial_protocol_state = PROTOCOL_STATE_COMMAND + (rx & 0x0F);
            } else {
                // invalid device type -> abort
                if (rx == OPENTCO_PROTOCOL_HEADER) {
                    serial_protocol_crc = OPENTCO_CRC8_FROM_HEADER;
                    serial_protocol_data_index = 0;
                } else {
                    serial_protocol_state = PROTOCOL_STATE_IDLE;
                }
            }
            break;

        // 3 byte commands:
        case(PROTOCOL_STATE_COMMAND + OPENTCO_OSD_COMMAND_REGISTER_ACCESS):
            // [HEADER] [DEV+CMD] [REGISTER:8] [VALUE_LO:8] [VALUE_HI:8] [CRC:8]
        case(PROTOCOL_STATE_COMMAND + OPENTCO_OSD_COMMAND_WRITE):
            // [HEADER] [DEV+CMD] [X:8] [Y:8] [VALUE:8] [CRC:8]
            serial_protocol_data_count = 3-1;
            serial_protocol_state = PROTOCOL_STATE_DATA;
            break;

        // 5 byte commands:
        case(PROTOCOL_STATE_COMMAND + OPENTCO_OSD_COMMAND_FILL_REGION):
            // [HEADER] [DEV+CMD] [X:8] [Y:8] [WIDTH:8] [HEIGHT:8] [VALUE:8] [CRC:8]
            serial_protocol_data_count = 5-1;
            serial_protocol_state = PROTOCOL_STATE_DATA;
            break;

        // variable length commands
        case(PROTOCOL_STATE_COMMAND + OPENTCO_OSD_COMMAND_WRITE_BUFFER_H):
        case(PROTOCOL_STATE_COMMAND + OPENTCO_OSD_COMMAND_WRITE_BUFFER_V):
        case(PROTOCOL_STATE_COMMAND + OPENTCO_OSD_COMMAND_SPECIAL):
            // [0x80] [DEV+CMD] [LEN] [LEN*DATA] [CRC:8] 
            if ((rx == 0) || (rx > (OPENTCO_MAX_FRAME_LENGTH-2))) {
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
                uint8_t dev = serial_protocol_data[0] >> 4;
                if (dev == OPENTCO_DEVICE_OSD) {
                    serial_protocol_process_osd_data();
                } else if (dev == OPENTCO_DEVICE_VTX) {
                    serial_protocol_process_vtx_data();
                }
                serial_protocol_state = PROTOCOL_STATE_IDLE;
            } else {
                // crc error! 
                video_uart_checksum_err++;
                if (rx == OPENTCO_PROTOCOL_HEADER) {
                    serial_protocol_crc = OPENTCO_CRC8_FROM_HEADER;
                    serial_protocol_data_index = 0;
                } else {
                    serial_protocol_state = PROTOCOL_STATE_IDLE;
                }
            }
            break;
    }
}


static void serial_protocol_process_osd_data(void) {
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
     
        case(OPENTCO_OSD_COMMAND_REGISTER_ACCESS):
        {
            // register access
            uint8_t reg = *data++;
            uint8_t value_lo = *data++;
            uint8_t value_hi = *data++;
            uint16_t value = (value_hi << 8) | value_lo;

            uint8_t read = reg & OPENTCO_REGISTER_ACCESS_MODE_READ;
            reg = reg & ~(OPENTCO_REGISTER_ACCESS_MODE_READ);

            if (read) {
                // read request
                serial_protocol_read_osd_register(reg);
            } else {
                // write
                serial_protocol_write_osd_register(reg, value);
            }
            break;
        }

        case(OPENTCO_OSD_COMMAND_FILL_REGION):
        {
            // fill screen region
            uint8_t xs = *data++;
            uint8_t ys = *data++;
            uint8_t width = *data++;
            uint8_t height = *data++;
            uint8_t c = *data;

            if (ys >= VIDEO_CHAR_BUFFER_HEIGHT) return;
            if (xs >= VIDEO_CHAR_BUFFER_WIDTH) return;

            for(uint8_t y = ys; y < VIDEO_CHAR_BUFFER_HEIGHT; y++){
                if (height == 0) break;
                height--;
                uint8_t w = width;
                for(uint8_t x = xs; x < VIDEO_CHAR_BUFFER_WIDTH; x++){
                    if (w == 0) break;
                    w--;
                    // ok, inside of safe region!
                    video_char_buffer[y][x] = c;
                }
            }
            break;
        }

        case(OPENTCO_OSD_COMMAND_WRITE):
        {
            // set single character
            uint8_t x = *data++;
            uint8_t y = *data++;
            uint8_t c = *data;
            if (x >= VIDEO_CHAR_BUFFER_WIDTH) return;
            if (y >= VIDEO_CHAR_BUFFER_HEIGHT) return;
            video_char_buffer[y][x] = c;
            break;
        }

        case(OPENTCO_OSD_COMMAND_WRITE_BUFFER_H):
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

        case(OPENTCO_OSD_COMMAND_WRITE_BUFFER_V):
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
    
        case(OPENTCO_OSD_COMMAND_SPECIAL):
        {
            // special command
            uint8_t subcmd = *data++;
            if (subcmd == OPENTCO_OSD_COMMAND_SPECIAL_SUB_STICKSTATUS) {
                // stick input [A] [E] [T] [R] [FC_STATUS]
                video_stick_data[0] = *data++;
                video_stick_data[1] = *data++;
                video_stick_data[2] = *data++;
                video_stick_data[3] = *data++;
                video_armed_state =  *data;
            } else if (subcmd == OPENTCO_OSD_COMMAND_SPECIAL_SUB_SPECTRUM) {
                uint8_t axis = *data++;
                for(uint32_t bin=0; bin < VIDEO_RENDER_SPECTRUM_BINS; bin++) {
                    // spectrum needs to be 0..127
                    video_spectrum_buffer[bin] = (*data++) / 2;
                }
            }
            break;
         }
    }
}


static void serial_protocol_process_vtx_data(void) {
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

        case(OPENTCO_VTX_COMMAND_REGISTER_ACCESS):
        {
            // register access
            uint8_t reg = *data++;
            uint8_t value_lo = *data++;
            uint8_t value_hi = *data++;
            uint16_t value = (value_hi << 8) | value_lo;

            uint8_t read = reg & OPENTCO_REGISTER_ACCESS_MODE_READ;
            reg = reg & ~(OPENTCO_REGISTER_ACCESS_MODE_READ);

            if (read) {
                // read request
                serial_protocol_read_vtx_register(reg);
            } else {
                // write
                serial_protocol_write_vtx_register(reg, value);
            }
            break;
        }
    }
}

void serial_protocol_write_osd_register(uint8_t reg, uint16_t value) {
    uint16_t tmp;

    // check register address
    if (reg > OPENTCO_MAX_REGISTER) return;

    switch(reg) {
        default:
            break;

        case (OPENTCO_OSD_REGISTER_VIDEO_FORMAT):
        case (OPENTCO_OSD_REGISTER_STATUS):
            //FIXME: ADD HANDLER
            break;

        case (OPENTCO_OSD_REGISTER_SUPPORTED_FEATURES):
            // READONLY -> RETURN before reg write!
            return;

        // brightness black, allow 0...500 mV
        case (OPENTCO_OSD_REGISTER_BRIGHTNESS_BLACK):
            // input is 0..100
            tmp = 5 * value;
            video_io_set_level_mv(BLACK, tmp);
            break;

        // brightness white, allow 500..1200 mV for black
        case (OPENTCO_OSD_REGISTER_BRIGHTNESS_WHITE):
            // input is 0..100
            tmp = value;
            tmp = 500 + ((1200-500)*tmp) / 100;
            video_io_set_level_mv(WHITE, tmp);
            break;
    }

    // store value
    osd_register[reg] = value;
}

static bool serial_tx_active = false;

typedef struct {
    uint8_t data[SERIAL_TX_BUFFER_SIZE];
    uint8_t read;
    uint8_t write;
} serial_tx_buffer_t;

serial_tx_buffer_t serial_tx_buffer;

/*static void serial_putc(char ch) {
    // disable interrupt trigger
    usart_disable_tx_interrupt(SERIAL_UART);

    // interrupt already enabled?
    if (serial_tx_active) {
        // alredy started to send, copy to buffer!
        serial_tx_buffer.data[serial_tx_buffer.write] = ch;
        serial_tx_buffer.write = (serial_tx_buffer.write + 1) % SERIAL_TX_BUFFER_SIZE;

        // check if free space in buffer:
        if (serial_tx_buffer.write == serial_tx_buffer.read) {
            // no more space in buffer! this will loose some data!
            return;
        }
    } else {
        // no int active. send first byte and reset buffer indices
        serial_tx_buffer.write  = serial_tx_buffer.read;

        // mark as tx active
        serial_tx_active = true;

        // enable TXE int
        USART_CR1(SERIAL_UART) |= USART_CR1_TXEIE;

        // start transmission of first char
        USART_TDR(SERIAL_UART) = (ch & USART_TDR_MASK);
    }

    // re enable interrupts
    usart_enable_tx_interrupt(SERIAL_UART);
}*/

void SERIAL_UART_IRQ(void) {
    if (USART_ISR(SERIAL_UART) & USART_ISR_TXE) {
        // tx empty
        if (serial_tx_buffer.read == serial_tx_buffer.write) {
            // no data in fifo -> disable tx int:
            USART_CR1(SERIAL_UART) &= ~(USART_CR1_TXEIE);

            // mark sending as done
            serial_tx_active = false;
        } else {
            // transmit data from fifo:
            USART_TDR(SERIAL_UART) = (serial_tx_buffer.data[serial_tx_buffer.read] & USART_TDR_MASK);

            // handle output pointer
            serial_tx_buffer.read = (serial_tx_buffer.read+1) % SERIAL_TX_BUFFER_SIZE;
        }
    }
}



/*void serial_protocol_read_osd_register(uint8_t reg) {
    // check register address
    if (reg > OPENTCO_MAX_REGISTER) return;

    uint8_t buffer[6];
    buffer[0] = OPENTCO_PROTOCOL_HEADER;
    buffer[1] = (OPENTCO_DEVICE_OSD_RESPONSE << 4) | OPENTCO_OSD_COMMAND_REGISTER_ACCESS;
    buffer[2] = reg;
    buffer[3] = osd_register[reg] & ~OPENTCO_REGISTER_ACCESS_MODE_READ;  // remove read flag 0x80
    buffer[4] = (osd_register[reg] >> 8) & 0xFF;

    uint8_t crc;
    CRC8_INIT(crc, 0);
    for(uint32_t i = 0; i < 5; i++) {
        CRC8_UPDATE(crc, buffer[i]);
    }
    buffer[5] = crc;

    // send response
    for(uint32_t i = 0; i < 6; i++) {
        //usart_send_blocking(SERIAL_UART, buffer[i]);
        serial_putc(buffer[i]);
    }
}*/

#define OPENTCO_TYPE_UINT16   2
#define OPENTCO_TYPE_TEXT_SELECTION 9

static uint8_t opentco_response_buffer[OPENTCO_MAX_DATA_LENGTH];
static uint8_t *opentco_response_buffer_ptr;

static void opentco_response_init_buffer(void) {
    opentco_response_buffer_ptr =  opentco_response_buffer;
}

static void opentco_response_add_uint16(uint16_t val) {
    *opentco_response_buffer_ptr++ = val & 0xFF;
    *opentco_response_buffer_ptr++ = (val >> 8) & 0xFF;
}

static void opentco_response_add_uint8(uint8_t val) {
    *opentco_response_buffer_ptr++ = val;
}

static void opentco_response_add_string(char *val) {
    while (*val) {
        *opentco_response_buffer_ptr++ = *val++;
    }
}


static void opentco_response_store_length(void) {
    opentco_response_buffer[3] = opentco_response_buffer_ptr - &opentco_response_buffer[3] - 1;
}

static void opentco_response_add_crc(void) {
    uint8_t crc;
    CRC8_INIT(crc, 0);
    uint32_t payload_len = opentco_response_buffer_ptr - &opentco_response_buffer[0];

    for(uint32_t i = 0; i < (payload_len); i++) {
        CRC8_UPDATE(crc, opentco_response_buffer[i]);
    }
    opentco_response_add_uint8(crc);
}

static void opentco_response_send(void) {
    uint32_t payload_len = opentco_response_buffer_ptr - &opentco_response_buffer[0];

    for(uint32_t i = 0; i < payload_len; i++) {
        usart_send_blocking(SERIAL_UART, opentco_response_buffer[i]);
    }
}

static void serial_protocol_read_osd_register(uint8_t reg) {
    opentco_response_init_buffer();

    // remove read flag
    reg = reg & ~OPENTCO_REGISTER_ACCESS_MODE_READ;

    // add header
    opentco_response_add_uint8(OPENTCO_PROTOCOL_HEADER);

    // add response code
    opentco_response_add_uint8((OPENTCO_DEVICE_OSD_RESPONSE << 4) | OPENTCO_OSD_COMMAND_REGISTER_ACCESS);

    // add register
    opentco_response_add_uint8(reg);

    // skip length (will be added later)
    opentco_response_add_uint8(0);

    // add 16bit value
    opentco_response_add_uint16(osd_register[reg]);

    // add string value
    switch (reg) {
        default:
            opentco_response_add_string("NO_DESC|");
            break;
    }

    // update length
    opentco_response_store_length();

    // calc and add CRC
    opentco_response_add_crc();

    // send it
    opentco_response_send();
}


void serial_protocol_write_vtx_register(uint8_t reg, uint16_t value) {
    //uint16_t tmp;

    // check register address
    if (reg > OPENTCO_MAX_REGISTER) return;

    switch(reg) {
        default:
            break;

        case (OPENTCO_VTX_REGISTER_STATUS):
            break;

        case (OPENTCO_VTX_REGISTER_BAND):
        case (OPENTCO_VTX_REGISTER_CHANNEL):
            if (vtx_register[OPENTCO_VTX_REGISTER_STATUS] & OPENTCO_VTX_STATUS_ENABLE) {
                // abort if tx is active!
                return;
            }
            vtx_register[reg] = value;
            uint8_t band    = vtx_register[OPENTCO_VTX_REGISTER_BAND];
            uint8_t channel = vtx_register[OPENTCO_VTX_REGISTER_CHANNEL];
            rtc6705_set_band_and_channel(band, channel);
            break;

        case (OPENTCO_VTX_REGISTER_FREQUENCY):
            if (vtx_register[OPENTCO_VTX_REGISTER_STATUS] & OPENTCO_VTX_STATUS_ENABLE) {
                // abort if tx is active!
                return;
            }
            rtc6705_set_frequency(value);
            break;

        case (OPENTCO_VTX_REGISTER_POWER):
            //FIXME: ADD HANDLER
            break;
    }

    // store value
    vtx_register[reg] = value;
}


static void serial_protocol_read_vtx_register(uint8_t reg) {
    opentco_response_init_buffer();

    // remove read flag
    reg = reg & ~OPENTCO_REGISTER_ACCESS_MODE_READ;

    // add header
    opentco_response_add_uint8(OPENTCO_PROTOCOL_HEADER);

    // add response code
    opentco_response_add_uint8((OPENTCO_DEVICE_VTX_RESPONSE << 4) | OPENTCO_VTX_COMMAND_REGISTER_ACCESS);

    // add register
    opentco_response_add_uint8(reg);

    // skip length (will be added later)
    opentco_response_add_uint8(0);

    // add 16bit value
    opentco_response_add_uint16(vtx_register[reg]);

    // add string value
    switch (reg) {
        default:
            opentco_response_add_string("NO_DESC|");
            break;

        case(OPENTCO_VTX_REGISTER_POWER):
            opentco_response_add_string("POWER LEVEL:10 |25 |");
            break;

        case(OPENTCO_VTX_REGISTER_BAND):
            opentco_response_add_string("BAND:Boscam A|Boscam B|Boscam E|FatShark|RaceBand|"); // BE CAREFUL WITH LONG STR!
            break;

        case(OPENTCO_VTX_REGISTER_CHANNEL):
            opentco_response_add_string("CHANNEL:1|2|3|4|5|6|7|8|");
            break;
    }

    // update length
    opentco_response_store_length();

    // calc and add CRC
    opentco_response_add_crc();

    // send it
    opentco_response_send();
}
