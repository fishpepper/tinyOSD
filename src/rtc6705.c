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

#include "rtc6705.h"
#include "macros.h"
#include "config.h"
#include "debug.h"
#include "delay.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

static void rtc6705_init_gpio(void);
static void rtc6705_init_rcc(void);

static uint32_t rtc6705_transfer(uint8_t address, uint8_t rw, uint32_t data);
static uint32_t rtc6705_receive_bits(uint8_t n);
static void rtc6705_send_bits(uint8_t n, uint32_t data);

void rtc6705_init(void) {
    debug_function_call();

    rtc6705_init_rcc();
    rtc6705_init_gpio();

    // shift out some dummy clocks:
    RTC6705_CS_HI();
    for(uint8_t i=0; i<32; i++){
        RTC6705_SCK_HI();
        delay_us(1);
        RTC6705_SCK_LO();
        delay_us(1);
    }

#if DEBUG_PRINTS_ENABLED
   for (uint8_t i=0; i<=0xF; i++){
       timeout_delay_ms(100);
       uint32_t res = rtc6705_transfer(i, RTC6705_COMMAND_READ, 0x0000);
       debug("rtc6705 reg 0x"); debug_put_hex8(i);
       debug(" = 0x");
       debug_put_hex32(res); debug_put_newline();
   }
#endif

    // wait for the rtc6705 to be ready
    // TODO: find correct value?!
    timeout_delay_ms(100);

   //F1 5740
   //
   // 2*(N*64+A)*20KHZ = 5740
   //
   // N = 2242, A=12
   //rtc6705_transfer(0x01, RTC6705_COMMAND_WRITE, (2242<<7) | 12);

    //B4 5790
    //
    // 2*(N*64+A)*20KHZ = 5790
    //
    // N = 2261, A=46
    rtc6705_transfer(0x00, RTC6705_COMMAND_WRITE, 0x190); // default, 8MHZ clock / 20khz spacing
    rtc6705_transfer(0x01, RTC6705_COMMAND_WRITE, RTC6705_FREQUENCY_TO_REGVAL(5790)); //(2261<<7) + 46);
}

// A B E F R
static const uint32_t rtc6705_frequency_lookuptable[RTC6705_BAND_COUNT][RTC6705_CHANNEL_COUNT] = {
    // BAND A
    {
        RTC6705_FREQUENCY_TO_REGVAL(5865), //A1
        RTC6705_FREQUENCY_TO_REGVAL(5845), //A2
        RTC6705_FREQUENCY_TO_REGVAL(5825), //A3
        RTC6705_FREQUENCY_TO_REGVAL(5805), //A4
        RTC6705_FREQUENCY_TO_REGVAL(5785), //A5
        RTC6705_FREQUENCY_TO_REGVAL(5765), //A6
        RTC6705_FREQUENCY_TO_REGVAL(5745), //A7
        RTC6705_FREQUENCY_TO_REGVAL(5725), //A8
    },
    // BAND B
    {
        RTC6705_FREQUENCY_TO_REGVAL(5733),  // B1
        RTC6705_FREQUENCY_TO_REGVAL(5752),  // B2
        RTC6705_FREQUENCY_TO_REGVAL(5771),  // B3
        RTC6705_FREQUENCY_TO_REGVAL(5790),  // B4
        RTC6705_FREQUENCY_TO_REGVAL(5809),  // B5
        RTC6705_FREQUENCY_TO_REGVAL(5828),  // B6
        RTC6705_FREQUENCY_TO_REGVAL(5847),  // B7
        RTC6705_FREQUENCY_TO_REGVAL(5866),  // B8
    },
    // BAND E
    {
        RTC6705_FREQUENCY_TO_REGVAL(5705),  // E1
        RTC6705_FREQUENCY_TO_REGVAL(5685),  // E2
        RTC6705_FREQUENCY_TO_REGVAL(5665),  // E3
        RTC6705_FREQUENCY_TO_REGVAL(5645),  // E4
        RTC6705_FREQUENCY_TO_REGVAL(5885),  // E5
        RTC6705_FREQUENCY_TO_REGVAL(5905),  // E6
        RTC6705_FREQUENCY_TO_REGVAL(5925),  // E7
        RTC6705_FREQUENCY_TO_REGVAL(5945),  // E8
    },
    // BAND F
    {
        RTC6705_FREQUENCY_TO_REGVAL(5740),  // F1
        RTC6705_FREQUENCY_TO_REGVAL(5760),  // F2
        RTC6705_FREQUENCY_TO_REGVAL(5780),  // F3
        RTC6705_FREQUENCY_TO_REGVAL(5800),  // F4
        RTC6705_FREQUENCY_TO_REGVAL(5820),  // F5
        RTC6705_FREQUENCY_TO_REGVAL(5840),  // F6
        RTC6705_FREQUENCY_TO_REGVAL(5860),  // F7
        RTC6705_FREQUENCY_TO_REGVAL(5880),  // F8
    },
    // BAND R
    {
        RTC6705_FREQUENCY_TO_REGVAL(5658),  // R1
        RTC6705_FREQUENCY_TO_REGVAL(5695),  // R2
        RTC6705_FREQUENCY_TO_REGVAL(5732),  // R3
        RTC6705_FREQUENCY_TO_REGVAL(5769),  // R4
        RTC6705_FREQUENCY_TO_REGVAL(5806),  // R5
        RTC6705_FREQUENCY_TO_REGVAL(5843),  // R6
        RTC6705_FREQUENCY_TO_REGVAL(5880),  // R7
        RTC6705_FREQUENCY_TO_REGVAL(5917),  // R8
    }
};


void rtc6705_set_band_and_channel(uint8_t band, uint8_t channel) {
    // band is 0, 1, 2, 3, 4 = A B E F R
    band = min(band, RTC6705_BAND_COUNT - 1);
    // channel is 0..7 -> limit
    channel = min(channel, RTC6705_CHANNEL_COUNT - 1);
    // fetch value from lookuptable
    uint32_t regval = rtc6705_frequency_lookuptable[band][channel];

    // default, 8MHZ clock / 20khz spacing
    rtc6705_transfer(0x00, RTC6705_COMMAND_WRITE, 0x190);
    // send A and N
    rtc6705_transfer(0x01, RTC6705_COMMAND_WRITE, regval);
}

void rtc6705_set_frequency(uint16_t f) {
    // do not allow frequencies to low
    f = max(f, RTC6705_FREQUENCY_MIN);
    // do not allow frequencies to high
    f = min(f, RTC6705_FREQUENCY_MAX);

    // default, 8MHZ clock / 20khz spacing
    rtc6705_transfer(0x00, RTC6705_COMMAND_WRITE, 0x190);
    // send A and N
    rtc6705_transfer(0x01, RTC6705_COMMAND_WRITE, RTC6705_FREQUENCY_TO_REGVAL(f));
}

static void rtc6705_init_rcc(void) {
    debug_function_call();

    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(RTC6705_GPIO));
}

static void rtc6705_init_gpio(void) {
    debug_function_call();

    // set MOSI, SCK and CS pin as output
    uint32_t pins = RTC6705_PIN_CS | RTC6705_PIN_SCK | RTC6705_PIN_DATA;

    gpio_mode_setup(RTC6705_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, pins);
    gpio_set_output_options(RTC6705_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, pins);

    // set up default idle levels
    RTC6705_CS_HI();
    RTC6705_SCK_LO();
    RTC6705_DATA_LO();
}


static void rtc6705_send_bits(uint8_t n, uint32_t data) {
    // send n bits of data
    // data has to be right aligned
    while(n--) {
        //delay_us(1);
        asm("NOP");
        RTC6705_SCK_LO();

        // send bit
        if (data & 1) {
            RTC6705_DATA_HI();
        } else {
            RTC6705_DATA_LO();
        }
        data >>= 1;

        // toggle clock:
        //delay_us(1);
        asm("NOP");
        RTC6705_SCK_HI();
    }
}

static uint32_t rtc6705_receive_bits(uint8_t n) {
    uint32_t result = 0;
    // receive n bits of data
    for (uint8_t i=0; i<n; i++) {
        //delay_us(1);
        asm("NOP");
        asm("NOP");
        asm("NOP");
        RTC6705_SCK_LO();

        //delay_us(1);
        asm("NOP");
        asm("NOP");
        asm("NOP");
        RTC6705_SCK_HI();

        // read data
        if (RTC6705_DATA_READ()) {
            result |= (1<<i);
        }
    }

    return result;
}

static uint32_t rtc6705_transfer(uint8_t address, uint8_t rw, uint32_t data) {
    debug_function_call();

    uint32_t result = 0;
    // select device
    RTC6705_CS_LO();
    delay_us(1);

    // send address
    rtc6705_send_bits(4, address);

    // send r/w bit
    rtc6705_send_bits(1, rw);

    // read or write?
    if (rw == RTC6705_COMMAND_READ) {
        RTC6705_DATA_INPUT();
        result = rtc6705_receive_bits(20);
        RTC6705_DATA_OUTPUT();
    } else {
        rtc6705_send_bits(20, data);
    }

    // finish last clock
    RTC6705_SCK_LO();

    // deselect device
    delay_us(1);
    RTC6705_CS_HI();

    return result;
}

#if 0

uint16_t void rtc6705_transfer (uint8_t address, uint8_t rw, uint16_t data) {
    uint16_t result = 0;

    // write or read a given address
    // data is 20 bits left aligned
    // on entry: sck = lo, data = lo, cs = hi

    // select device:
    RTC6705_CS_LO();

    // transfer address
    for(uint8_t i=0; i<4; i++) {
        // clock to low
        RTC6705_SCK_LO();

        // set MOSI state
        if (address & 1)) {
            RTC6705_DATA_HI();
        } else {
            RTC6705_DATA_LO();
        }
        // shift address
        address >>= 1;

        // toggle clock hi
        RTC6705_SCK_HI();
    }

    RTC6705_SCK_LO();

    // read or write?
    if (rw == 1) {
        // read: device will send us data, make sure SPI_DATA is input!
        RTC6705_DATA_HI();
    } else {
        // write
        RTC6705_DATA_LO();
    }

    // toggle clock
    RTC6705_SCK_HI();

    if (rw) {
        // send 20 bits of data
        for(uint8_t i=0; i<20; i++) {
            // clock to low
            RTC6705_SCK_LO();

            // set MOSI state
            if (data & 1) {
                RTC6705_MOSI_HI();
            } else {
                RTC6705_MOSI_LO();
            }
            // shift data
            data >>= 1;

            // toggle clock hi
            RTC6705_SCK_HI();
        }
        // clock back to low level
        RTC6705_SCK_LO();

    }else {
        // we will read data, gpio has to be input
        RTC6705_DATA_INPUT();

        // read reply
        for(uint8_t i=0; i<20; i++) {
            // clock to low
            RTC6705_SCK_LO();

            // read reply
            if (RTC6705_DATA_READ()) {
                result |= (1<<24);
            }
            result >>= 1;


            // toggle clock hi
            RTC6705_SCK_HI();
        }

        // clock back to low level
        RTC6705_SCK_LO();

        // revert data pin back to output
        RTC6705_DATA_OUTPUT();
    }


    // make sure data is output again
    if (rw) {
    }

    // deselect device:
    RTC6705_CS_HI();


    // restore idle state
    RTC6705_SCK_LO();
    RTC6705_MOSI_LO();
}

#endif
