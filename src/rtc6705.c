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

   for (uint8_t i=0; i<=0xF; i++){
       timeout_delay_ms(100);
       uint32_t res = rtc6705_transfer(i, RTC6705_COMMAND_READ, 0x0000);
       debug("rtc6705 reg 0x"); debug_put_hex8(i);
       debug(" = 0x");
       debug_put_hex32(res); debug_put_newline();
   }

    // wait for the rtc6705 to be ready
    // TODO: find correct value?!
    //timeout_delay_ms(300);

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
    rtc6705_transfer(0x01, RTC6705_COMMAND_WRITE, (2261<<7) + 46);


//1953 = 5GHZ
//2343 = 6 GHZ
/*
    for(uint16_t i=1953; i<2343; i+=(2243-1953)/60) {
        timeout_delay_ms(2000);
        rtc6705_transfer(0x00, RTC6705_COMMAND_WRITE, 0x190); // default, 8MHZ clock / 20khz spacing
        rtc6705_transfer(0x01, RTC6705_COMMAND_WRITE, (i<<7));
        debug_put_uint16(i); debug_put_newline();
    }*/
  // while(1);
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
