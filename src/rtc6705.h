/*
    Copyright 2016 fishpepper <AT> gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    author: fishpepper <AT> gmail.com
*/

#ifndef RTC6705_H_
#define RTC6705_H_

#include <stdint.h>

void rtc6705_init(void);

void rtc6705_set_frequency(uint16_t f);
void rtc6705_set_band_and_channel(uint8_t band, uint8_t channel);

#define RTC6705_CS_HI() { GPIO_SET(RTC6705_GPIO, RTC6705_PIN_CS); }
#define RTC6705_CS_LO() { GPIO_CLEAR(RTC6705_GPIO, RTC6705_PIN_CS); }

#define RTC6705_SCK_HI() { GPIO_SET(RTC6705_GPIO, RTC6705_PIN_SCK); }
#define RTC6705_SCK_LO() { GPIO_CLEAR(RTC6705_GPIO, RTC6705_PIN_SCK); }

#define RTC6705_DATA_HI() { GPIO_SET(RTC6705_GPIO, RTC6705_PIN_DATA); }
#define RTC6705_DATA_LO() { GPIO_CLEAR(RTC6705_GPIO, RTC6705_PIN_DATA); }

#define RTC6705_DATA_OUTPUT() { GPIO_MODER(RTC6705_GPIO) |= (1 << (2*RTC6705_PIN_DATA_PIN_ID)); }
#define RTC6705_DATA_INPUT()  { GPIO_MODER(RTC6705_GPIO) &= ~(3 << (2*RTC6705_PIN_DATA_PIN_ID)); }

#define RTC6705_DATA_READ() (GPIO_IDR(RTC6705_GPIO) & RTC6705_PIN_DATA)

#define RTC6705_COMMAND_READ  0
#define RTC6705_COMMAND_WRITE 1

#define RTC6705_FREQUENCY_MIN 5600
#define RTC6705_FREQUENCY_MAX 5950

#define RTC6705_FREQ_TO_A(__f) (((((uint64_t)(__f)*(uint64_t)1000000*(uint64_t)400)/(uint64_t)1000000) % 1024) / 16)
#define RTC6705_FREQ_TO_N(__f) ((((uint64_t)(__f)*(uint64_t)1000000*(uint64_t)400)/(uint64_t)1000000) / 1024)
#define RTC6705_FREQUENCY_TO_REGVAL(__f) ((RTC6705_FREQ_TO_N(__f)<<7) | RTC6705_FREQ_TO_A(__f))

#define RTC6705_BAND_COUNT      5  // A B E F R
#define RTC6705_CHANNEL_COUNT   8  // 1 2 3 4 5 6 7 8


#endif  // RTC6705_H_
