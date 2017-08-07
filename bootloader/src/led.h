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

#ifndef LED_H_
#define LED_H_

#include <libopencm3/stm32/gpio.h>
#include "config.h"
#include "macros.h"

void led_init(void);

#define led_on()     { GPIO_TOGGLE(LED_GPIO, LED_PIN); }
#define led_off()    { GPIO_CLEAR(LED_GPIO, LED_PIN); }
#define led_set(__val)     {if (__val) {led_on(); } else {led_off(); }}
#define led_toggle() { GPIO_TOGGLE(LED_GPIO, LED_PIN); }

#endif  // LED_H_
