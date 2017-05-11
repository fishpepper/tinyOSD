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

#include "main.h"
#include "delay.h"
#include "timeout.h"
#include "led.h"
#include "clocksource.h"
#include "debug.h"
#include "video.h"
#include "serial.h"

#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>


int main(void) {
    // init crystal osc & set clock options
    clocksource_init();

    timeout_init();
    delay_init();

    led_init();

    debug_init();

    video_init();

    serial_init();

    // this will never exit...
    video_main_loop();
}

