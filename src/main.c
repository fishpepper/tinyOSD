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
#include "adc.h"
#include "rtc6705.h"
#include "macros.h"

#include <stdlib.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>


// STM32F301 reference manual DM00094350.pdf
// http://www.st.com/resource/en/reference_manual/dm00094350.pdf
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/rcc.h>
int main(void) {
#if 0
    rcc_periph_clock_enable(GPIO_RCC(VIDEO_GPIO));
    rcc_periph_clock_enable(RCC_DAC1);
    // set dac to output
    gpio_mode_setup(VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, VIDEO_DAC_OUT_PIN);
    //gpio_set_output_options(VIDEO_GPIO, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, VIDEO_DAC_OUT_PIN);


    // start with disabled dac
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);
    dac_buffer_disable(CHANNEL_1);

    // set default value and enable output

    dac_enable(CHANNEL_1);

    // software update trigher
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
//invert??https://www.mikrocontroller.net/topic/404156
clocksource_init();
    uint16_t i=0;
    while(1){

        dac_load_data_buffer_single(i+=10, RIGHT12, CHANNEL_1);
        if (i>500) i = 0;
        dac_software_trigger(CHANNEL_1);
        for(uint8_t j=0; j<10; j++) delay_us(100);
    }

    while(1);
#endif
    // init crystal osc & set clock options
    clocksource_init();

    timeout_init();
    delay_init();

    led_init();

    debug_init();

    rtc6705_init();

    adc_init();

    video_init();

    serial_init();

    // this will never exit...
    video_main_loop();

    while(1){
        led_toggle();
        timeout_delay_ms(100);
    }
}
