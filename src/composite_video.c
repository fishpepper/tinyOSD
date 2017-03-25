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

#include "composite_video.h"
#include "config.h"
#include "macros.h"
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/comparator.h>
#include <libopencm3/stm32/dac.h>

static void composite_video_init_rcc(void);
static void composite_video_init_gpio(void);
static void composite_video_init_init_comparator(void);
static void composite_video_init_dac(void);

static void composite_video_set_dac_value(uint16_t target);

void composite_video_init(void) {
    composite_video_init_rcc();
    composite_video_init_gpio();
    composite_video_init_init_comparator();
    composite_video_init_dac();

    // set to half value
    composite_video_set_dac_value(0x0FFF/2);
}

static void composite_video_init_rcc(void) {
    // DAC clock
    rcc_periph_clock_enable(RCC_DAC);


    // peripheral clocks enable
    rcc_periph_clock_enable(GPIO_RCC(COMPOSITE_VIDEO_GPIO));
}

static void composite_video_init_gpio(void) {
    // set video input pin as input
    gpio_mode_setup(COMPOSITE_VIDEO_GPIO, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, COMPOSITE_VIDEO_INPUT_PIN);

    // set dac to output
    gpio_mode_setup(COMPOSITE_VIDEO_GPIO, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, COMPOSITE_VIDEO_DAC_OUT_PIN);
    gpio_set_output_options(COMPOSITE_VIDEO_GPIO, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, COMPOSITE_VIDEO_DAC_OUT_PIN);
}

static void composite_video_init_init_comparator(void) {
    // use comparator 1
    // inp = PA1
    // inm = DAC_OUT_1 (PA4) ->
    // COMP1
}

static void composite_video_init_dac(void) {
    // start with disabled dac
    dac_disable(CHANNEL_1);
    dac_disable_waveform_generation(CHANNEL_1);

    // set default value and enable output
    composite_video_set_dac_value(0);
    dac_enable(CHANNEL_1);

    // software update trigher
    dac_set_trigger_source(DAC_CR_TSEL1_SW);
}

static void composite_video_set_dac_value(uint16_t target) {
    dac_load_data_buffer_single(target, RIGHT12, CHANNEL_1);
    dac_software_trigger(CHANNEL_1);
}

